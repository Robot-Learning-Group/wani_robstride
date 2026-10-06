"""Deterministic diagnostic tests: no CAN devices, sockets, or network."""

from collections import deque
import gc
import struct
import threading

import pytest

can = pytest.importorskip("can")
sdk = pytest.importorskip("robstride_dynamics")
pytest.importorskip("robstride_dynamics.protocol")

from robstride_dynamics import position as diagnostic
from robstride_dynamics.position import PositionReader

RobstrideBus = sdk.RobstrideBus


class Clock:
    def __init__(self):
        self.now = 10.0

    def monotonic(self):
        return self.now


class FakeTransport:
    def __init__(self, clock):
        self.clock = clock
        self.queued = deque()
        self.replies = []
        self.sent = []
        self.received = []
        self.send_cost = 0.0
        self.recv_cost = 0.001
        self.send_error = None
        self.recv_error = None
        self.shutdown_error = None
        self.shutdown_calls = 0
        self.shutdown_hook = None
        self.recv_hook = None
        self.flood = None

    def send(self, frame, timeout):
        self.sent.append((frame, timeout))
        self.clock.now += self.send_cost
        if self.send_error:
            raise self.send_error
        replies = self.replies(frame) if callable(self.replies) else self.replies
        self.queued.extend(replies)

    def recv(self, timeout):
        self.received.append(timeout)
        if self.recv_error:
            raise self.recv_error
        if self.recv_hook:
            self.recv_hook(timeout)
        if self.queued:
            self.clock.now += self.recv_cost
            return self.queued.popleft()
        if self.flood:
            self.clock.now += self.recv_cost
            return self.flood
        self.clock.now += timeout
        return None

    def shutdown(self):
        self.shutdown_calls += 1
        if self.shutdown_hook:
            self.shutdown_hook()
        if self.shutdown_error:
            raise self.shutdown_error


def reply(value=1.25, *, motor=1, host=0xFF, register=0x7019, kind=17, **kwargs):
    return can.Message(
        arbitration_id=(kind << 24) | (motor << 8) | host,
        is_extended_id=True,
        data=struct.pack("<HHf", register, 0, value),
        **kwargs,
    )


@pytest.fixture
def setup(monkeypatch):
    clock = Clock()
    transport = FakeTransport(clock)
    factory_calls = []

    def factory(**kwargs):
        factory_calls.append(kwargs)
        return transport

    def forbidden(*args, **kwargs):
        pytest.fail("Read-only diagnostics must never use RobstrideBus")

    monkeypatch.setattr(diagnostic.time, "monotonic", clock.monotonic)
    monkeypatch.setattr(can.interface, "Bus", factory)
    monkeypatch.setattr(can, "Bus", forbidden)
    for name in ("__init__", "scan_channel", "enable", "disable", "write",
                 "transmit", "disconnect"):
        monkeypatch.setattr(RobstrideBus, name, forbidden)
    reader = PositionReader("fake-can", motor_id=1)
    yield reader, transport, clock, factory_calls
    reader.close()


def connect_with(setup, frames):
    reader, transport, _, _ = setup
    transport.replies = frames
    reader.connect()
    return reader, transport


def assert_poisoned(reader, transport):
    counts = (len(transport.sent), len(transport.received))
    with pytest.raises(RuntimeError, match="Failed session"):
        reader.read_position()
    with pytest.raises(RuntimeError, match="Failed session"):
        reader.read_settings()
    with pytest.raises(RuntimeError, match="Already connected"):
        reader.connect()
    assert counts == (len(transport.sent), len(transport.received))


def test_public_api_and_exact_read_only_request(setup):
    reader, transport, _, calls = setup
    assert not calls and not transport.sent
    reader.connect()
    assert calls == [{"interface": "socketcan", "channel": "fake-can",
                      "bitrate": 1000000, "ignore_config": True}]
    transport.replies = [reply(-2.5)]
    value = reader.read_position()
    assert type(value) is float and value == -2.5
    frame, timeout = transport.sent[0]
    assert frame.arbitration_id == 0x1100FF01
    assert frame.data == bytes.fromhex("1970000000000000")
    assert frame.is_extended_id and frame.dlc == 8
    assert not (frame.is_fd or frame.is_remote_frame or frame.is_error_frame)
    assert timeout == pytest.approx(0.1)
    assert transport.received[0] == 0.0
    reader.close()
    reader.close()
    assert transport.shutdown_calls == 1
    assert len(transport.sent) == 1


def test_custom_host_motor_and_bitrate(setup):
    _, transport, _, calls = setup
    reader = PositionReader("fake-can", motor_id=7, bitrate=500000, host_id=3)
    reader.connect()
    transport.replies = [reply(motor=7, host=3)]
    try:
        assert reader.read_position() == 1.25
        assert transport.sent[0][0].arbitration_id == 0x11000307
        assert calls == [{"interface": "socketcan", "channel": "fake-can",
                          "bitrate": 500000, "ignore_config": True}]
    finally:
        reader.close()


@pytest.mark.parametrize("unrelated", [
    reply(motor=2), reply(host=0xFE), reply(register=0x701B),
    reply(kind=2), reply(kind=21, motor=2), reply(kind=21, host=0xFE),
    can.Message(arbitration_id=0x123, is_extended_id=False, data=b""),
    can.Message(arbitration_id=0, is_error_frame=True),
    reply(motor=2, is_fd=True), reply(motor=2, dlc=7),
])
def test_unrelated_frames_ignored_until_correlated_reply(setup, unrelated):
    reader, _ = connect_with(setup, [unrelated, reply(3.5)])
    assert reader.read_position() == 3.5


@pytest.mark.parametrize("change", [
    {"is_extended_id": False}, {"is_error_frame": True},
    {"is_fd": True}, {"is_remote_frame": True},
    {"bitrate_switch": True}, {"error_state_indicator": True},
    {"dlc": 7}, {"dlc": 9}, {"data": b""}, {"data": b"\x19"},
    {"data": bytes.fromhex("19700000")},
    {"data": bytes.fromhex("197000000000000000")},
    {"data": struct.pack("<HHf", 0x7019, 1, 1.0)},
    {"arbitration_id": 0x110101FF},
    {"arbitration_id": 0x310001FF},
])
def test_malformed_target_reply_fails_and_poisons(setup, change):
    frame = reply()
    for field, value in change.items():
        setattr(frame, field, value)
    reader, transport = connect_with(setup, [frame, reply()])
    with pytest.raises(ValueError, match="Malformed"):
        reader.read_position()
    assert_poisoned(reader, transport)


@pytest.mark.parametrize("value", [float("nan"), float("inf"), -float("inf")])
def test_nonfinite_position_rejected(setup, value):
    reader, transport = connect_with(setup, [reply(value)])
    with pytest.raises(ValueError, match="Nonfinite"):
        reader.read_position()
    assert_poisoned(reader, transport)


def test_matching_fault_fails(setup):
    fault = reply(kind=21)
    fault.data = bytearray(struct.pack("<LL", 4, 1))
    reader, transport = connect_with(setup, [fault, reply()])
    with pytest.raises(RuntimeError, match="fault=0x00000004, warning=0x00000001"):
        reader.read_position()
    assert_poisoned(reader, transport)


def test_malformed_matching_fault_fails(setup):
    reader, transport = connect_with(setup, [reply(kind=21, dlc=3)])
    with pytest.raises(ValueError, match="Malformed"):
        reader.read_position()
    assert_poisoned(reader, transport)


def test_queued_stale_reply_is_drained_before_send(setup):
    reader, transport = connect_with(setup, [reply(4.0)])
    transport.queued.extend([reply(99.0), reply(motor=2), reply(kind=21)])
    assert reader.read_position() == 4.0
    assert transport.received[:4] == [0.0] * 4
    assert transport.sent[0][1] == pytest.approx(0.097)


def test_drain_frame_limit_no_request_sent(setup):
    reader, transport = connect_with(setup, [reply()])
    transport.recv_cost = 0.0
    transport.flood = reply(motor=2)
    with pytest.raises(RuntimeError, match="256 frames"):
        reader.read_position(timeout_s=0.1)
    assert len(transport.received) == 256
    assert not transport.sent
    assert_poisoned(reader, transport)


def test_drain_uses_request_deadline(setup):
    reader, transport = connect_with(setup, [reply()])
    transport.flood = reply(motor=2)
    with pytest.raises(TimeoutError):
        reader.read_position(timeout_s=0.01)
    assert len(transport.received) < 20
    assert not transport.sent
    assert_poisoned(reader, transport)


def test_unrelated_receive_flood_bounded_by_deadline(setup):
    reader, transport, clock, _ = setup
    reader.connect()
    transport.replies = [reply(motor=2)] * 1000
    start = clock.now
    with pytest.raises(TimeoutError):
        reader.read_position(timeout_s=0.01)
    assert clock.now - start < 0.012
    assert len(transport.received) < 20
    assert len(transport.sent) == 1
    assert_poisoned(reader, transport)


@pytest.mark.parametrize("frames", [[], [reply(motor=2)], [reply(host=3)],
                                    [reply(register=0x701B)]])
def test_silence_or_only_wrong_replies_times_out(setup, frames):
    reader, transport, clock, _ = setup
    reader.connect()
    transport.replies = frames
    start = clock.now
    with pytest.raises(TimeoutError):
        reader.read_position(timeout_s=0.1)
    assert clock.now - start == pytest.approx(0.1)
    assert_poisoned(reader, transport)


def test_send_and_receive_share_remaining_budget(setup):
    reader, transport = connect_with(setup, [reply(motor=2), reply()])
    transport.queued.append(reply(90))
    transport.send_cost = 0.04
    assert reader.read_position(timeout_s=0.1) == 1.25
    assert transport.sent[0][1] == pytest.approx(0.099)
    assert transport.received[2:] == pytest.approx([0.059, 0.058])


def test_send_deadline_overrun_cannot_accept_reply(setup):
    reader, transport = connect_with(setup, [reply()])
    transport.send_cost = 0.11
    with pytest.raises(TimeoutError):
        reader.read_position(timeout_s=0.1)
    assert transport.received == [0.0]
    assert_poisoned(reader, transport)


def test_reply_arriving_after_deadline_rejected(setup):
    reader, transport = connect_with(setup, [reply()])
    transport.recv_cost = 0.11
    with pytest.raises(TimeoutError):
        reader.read_position(timeout_s=0.1)
    assert_poisoned(reader, transport)


@pytest.mark.parametrize("stage", ["send", "drain", "receive"])
def test_transport_failure_poisons(setup, stage):
    reader, transport = connect_with(setup, [reply()])
    error = can.CanOperationError("fake transport failure")
    if stage == "send":
        transport.send_error = error
    elif stage == "drain":
        transport.recv_error = error
    else:
        def fail_on_receive(timeout):
            if timeout:
                raise error
        transport.recv_hook = fail_on_receive
    with pytest.raises(can.CanOperationError, match="fake transport failure"):
        reader.read_position()
    assert_poisoned(reader, transport)


def test_explicit_close_reconnect_recovers_and_drains_late_reply(setup):
    reader, transport = connect_with(setup, [])
    with pytest.raises(TimeoutError):
        reader.read_position()
    transport.queued.append(reply(999))
    reader.close()
    reader.connect()
    transport.replies = [reply(2.0)]
    assert reader.read_position() == 2.0


def test_successful_session_reusable_with_drain_on_each_read(setup):
    reader, transport = connect_with(setup, [reply()])
    assert reader.read_position() == 1.25
    transport.queued.append(reply(99))
    transport.replies = [reply(2)]
    assert reader.read_position() == 2
    assert len(transport.sent) == 2


def test_close_clears_handler_before_shutdown_even_on_failure(setup):
    reader, transport = connect_with(setup, [])

    def check_detached():
        assert reader.channel_handler is None
    transport.shutdown_hook = check_detached
    transport.shutdown_error = can.CanOperationError("shutdown failed")
    with pytest.raises(can.CanOperationError, match="shutdown failed"):
        reader.close()
    assert reader.channel_handler is None
    reader.close()
    assert transport.shutdown_calls == 1
    assert not transport.sent
    with pytest.raises(RuntimeError, match="Not connected"):
        reader.read_position()


def test_connect_failure_leaves_no_handler_and_no_commands(setup, monkeypatch):
    reader, transport, _, _ = setup

    def fail(**kwargs):
        raise can.CanInitializationError("open failed")
    monkeypatch.setattr(can.interface, "Bus", fail)
    with pytest.raises(can.CanInitializationError, match="open failed"):
        reader.connect()
    assert reader.channel_handler is None
    reader.close()
    assert not transport.sent


def test_lifecycle_and_no_destructor_motor_commands(setup):
    reader, transport, _, _ = setup
    with pytest.raises(RuntimeError, match="Not connected"):
        reader.read_position()
    reader.close()
    reader.connect()
    with pytest.raises(RuntimeError, match="Already connected"):
        reader.connect()
    assert "__del__" not in PositionReader.__dict__
    other = PositionReader("fake-can", motor_id=1)
    other.connect()
    del other
    gc.collect()
    assert not transport.sent
    assert transport.shutdown_calls == 0


@pytest.mark.parametrize("timeout", [0, -1, float("nan"), float("inf"),
                                     -float("inf"), None, "0.1", True])
def test_invalid_timeout_does_not_poison_or_touch_transport(setup, timeout):
    reader, transport = connect_with(setup, [reply()])
    with pytest.raises((TypeError, ValueError), match="timeout_s"):
        reader.read_position(timeout_s=timeout)
    assert not transport.sent and not transport.received
    assert reader.read_position() == 1.25


@pytest.mark.parametrize("kwargs", [
    {"channel": ""}, {"channel": None}, {"motor_id": None},
    {"motor_id": 0}, {"motor_id": 256},
    {"motor_id": True}, {"motor_id": 1.0},
    {"host_id": -1}, {"host_id": 256}, {"host_id": True},
    {"bitrate": 0}, {"bitrate": 1.5},
])
def test_invalid_configuration_no_transport_open(setup, kwargs):
    _, transport, _, calls = setup
    args = {"channel": "fake-can", "motor_id": 1}
    args.update(kwargs)
    with pytest.raises((ValueError, TypeError)):
        PositionReader(**args)
    assert not calls and not transport.sent


def test_concurrent_operations_rejected_without_poisoning(setup):
    reader, transport = connect_with(setup, [reply()])
    entered = threading.Event()
    release = threading.Event()
    results = []
    errors = []

    def block_receive(timeout):
        if timeout:
            entered.set()
            assert release.wait(2), "test worker was not released"

    def read_in_thread():
        try:
            results.append(reader.read_position())
        except BaseException as error:
            errors.append(error)

    transport.recv_hook = block_receive
    thread = threading.Thread(target=read_in_thread)
    thread.start()
    try:
        assert entered.wait(2), "test worker did not enter receive"
        for operation in (reader.read_position, reader.read_settings,
                          reader.connect, reader.close):
            with pytest.raises(RuntimeError, match="in progress"):
                operation()
        assert len(transport.sent) == 1
        assert transport.shutdown_calls == 0
    finally:
        release.set()
        thread.join(2)
    assert not thread.is_alive()
    assert errors == [] and results == [1.25]
    transport.recv_hook = None
    assert reader.read_position() == 1.25


SETTINGS = (
    ("run_mode", 0x7005, "<B", 5),
    ("velocity_limit_rad_s", 0x7017, "<f", 12.5),
    ("current_limit_a", 0x7018, "<f", 3.25),
    ("torque_limit_nm", 0x700B, "<f", 1.5),
    ("can_timeout_raw", 0x7028, "<I", 0xFEDCBA98),
    ("zero_state", 0x7029, "<B", 1),
    ("raw_motor_position_rad", 0x7019, "<f", -2.5),
)


def settings_reply(register, format, value):
    frame = reply(register=register)
    # Nonzero unused bytes prove uint8 decoding consumes only one byte.
    payload = struct.pack(format, value).ljust(4, b"\xa5")
    frame.data = bytearray(struct.pack("<HH", register, 0) + payload)
    return frame


def connect_settings(setup, overrides=None):
    reader, transport, _, _ = setup
    overrides = overrides or {}

    def respond(request):
        parameter, = struct.unpack_from("<H", request.data)
        for key, register, format, value in SETTINGS:
            if parameter == register:
                return [settings_reply(register, format, overrides.get(key, value))]
        pytest.fail(f"Unexpected register 0x{parameter:04x}")

    transport.replies = respond
    reader.connect()
    return reader, transport


def test_settings_keys_types_sequence_endianness_and_read_only_close(setup):
    reader, transport = connect_settings(setup)
    result = reader.read_settings()
    assert list(result) == [key for key, _, _, _ in SETTINGS]
    assert result == {key: value for key, _, _, value in SETTINGS}
    for key, _, format, _ in SETTINGS:
        assert type(result[key]) is (float if format == "<f" else int)
    assert result["can_timeout_raw"] > 2**31
    assert len(transport.sent) == 7
    for (frame, timeout), (_, register, _, _) in zip(transport.sent, SETTINGS):
        assert frame.arbitration_id == 0x1100FF01
        assert frame.data == struct.pack("<HHL", register, 0, 0)
        assert frame.is_extended_id and frame.dlc == 8
        assert not (frame.is_fd or frame.is_remote_frame or frame.is_error_frame)
        assert timeout == pytest.approx(0.1)
    reader.close()
    reader.close()
    assert transport.shutdown_calls == 1
    assert len(transport.sent) == 7


@pytest.mark.parametrize("mode", [0, 1, 2, 3, 5])
@pytest.mark.parametrize("zero_state", [0, 1])
def test_supported_mode_and_zero_state(setup, mode, zero_state):
    reader, _ = connect_settings(setup, {"run_mode": mode, "zero_state": zero_state})
    result = reader.read_settings()
    assert result["run_mode"] == mode
    assert result["zero_state"] == zero_state


@pytest.mark.parametrize("key,value,count,message", [
    ("run_mode", 4, 1, "Unsupported run_mode raw value: 4"),
    ("run_mode", 255, 1, "Unsupported run_mode raw value: 255"),
    ("zero_state", 2, 6, "Unsupported zero_state raw value: 2"),
    ("zero_state", 255, 6, "Unsupported zero_state raw value: 255"),
    ("velocity_limit_rad_s", -0.5, 2, "Negative velocity_limit_rad_s"),
    ("current_limit_a", -0.5, 3, "Negative current_limit_a"),
    ("torque_limit_nm", -0.5, 4, "Negative torque_limit_nm"),
])
def test_invalid_settings_stop_without_partial_success_and_poison(
    setup, key, value, count, message
):
    reader, transport = connect_settings(setup, {key: value})
    with pytest.raises(ValueError, match=message):
        reader.read_settings()
    assert len(transport.sent) == count
    assert_poisoned(reader, transport)


@pytest.mark.parametrize("key", ["velocity_limit_rad_s", "current_limit_a",
                                 "torque_limit_nm", "raw_motor_position_rad"])
@pytest.mark.parametrize("value", [float("nan"), float("inf"), -float("inf")])
def test_settings_nonfinite_float_fails_and_poisons(setup, key, value):
    reader, transport = connect_settings(setup, {key: value})
    with pytest.raises(ValueError, match="Nonfinite"):
        reader.read_settings()
    assert len(transport.sent) == next(
        index for index, (name, _, _, _) in enumerate(SETTINGS, 1) if name == key
    )
    assert_poisoned(reader, transport)


@pytest.mark.parametrize("raw_timeout", [0, 0x01020304, 0xFFFFFFFF])
def test_zero_limits_and_raw_uint32_timeout_no_defaults(setup, raw_timeout):
    reader, _ = connect_settings(setup, {
        "velocity_limit_rad_s": 0.0, "current_limit_a": 0.0,
        "torque_limit_nm": 0.0, "can_timeout_raw": raw_timeout,
    })
    result = reader.read_settings()
    assert result["can_timeout_raw"] == raw_timeout
    assert result["velocity_limit_rad_s"] == 0.0
    assert result["current_limit_a"] == 0.0
    assert result["torque_limit_nm"] == 0.0


def test_settings_each_register_has_separate_deadline(setup):
    reader, transport = connect_settings(setup)
    clock = setup[2]
    transport.recv_cost = 0.09
    start = clock.now
    assert reader.read_settings(timeout_s=0.1)["run_mode"] == 5
    assert clock.now - start == pytest.approx(7 * 0.09)
    assert [timeout for _, timeout in transport.sent] == pytest.approx([0.1] * 7)


@pytest.mark.parametrize("failure", ["timeout", "transport", "malformed", "fault"])
def test_settings_failure_halfway_poisoned_until_reconnect(setup, failure):
    reader, transport = connect_settings(setup)
    respond = transport.replies

    def fail_halfway(request):
        parameter, = struct.unpack_from("<H", request.data)
        if parameter != 0x700B:
            return respond(request)
        if failure == "timeout":
            return []
        if failure == "transport":
            raise can.CanOperationError("halfway failure")
        if failure == "malformed":
            return [reply(register=parameter, dlc=7)]
        frame = reply(kind=21)
        frame.data = bytearray(struct.pack("<LL", 4, 1))
        return [frame]

    transport.replies = fail_halfway
    error = {"timeout": TimeoutError, "transport": can.CanOperationError,
             "malformed": ValueError, "fault": RuntimeError}[failure]
    with pytest.raises(error):
        reader.read_settings()
    assert len(transport.sent) == 4
    assert_poisoned(reader, transport)
    reader.close()
    reader.connect()
    transport.replies = respond
    assert reader.read_settings()["torque_limit_nm"] == 1.5
    assert len(transport.sent) == 11
    assert all(frame.arbitration_id >> 24 == 17 for frame, _ in transport.sent)


@pytest.mark.parametrize("timeout", [0, -1, float("nan"), float("inf"),
                                     -float("inf"), None, "0.1", True])
def test_settings_invalid_timeout_does_not_poison_or_touch_transport(setup, timeout):
    reader, transport = connect_settings(setup)
    with pytest.raises((TypeError, ValueError), match="timeout_s"):
        reader.read_settings(timeout_s=timeout)
    assert not transport.sent and not transport.received
    assert reader.read_settings()["run_mode"] == 5


def test_settings_rejects_lifecycle_and_concurrency_without_commands(setup):
    reader, transport, _, _ = setup
    with pytest.raises(RuntimeError, match="Not connected"):
        reader.read_settings()
    assert not transport.sent and not transport.received
    reader, transport = connect_settings(setup)

    def check_locked(timeout):
        if timeout:
            for operation in (reader.read_settings, reader.read_position,
                              reader.connect, reader.close):
                with pytest.raises(RuntimeError, match="in progress"):
                    operation()

    transport.recv_hook = check_locked
    assert reader.read_settings()["run_mode"] == 5
    assert len(transport.sent) == 7
    assert transport.shutdown_calls == 0


def test_settings_ignores_wrong_host_motor_and_register(setup):
    reader, transport = connect_settings(setup)
    respond = transport.replies

    def unrelated_then_matching(request):
        parameter, = struct.unpack_from("<H", request.data)
        return [reply(register=parameter, host=3),
                reply(register=parameter, motor=2),
                reply(register=0xFFFF)] + respond(request)

    transport.replies = unrelated_then_matching
    assert reader.read_settings() == {key: value for key, _, _, value in SETTINGS}
