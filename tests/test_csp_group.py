"""Hardware-free group protocol/deadline tests; no CAN sockets are opened."""
from collections import deque
import math
import struct

import can
import pytest

from robstride_dynamics import csp_group
from robstride_dynamics.csp_group import CspGroup
from robstride_dynamics.table import MODEL_MIT_POSITION_TABLE


LIMIT = dict(velocity_limit_rad_s=1., current_limit_a=2., torque_limit_nm=3.,
             position_tolerance_rad=.01, position_min_rad=-1.,
             position_max_rad=1., expected_position_rad=.25)
REGISTERS = [0x7005, 0x7017, 0x7018, 0x700B, 0x7028, 0x7029, 0x7019]
MODEL = "rs-02"


def encode_status_position(position, model=MODEL):
    if not math.isfinite(position):
        return 0
    return round((position / MODEL_MIT_POSITION_TABLE[model] + 1) * 0x7FFF)


def status(motor, mode=0, faults=0, kind=2, temperature_u16=0,
           position_u16=0, velocity_u16=0, torque_u16=0):
    return can.Message(arbitration_id=(kind << 24) | (mode << 22) |
                       (faults << 16) | (motor << 8) | 255,
                       is_extended_id=True,
                       data=struct.pack(">HHHH", position_u16, velocity_u16,
                                        torque_u16, temperature_u16))


def reply(motor, register, value, fmt="<f", reserved=0):
    return can.Message(arbitration_id=(17 << 24) | (motor << 8) | 255,
                       is_extended_id=True, data=struct.pack("<HH", register, reserved) +
                       struct.pack(fmt, value).ljust(4, b"\0"))


class Clock:
    def __init__(self):
        self.now = 0.

    def monotonic(self):
        return self.now


class GroupBus:
    def __init__(self, clock, count=2):
        self.clock = clock
        self.values = {motor: {0x7005: 0, 0x7017: 4., 0x7018: 5., 0x700B: 6.,
                              0x7028: 20000, 0x7029: 1, 0x7019: .25, 0x7016: 0.}
                       for motor in range(1, count + 1)}
        self.modes = {motor: 0 for motor in self.values}
        self.queue = deque()
        self.events = []
        self.sent = []
        self.hook = None
        self.recv_hook = None
        self.send_cost = .00001
        self.recv_cost = .00001
        self.enable_delay = 0.
        self.reverse = False
        self.flood = None
        self.shutdown_calls = 0
        self.open_kwargs = None

    def send(self, frame, timeout):
        motor = frame.arbitration_id & 255
        kind = frame.arbitration_id >> 24
        register = struct.unpack_from("<H", frame.data)[0] if kind in (17, 18) else None
        self.events.append(("send", kind, motor, register, timeout))
        self.sent.append(frame)
        self.clock.now += min(self.send_cost, timeout)
        if self.send_cost > timeout:
            raise TimeoutError("send timed out")
        if kind == 17:
            fmt = "<B" if register in (0x7005, 0x7029) else "<I" if register == 0x7028 else "<f"
            frames = [reply(motor, register, self.values[motor][register], fmt)]
        else:
            if kind == 18:
                fmt = "<B" if register == 0x7005 else "<f"
                self.values[motor][register], = struct.unpack_from(fmt, frame.data, 4)
            elif kind == 3:
                self.modes[motor] = 2
            elif kind == 4:
                self.modes[motor] = 0
            frames = [status(motor, self.modes[motor],
                             position_u16=encode_status_position(
                                 self.values[motor][0x7019]))]
        if self.hook:
            frames = self.hook(frame, frames)
        for response in frames:
            entry = (self.clock.now + (self.enable_delay if kind == 3 else 0.), response)
            if self.reverse:
                self.queue.appendleft(entry)
            else:
                self.queue.append(entry)

    def recv(self, timeout):
        self.events.append(("recv", timeout))
        if self.recv_hook:
            self.recv_hook(timeout)
        self.clock.now += min(timeout, self.recv_cost)
        if self.queue:
            ready, frame = self.queue[0]
            wait = max(0., ready - self.clock.now)
            if wait > timeout:
                self.clock.now += timeout
                return None
            self.clock.now += wait
            return self.queue.popleft()[1]
        if self.flood is not None:
            return self.flood
        self.clock.now += timeout
        return None

    def shutdown(self):
        self.shutdown_calls += 1


@pytest.fixture
def rig(monkeypatch):
    clock = Clock()
    bus = GroupBus(clock)
    monkeypatch.setattr(csp_group.time, "monotonic", clock.monotonic)

    def open_bus(**kwargs):
        bus.open_kwargs = kwargs
        return bus

    monkeypatch.setattr(can.interface, "Bus", open_bus)
    group = CspGroup("fake", {"a": 1, "b": 2},
                     motor_models={"a": MODEL, "b": MODEL})
    group.connect()
    yield group, bus, clock
    group.close()


def limits(group):
    return {n: dict(LIMIT) for n in group._motor_ids}


def kinds(bus):
    return [f.arbitration_id >> 24 for f in bus.sent]


def prepared(rig):
    group, bus, clock = rig
    assert group.prepare(limits(group)) == {"a": .25, "b": .25}
    bus.events.clear()
    bus.sent.clear()
    return group, bus, clock


def assert_batches(events, count):
    """Every consecutive send burst contains all IDs before a receive."""
    bursts = []
    current = []
    for event in events:
        if event[0] == "send":
            current.append(event)
        elif current:
            bursts.append(current)
            current = []
    if current:
        bursts.append(current)
    assert bursts
    assert all(len(burst) == count for burst in bursts)
    assert all({e[2] for e in burst} == set(range(1, count + 1)) for burst in bursts)
    return bursts


def assert_disable_order(events, count=2):
    first_send = next(i for i, event in enumerate(events) if event[0] == "send")
    assert all(event == ("recv", 0.) for event in events[:first_send])
    assert all(event[0] == "send" for event in events[first_send:first_send + count])
    assert_batches(events, count)


def test_connect_filter_close_no_commands(rig):
    group, bus, _ = rig
    assert not bus.sent
    assert bus.open_kwargs == dict(interface="socketcan", channel="fake", bitrate=1000000,
                                   ignore_config=True, can_filters=[
                                       dict(can_id=0x1FF, can_mask=0xFFFF, extended=True),
                                       dict(can_id=0x2FF, can_mask=0xFFFF, extended=True)])
    group.close()
    group.close()
    assert not bus.sent and bus.shutdown_calls == 1
    group.connect()
    assert not group._prepared


@pytest.mark.parametrize("ids", [{}, {"a": 0}, {"a": True}, {"a": 256},
                                  {"": 1}, {"a": 1, "b": 1}, None])
def test_invalid_ids(ids):
    with pytest.raises((ValueError, TypeError)):
        CspGroup("fake", ids)


@pytest.mark.parametrize("models", [{"a": MODEL}, {"a": MODEL, "b": "unknown"},
                                    {"a": MODEL, "b": None}, []])
def test_invalid_motor_models_rejected(models):
    with pytest.raises((ValueError, TypeError)):
        CspGroup("fake", {"a": 1, "b": 2}, motor_models=models)


def test_missing_models_allow_read_only_but_reject_prepare_before_sends(monkeypatch):
    clock = Clock()
    bus = GroupBus(clock)
    monkeypatch.setattr(csp_group.time, "monotonic", clock.monotonic)
    monkeypatch.setattr(can.interface, "Bus", lambda **kwargs: bus)
    group = CspGroup("fake", {"a": 1, "b": 2})
    group.connect()
    try:
        assert set(group.read_settings(timeout_s=.02)) == {"a", "b"}
        bus.sent.clear()
        with pytest.raises(ValueError, match="motor_models"):
            group.prepare(limits(group))
        assert not bus.sent
        group._prepared = True
        group._watchdogs = {"a": 20000, "b": 20000}
        with pytest.raises(ValueError, match="motor_models"):
            group.send_positions({"a": .5, "b": .5}, timeout_s=.02)
        assert not bus.sent
    finally:
        group.close()


def test_settings_register_batches_out_of_order(rig):
    group, bus, _ = rig
    bus.reverse = True
    bus.values[2][0x7019] = -.375
    result = group.read_settings(timeout_s=.02)
    assert result["b"]["raw_motor_position_rad"] == -.375
    assert set(result["a"]) == {key for key, _, _ in csp_group._SETTINGS}
    bursts = assert_batches(bus.events, 2)
    assert [b[0][3] for b in bursts] == REGISTERS


@pytest.mark.parametrize("count", [2, 12])
def test_prepare_phases_delayed_enable_and_control(monkeypatch, count):
    clock = Clock()
    bus = GroupBus(clock, count)
    bus.enable_delay = .006  # Deliberately >3 ms, <20 ms; no per-ID 3 ms wait.
    bus.reverse = True
    monkeypatch.setattr(csp_group.time, "monotonic", clock.monotonic)
    monkeypatch.setattr(can.interface, "Bus", lambda **kwargs: bus)
    motor_ids = {f"m{i}": i for i in range(1, count + 1)}
    group = CspGroup("fake", motor_ids,
                     motor_models={name: MODEL for name in motor_ids})
    group.connect()
    try:
        assert group.prepare(limits(group)) == {n: .25 for n in group._motor_ids}
        bursts = assert_batches(bus.events, count)
        assert [b[0][1] for b in bursts] == [4] + [17]*7 + [18]*5 + [17]*8 + [3, 17]
        writes = [b for b in bursts if b[0][1] == 18]
        assert [b[0][3] for b in writes] == [0x7005, 0x7017, 0x7018, 0x700B, 0x7016]
        # Every configuration/verification send precedes the first enable.
        first_enable = next(i for i, f in enumerate(bus.sent) if f.arbitration_id >> 24 == 3)
        assert all(f.arbitration_id >> 24 != 18 for f in bus.sent[first_enable:])
        bus.events.clear()
        bus.sent.clear()
        for motor in bus.values:
            bus.values[motor][0x7019] = -.375
        result = group.send_positions({n: .5 for n in group._motor_ids}, timeout_s=.02)
        quantization = MODEL_MIT_POSITION_TABLE[MODEL] / 0x7FFF
        assert all(value == pytest.approx(-.375, abs=quantization / 2)
                   for value in result.values())
        assert [b[0][1] for b in assert_batches(bus.events, count)] == [18]
        assert 17 not in kinds(bus)
        cycle = group.last_cycle
        assert set(cycle["target_send_monotonic_s"]) == set(group._motor_ids)
        stamps = list(cycle["target_send_monotonic_s"].values())
        assert stamps == sorted(stamps)
        assert all(later - earlier == pytest.approx(bus.send_cost)
                   for earlier, later in zip(stamps, stamps[1:]))
        # Timestamp is captured after each potentially blocking transport send.
        assert cycle["target_burst_s"] - (stamps[-1] - stamps[0]) == pytest.approx(bus.send_cost)
        assert cycle["target_burst_s"] == pytest.approx(count * bus.send_cost)
        assert cycle["total_s"] >= cycle["target_burst_s"]
        assert cycle["status_position_rad"] == result
        bus.events.clear()
        assert group.disable(timeout_s=.02) == {n: None for n in group._motor_ids}
        assert_disable_order(bus.events, count)
    finally:
        group.close()


@pytest.mark.parametrize("key", sorted(CspGroup._FIELDS))
@pytest.mark.parametrize("value", [None, True, float("nan"), float("inf")])
def test_prepare_invalid_args_before_any_commands(rig, key, value):
    group, bus, _ = rig
    config = limits(group)
    config["b"][key] = value
    with pytest.raises((ValueError, TypeError)):
        group.prepare(config)
    assert not bus.sent and not group._failed


@pytest.mark.parametrize("change", [{"position_min_rad": -4.}, {"position_max_rad": 4.},
                                    {"position_min_rad": .25}, {"expected_position_rad": .5},
                                    {"current_limit_a": 1e100}, {"current_limit_a": 1e-100},
                                    {"position_tolerance_rad": 0.}])
def test_prepare_invalid_guards_and_float32(rig, change):
    group, bus, _ = rig
    config = limits(group)
    config["b"].update(change)
    # Expected .5 is itself valid, but mismatches the observed .25.
    with pytest.raises(ValueError):
        group.prepare(config)
    assert 3 not in kinds(bus)
    if "expected_position_rad" not in change:
        assert not bus.sent


@pytest.mark.parametrize("operation", ["prepare", "send"])
@pytest.mark.parametrize("bad", [{"a": .2}, {"a": .2, "b": .2, "c": .2}, None])
def test_exact_keys_no_writes(rig, operation, bad):
    group, bus, _ = rig
    with pytest.raises((ValueError, TypeError)):
        if operation == "prepare":
            group.prepare(bad)
        else:
            group.send_positions(bad, timeout_s=.02)
    assert not bus.sent and not group._failed


@pytest.mark.parametrize("value", [None, True, float("nan"), float("inf"), -4., 4., math.pi])
def test_validate_all_targets_before_writes(rig, value):
    group, bus, _ = prepared(rig)
    # pi encodes slightly above pi in float32 and must also be rejected.
    with pytest.raises((ValueError, TypeError)):
        group.send_positions({"a": .5, "b": value}, timeout_s=.02)
    assert not bus.sent and not group._failed and group._prepared


@pytest.mark.parametrize("register,value", [(0x7005, 4), (0x7029, 0), (0x7029, 2),
    (0x7028, 0), (0x7028, 400), (0x7017, 0.), (0x7018, -1.),
    (0x700B, float("nan")), (0x7019, 4.), (0x7019, .255), (0x7019, float("inf"))])
def test_reported_settings_and_margin_reject_before_enable(rig, register, value):
    group, bus, _ = rig
    config = limits(group)
    if value == .255:
        config["b"]["position_max_rad"] = .26  # .255 lacks .01 clearance.
    bus.values[2][register] = value
    with pytest.raises(ValueError):
        group.prepare(config)
    assert 3 not in kinds(bus) and 18 not in kinds(bus)
    assert group._failed and not group._prepared


@pytest.mark.parametrize("key,register", [("velocity_limit_rad_s", 0x7017),
    ("current_limit_a", 0x7018), ("torque_limit_nm", 0x700B)])
def test_no_limit_increases_including_float32(rig, key, register):
    group, bus, _ = rig
    config = limits(group)
    config["b"][key] = bus.values[2][register] + 1e-10
    with pytest.raises(ValueError, match="increase"):
        group.prepare(config)
    assert 18 not in kinds(bus) and 3 not in kinds(bus)


@pytest.mark.parametrize("register,value", [(0x7005, 1), (0x7017, 1.1), (0x7018, 2.1),
    (0x700B, 3.1), (0x7028, 30000), (0x7029, 0), (0x7016, .5), (0x7019, .5)])
def test_all_readbacks_gate_all_enables(rig, register, value):
    group, bus, _ = rig
    def hook(request, frames):
        if request.arbitration_id >> 24 == 18 and request.arbitration_id & 255 == 2:
            if struct.unpack_from("<H", request.data)[0] == 0x7016:
                bus.values[2][register] = value
        return frames
    bus.hook = hook
    with pytest.raises(ValueError):
        group.prepare(limits(group))
    assert 3 not in kinds(bus) and group._failed


def test_duplicates_do_not_satisfy_other_motor(rig):
    group, bus, _ = prepared(rig)
    def hook(request, frames):
        kind = request.arbitration_id >> 24
        if kind == 18:
            if request.arbitration_id & 255 == 2:
                return []
            return frames * 4
        return frames
    bus.hook = hook
    with pytest.raises(TimeoutError):
        group.send_positions({"a": .5, "b": .5}, timeout_s=.02)
    assert group.last_error == dict(phase="control:target_status", pending_motors=["b"])
    assert group._failed and not group._prepared
    count = len(bus.sent)
    with pytest.raises(RuntimeError, match="Failed session"):
        group.send_positions({"a": .5, "b": .5}, timeout_s=.02)
    assert len(bus.sent) == count


@pytest.mark.parametrize("phase", ["drain", "status"])
@pytest.mark.parametrize("bit", range(6))
def test_fault_any_selected_motor_in_any_phase(rig, phase, bit):
    group, bus, clock = prepared(rig)
    fault = status(2, 2, 1 << bit)
    if phase == "drain":
        bus.queue.append((clock.now, fault))
    else:
        def hook(request, frames):
            kind = request.arbitration_id >> 24
            if request.arbitration_id & 255 == 1 and kind == 18:
                return [fault] + frames
            return frames
        bus.hook = hook
    with pytest.raises(RuntimeError, match="fault"):
        group.send_positions({"a": .5, "b": .5}, timeout_s=.02)
    assert group._failed and not group._prepared
    if phase == "drain":
        assert not bus.sent


@pytest.mark.parametrize("kind", [2, 17, 21])
@pytest.mark.parametrize("change", [{"is_extended_id": False}, {"is_fd": True},
    {"is_remote_frame": True}, {"is_error_frame": True}, {"bitrate_switch": True},
    {"error_state_indicator": True}, {"dlc": 7}, {"data": b""}])
def test_malformed_selected_frames_poison(rig, kind, change):
    group, bus, clock = prepared(rig)
    bad = reply(2, 0x7019, .25) if kind == 17 else status(2, kind=kind)
    for key, value in change.items():
        setattr(bad, key, value)
    bus.queue.append((clock.now, bad))
    with pytest.raises(ValueError, match="Malformed"):
        group.send_positions({"a": .5, "b": .5}, timeout_s=.02)
    assert not bus.sent and group._failed


@pytest.mark.parametrize("bad", [status(2, 2, faults=1), status(2, kind=21)])
def test_invalid_status_or_fault_report(rig, bad):
    group, bus, _ = prepared(rig)
    bus.hook = lambda request, frames: ([bad] if request.arbitration_id >> 24 == 18
                                        and request.arbitration_id & 255 == 2 else frames)
    with pytest.raises((ValueError, RuntimeError)):
        group.send_positions({"a": .5, "b": .5}, timeout_s=.02)
    assert group._failed


def test_unrelated_register_and_address_traffic_is_ignored(rig):
    group, bus, _ = prepared(rig)
    bus.hook = lambda request, frames: [reply(2, 0x7016, .5), reply(3, 0x7019, .5)] + frames
    result = group.send_positions({"a": .5, "b": .5}, timeout_s=.02)
    assert all(value == pytest.approx(.25, abs=MODEL_MIT_POSITION_TABLE[MODEL] / 0x7FFF / 2)
               for value in result.values())


@pytest.mark.parametrize("mode", [0, 1, 3])
def test_control_requires_mode2(rig, mode):
    group, bus, _ = prepared(rig)
    bus.hook = lambda request, frames: [status(request.arbitration_id & 255, mode)]
    with pytest.raises(RuntimeError, match="mode"):
        group.send_positions({"a": .5, "b": .5}, timeout_s=.02)


class Cancelled(Exception):
    pass


@pytest.mark.parametrize("when", ["drain", "before_enable", "partial_enable"])
def test_cancellation_and_explicit_cleanup(rig, when):
    group, bus, _ = rig
    calls = []
    def cancel():
        calls.append(len(bus.events))
        if when == "drain" and group._phase == "prepare:before:run_mode":
            raise Cancelled()
        if group._phase == "prepare:enable":
            if when == "before_enable" or (when == "partial_enable" and 3 in kinds(bus)):
                raise Cancelled()
    with pytest.raises(Cancelled):
        group.prepare(limits(group), check_cancel=cancel)
    assert group._failed and not group._prepared
    assert kinds(bus).count(3) == (1 if when == "partial_enable" else 0)
    assert group._cancel is None
    before = len(calls)
    bus.events.clear()
    # Cleanup ignores both poison and cancellation, and skips in-flight Motor.
    assert group.disable(timeout_s=.02) == {"a": None, "b": None}
    assert len(calls) == before
    assert_disable_order(bus.events)
    assert group._failed


def test_callback_before_every_io_including_drains(rig):
    group, bus, _ = rig
    calls = []
    def cancel():
        calls.append(len(bus.events))
    group.prepare(limits(group), check_cancel=cancel)
    assert calls == list(range(len(bus.events)))


def test_partial_enable_send_failure_cleanup(rig):
    group, bus, _ = rig
    def hook(request, frames):
        if request.arbitration_id >> 24 == 3 and request.arbitration_id & 255 == 2:
            raise OSError("enable send failed")
        return frames
    bus.hook = hook
    with pytest.raises(OSError):
        group.prepare(limits(group))
    assert group.last_error["phase"] == "prepare:enable"
    assert group._failed and not group._prepared
    bus.hook = None
    assert group.disable(timeout_s=.02) == {"a": None, "b": None}


@pytest.mark.parametrize("outcome", ["missing", "fault", "malformed", "mode", "send"])
def test_disable_per_motor_outcomes_and_all_sends_first(rig, outcome):
    group, bus, clock = prepared(rig)
    group._failed = True
    group._cancel = lambda: pytest.fail("cleanup invoked cancellation")
    bus.queue.append((clock.now, status(1, 2)))
    def hook(request, frames):
        if request.arbitration_id & 255 == 2:
            if outcome == "missing":
                return []
            if outcome == "send":
                raise OSError("send failed")
            if outcome == "fault":
                return [status(2, faults=1)] + frames
            if outcome == "mode":
                return [status(2, 1)] + frames
            bad = status(2)
            bad.dlc = 7
            return [bad] + frames
        return frames
    bus.hook = hook
    result = group.disable(timeout_s=.02)
    assert result["a"] is None and isinstance(result["b"], str)
    assert_disable_order(bus.events)
    assert kinds(bus) == [4, 4] and group._failed and not group._prepared
    assert group.last_error == {"phase": "disable", "pending_motors": ["b"]}
    group._cancel = None


def test_disable_attempts_every_send_after_failure_and_deadline(rig):
    group, bus, clock = rig
    bus.send_cost = .03
    start = clock.now
    result = group.disable(timeout_s=.02)
    assert kinds(bus) == [4, 4]
    assert all(isinstance(v, str) for v in result.values())
    assert clock.now - start <= .020001
    assert [e for e in bus.events if e[0] == "send"][1][-1] == 0.


def test_disable_without_connection_returns_all_outcomes(rig):
    group, bus, _ = rig
    group.close()
    assert all(isinstance(v, str) for v in group.disable(timeout_s=.02).values())
    assert not bus.sent


@pytest.mark.parametrize("operation", ["control", "disable", "settings", "prepare"])
def test_shared_deadlines_not_per_motor(rig, operation):
    group, bus, clock = prepared(rig) if operation == "control" else rig
    bus.send_cost = .006
    start = clock.now
    if operation == "disable":
        bus.hook = lambda request, frames: []
        assert all(v is not None for v in group.disable(timeout_s=.01).values())
    else:
        with pytest.raises(TimeoutError):
            if operation == "control":
                group.send_positions({"a": .5, "b": .5}, timeout_s=.01)
            elif operation == "settings":
                group.read_settings(timeout_s=.01)
            else:
                group.prepare(limits(group), timeout_s=.01)
    assert clock.now - start <= .010001


def test_status_temperature_exact_source_out_of_order_and_cycle_snapshot(rig):
    group, bus, clock = prepared(rig)
    # This stale status is inspected by the control pre-drain, never accepted.
    bus.queue.append((clock.now, status(1, 2, temperature_u16=999)))
    bus.reverse = True

    def temperatures(request, frames):
        if request.arbitration_id >> 24 == 18:
            motor = request.arbitration_id & 255
            return [status(motor, 2, temperature_u16={1: 321, 2: 654}[motor],
                           position_u16=0x1234, velocity_u16=0x5678,
                           torque_u16=0x9ABC)]
        return frames

    bus.hook = temperatures
    group.send_positions({"a": .5, "b": .5}, timeout_s=.02)
    assert group.last_status_temperature_c == {"a": 32.1, "b": 65.4}
    assert group.last_cycle["status_temperature_c"] == {"a": 32.1, "b": 65.4}
    assert set(group.last_cycle["status_receive_monotonic_s"]) == {"a", "b"}
    assert group.last_cycle["status_receive_monotonic_s"] == group.last_status_monotonic_s

    # Cleanup statuses have a different payload but must not replace enabled telemetry.
    bus.hook = lambda request, frames: [status(request.arbitration_id & 255,
                                               temperature_u16=777)]
    assert group.disable(timeout_s=.02) == {"a": None, "b": None}
    assert group.last_status_temperature_c == {"a": 32.1, "b": 65.4}


def test_bad_status_does_not_update_temperature_and_reconnect_clears(rig):
    group, bus, _ = prepared(rig)
    before_temperature = dict(group.last_status_temperature_c)
    before_time = dict(group.last_status_monotonic_s)
    bus.hook = lambda request, frames: ([status(2, 2, faults=1, temperature_u16=900)]
        if request.arbitration_id >> 24 == 18 and request.arbitration_id & 255 == 1
        else frames)
    with pytest.raises(RuntimeError, match="fault"):
        group.send_positions({"a": .5, "b": .5}, timeout_s=.02)
    assert group.last_status_temperature_c == before_temperature
    assert group.last_status_monotonic_s == before_time

    group.close()
    group.connect()
    assert group.last_status_temperature_c == {}
    assert group.last_status_monotonic_s == {}


def test_control_one_deadline_for_target_status_batch(rig):
    group, bus, clock = prepared(rig)
    bus.send_cost = .002
    bus.recv_cost = .001
    start = clock.now
    with pytest.raises(TimeoutError):
        group.send_positions({"a": .5, "b": .5}, timeout_s=.006)
    assert clock.now - start <= .008001
    assert group.last_cycle["total_s"] <= .008001
    assert group.last_error["phase"] == "control:target_status"


@pytest.mark.parametrize("operation", ["control", "disable"])
def test_frame_budget_bounded_even_clock_stalled(rig, operation):
    group, bus, _ = prepared(rig)
    bus.recv_cost = 0.
    bus.flood = status(3)
    group._FRAME_BUDGET = 100
    if operation == "control":
        with pytest.raises(RuntimeError, match="budget"):
            group.send_positions({"a": .5, "b": .5}, timeout_s=.02)
        assert not bus.sent
    else:
        bus.hook = lambda request, frames: []
        assert all(v is not None for v in group.disable(timeout_s=.02).values())
    assert len([e for e in bus.events if e[0] == "recv"]) == 100


def test_nonblocking_concurrent_callers_no_interleaving(rig):
    group, bus, _ = rig
    def hook(request, frames):
        count = len(bus.sent)
        for op in (group.connect, group.close, lambda: group.read_settings(timeout_s=.02),
                   lambda: group.prepare(limits(group)),
                   lambda: group.send_positions({"a": .5, "b": .5}, timeout_s=.02),
                   lambda: group.disable(timeout_s=.02)):
            with pytest.raises(RuntimeError, match="in progress"):
                op()
        assert len(bus.sent) == count
        return frames
    bus.hook = hook
    group.prepare(limits(group))
    group.send_positions({"a": .5, "b": .5}, timeout_s=.02)
    assert not group._failed


@pytest.mark.parametrize("timeout", [0, -.1, True, None, float("nan"), float("inf")])
def test_invalid_timeouts_no_commands(rig, timeout):
    group, bus, _ = rig
    for operation in (lambda: group.prepare(limits(group), timeout_s=timeout),
                      lambda: group.read_settings(timeout_s=timeout),
                      lambda: group.disable(timeout_s=timeout),
                      lambda: group.send_positions({"a": .5, "b": .5}, timeout_s=timeout)):
        with pytest.raises((ValueError, TypeError)):
            operation()
    assert not bus.sent and not group._failed


@pytest.mark.parametrize("change", ["mode", "position", "interrupt", "missing"])
def test_enable_and_post_enable_errors_require_explicit_cleanup(rig, change):
    group, bus, _ = rig
    def hook(request, frames):
        if request.arbitration_id >> 24 == 3 and request.arbitration_id & 255 == 2:
            if change == "interrupt":
                raise KeyboardInterrupt()
            if change == "mode":
                return [status(2, 0)]
            if change == "missing":
                return []
            bus.values[2][0x7019] = .5
        return frames
    bus.hook = hook
    with pytest.raises((KeyboardInterrupt, RuntimeError, ValueError, TimeoutError),
                       match="status/register" if change == "position" else None):
        group.prepare(limits(group))
    assert kinds(bus).count(3) == 2
    assert kinds(bus).count(4) == 2  # No implicit cleanup commands.
    assert not group._prepared and group._failed
    bus.hook = None
    assert group.disable(timeout_s=.02) == {"a": None, "b": None}


def test_malformed_parameter_tail_during_control_fails_closed(rig):
    change = "reserved_id"
    group, bus, _ = prepared(rig)
    bad = reply(2, 0x7019, .25)
    if change == "reserved_id":
        bad.arbitration_id |= 1 << 16
    elif change == "wrong_register":
        bad = reply(2, 0x7016, .25)
    elif change == "wrong_host":
        bad.arbitration_id ^= 1
    else:
        bad = reply(3, 0x7019, .25)
    bus.hook = lambda request, frames: (frames + [bad] if request.arbitration_id >> 24 == 18
                                        and request.arbitration_id & 255 == 2 else frames)
    with pytest.raises(ValueError):
        group.send_positions({"a": .5, "b": .5}, timeout_s=.02)
    assert group._failed


def test_control_drain_consumes_same_total_budget(rig):
    group, bus, clock = prepared(rig)
    bus.flood = status(3)
    def drain_overhead(timeout):
        assert timeout == 0.
        clock.now += .001
    bus.recv_hook = drain_overhead
    start = clock.now
    with pytest.raises(TimeoutError):
        group.send_positions({"a": .5, "b": .5}, timeout_s=.005)
    assert not bus.sent
    assert clock.now - start <= .006001


def test_preparation_total_bounded_by_phase_count(rig):
    group, bus, clock = rig
    bus.send_cost = .004
    bus.recv_cost = .001
    start = clock.now
    assert group.prepare(limits(group), timeout_s=.02) == {"a": .25, "b": .25}
    assert clock.now - start < 23 * .02
    assert clock.now - start > .02


def test_successful_disable_revokes_prepare_and_reconnect_requires_prepare(rig):
    group, bus, _ = prepared(rig)
    assert group.disable(timeout_s=.02) == {"a": None, "b": None}
    count = len(bus.sent)
    with pytest.raises(RuntimeError, match="prepare"):
        group.send_positions({"a": .5, "b": .5}, timeout_s=.02)
    assert len(bus.sent) == count
    group.close()
    group.connect()
    assert not group._failed and not group._prepared
    assert group.last_error is None and group.last_cycle is None
    assert len(bus.sent) == count


@pytest.mark.parametrize("operation", ["disable", "prepare"])
def test_queued_reset_cannot_confirm_without_new_reply(rig, operation):
    group, bus, clock = rig
    for motor in (1, 2):
        stale = status(motor)
        stale.timestamp = 1800000000.  # Wall-clock epoch, not SDK monotonic time.
        bus.queue.append((clock.now, stale))
    bus.hook = lambda request, frames: []
    if operation == "disable":
        outcomes = group.disable(timeout_s=.02)
        assert all(isinstance(value, str) for value in outcomes.values())
        assert group.last_disable_outcomes == outcomes
    else:
        with pytest.raises(TimeoutError):
            group.prepare(limits(group))
    assert kinds(bus) == [4, 4]
    assert_disable_order(bus.events)
    assert group._failed and not group._prepared


@pytest.mark.parametrize("operation", ["disable", "prepare"])
@pytest.mark.parametrize("bad", ["fault", "malformed", "budget", "interrupt", "deadline"])
def test_failed_disable_boundary_never_suppresses_sends(rig, operation, bad):
    group, bus, clock = rig
    group._FRAME_BUDGET = 8
    if bad == "budget":
        bus.flood = status(1)
    elif bad == "interrupt":
        def interrupt_once(timeout):
            bus.recv_hook = None
            raise KeyboardInterrupt("drain interrupted")
        bus.recv_hook = interrupt_once
    elif bad == "deadline":
        def drain_cost(timeout):
            bus.recv_hook = None
            clock.now += .02
        bus.recv_hook = drain_cost
    else:
        frame = status(1, faults=1 if bad == "fault" else 0)
        if bad == "malformed":
            frame.dlc = 7
        bus.queue.append((clock.now, frame))
    if operation == "disable":
        outcomes = group.disable(timeout_s=.02)
        assert outcomes["a"] is not None
        if bad not in ("fault", "malformed"):
            assert all(v is not None for v in outcomes.values())
        assert group.last_disable_outcomes == outcomes
    else:
        with pytest.raises((RuntimeError, ValueError, KeyboardInterrupt, TimeoutError)):
            group.prepare(limits(group))
    assert kinds(bus) == [4, 4]
    assert_disable_order(bus.events)
    if bad == "deadline":
        assert all(e[-1] == 0. for e in bus.events if e[0] == "send")


def test_pre_enable_hold_matches_latest_disabled_position(rig):
    group, bus, _ = rig
    def hook(request, frames):
        if request.arbitration_id >> 24 == 18 and request.arbitration_id & 255 == 2:
            if struct.unpack_from("<H", request.data)[0] == 0x7016:
                # Both are <.01 from .25, but .016 apart from each other.
                bus.values[2][0x7016] = .242
                bus.values[2][0x7019] = .258
        return frames
    bus.hook = hook
    with pytest.raises(ValueError, match="Disabled hold"):
        group.prepare(limits(group))
    assert 3 not in kinds(bus) and group._failed


def test_queued_type17_positions_do_not_replace_status_positions(rig):
    group, bus, _ = prepared(rig)
    def hook(request, frames):
        motor = request.arbitration_id & 255
        if request.arbitration_id >> 24 == 18 and motor == 2:
            return frames + [reply(1, 0x7019, -.75), reply(2, 0x7019, -.75)]
        return frames
    bus.hook = hook
    result = group.send_positions({"a": .5, "b": .5}, timeout_s=.02)
    assert all(value == pytest.approx(.25, abs=MODEL_MIT_POSITION_TABLE[MODEL] / 0x7FFF / 2)
               for value in result.values())
    assert [b[0][1] for b in assert_batches(bus.events, 2)] == [18]
    assert 17 not in kinds(bus)


def test_control_read_batch_with_existing_deadline_always_drains(rig):
    group, bus, clock = rig
    for motor in (1, 2):
        bus.queue.append((clock.now, reply(motor, 0x7019, -.75)))
    with group._operation("test"):
        values = group._read_batch(0x7019, "<f", .02, "test:positions",
                                   deadline=clock.now + .02)
    assert values == {"a": .25, "b": .25}
    assert all(event == ("recv", 0.) for event in bus.events[:3])


@pytest.mark.parametrize("operation", ["prepare", "control", "disable"])
@pytest.mark.parametrize("tail", ["fault", "report", "malformed_status", "malformed_parameter", "header"])
def test_queued_tail_after_final_reply_cannot_succeed(rig, operation, tail):
    group, bus, _ = prepared(rig) if operation == "control" else rig
    bad = status(1, faults=1) if tail == "fault" else status(1, kind=21) if tail == "report" else status(1)
    if tail == "malformed_status":
        bad.is_fd = True
    elif tail in ("malformed_parameter", "header"):
        bad = reply(1, 0x7019, .25, reserved=1 if tail == "header" else 0)
        if tail == "malformed_parameter":
            bad.data = bytearray(b"\x19\x70")
    def hook(request, frames):
        motor = request.arbitration_id & 255
        kind = request.arbitration_id >> 24
        if motor == 2:
            if operation == "disable" and kind == 4:
                return frames + [bad]
            if ((operation == "control" and kind == 18) or
                    (operation == "prepare" and kind == 17 and
                     struct.unpack_from("<H", request.data)[0] == 0x7019 and
                     3 in kinds(bus))):
                return frames + [bad]
        return frames
    bus.hook = hook
    if operation == "disable":
        outcomes = group.disable(timeout_s=.02)
        assert outcomes["a"] is not None and outcomes["b"] is None
        assert group.last_disable_outcomes == outcomes
    else:
        with pytest.raises((RuntimeError, ValueError)):
            if operation == "prepare":
                group.prepare(limits(group))
            else:
                group.send_positions({"a": .5, "b": .5}, timeout_s=.02)
    assert group._failed and not group._prepared


@pytest.mark.parametrize("operation", ["prepare", "control", "disable"])
def test_final_inspection_bounded_and_no_fresh_deadline(rig, operation):
    group, bus, clock = prepared(rig) if operation == "control" else rig
    group._FRAME_BUDGET = 8
    def hook(request, frames):
        kind = request.arbitration_id >> 24
        if request.arbitration_id & 255 == 2:
            final = (kind == 4 if operation == "disable" else
                     kind == 18 if operation == "control" else
                     kind == 17 and struct.unpack_from("<H", request.data)[0] == 0x7019)
            if final and (operation != "prepare" or 3 in kinds(bus)):
                bus.flood = status(3)  # Unrelated, but an undrainable tail.
        return frames
    bus.hook = hook
    start = clock.now
    if operation == "disable":
        assert all(v is not None for v in group.disable(timeout_s=.02).values())
    else:
        with pytest.raises(RuntimeError, match="budget"):
            if operation == "prepare":
                group.prepare(limits(group))
            else:
                group.send_positions({"a": .5, "b": .5}, timeout_s=.02)
    assert clock.now - start < .02
    assert len(bus.events) < 400


@pytest.mark.parametrize("failure", [KeyboardInterrupt, SystemExit])
@pytest.mark.parametrize("where", ["send", "receive", "tail"])
def test_cleanup_interrupts_return_outcomes_without_aborting_sendall(rig, failure, where):
    group, bus, clock = rig
    def hook(request, frames):
        if where == "send" and request.arbitration_id & 255 == 1:
            raise failure("send interruption")
        return frames
    bus.hook = hook
    if where != "send":
        count = 0
        def interrupt_once(timeout):
            nonlocal count
            if bus.sent:
                count += 1
                if (where == "receive" and count == 1) or (where == "tail" and count == 3):
                    bus.recv_hook = None
                    raise failure("receive interruption")
        bus.recv_hook = interrupt_once
    start = clock.now
    outcomes = group.disable(timeout_s=.02)
    assert kinds(bus) == [4, 4]
    assert_disable_order(bus.events)
    assert group.last_disable_outcomes == outcomes
    assert any(failure.__name__ in (v or "") for v in outcomes.values())
    assert group._failed and not group._prepared
    assert clock.now - start < .02
    if where == "send":
        assert outcomes["b"] is None


def test_disable_interrupt_and_expiry_still_sends_remaining_nonblocking(rig):
    group, bus, clock = rig
    def hook(request, frames):
        if request.arbitration_id & 255 == 1:
            clock.now += .02 - bus.send_cost
            raise KeyboardInterrupt()
        return frames
    bus.hook = hook
    outcomes = group.disable(timeout_s=.02)
    assert kinds(bus) == [4, 4]
    assert [e for e in bus.events if e[0] == "send"][1][-1] == 0.
    assert all(isinstance(value, str) for value in outcomes.values())
    assert "KeyboardInterrupt" in outcomes["a"]


def test_preparation_initial_disable_cancellation_still_sends_all(rig):
    group, bus, _ = rig
    def cancel():
        raise Cancelled("cancel at boundary")
    with pytest.raises(Cancelled):
        group.prepare(limits(group), check_cancel=cancel)
    assert kinds(bus) == [4, 4]
    assert group._failed and not group._prepared
    assert group._cancel is None
    # Prior Reset replies are now queued, but cleanup must discard them and
    # require replies to its own fresh send burst.
    assert group.disable(timeout_s=.02) == {"a": None, "b": None}


def test_fault_behind_last_hold_readback_blocks_every_enable(rig):
    group, bus, _ = rig
    def hook(request, frames):
        if request.arbitration_id >> 24 == 17 and request.arbitration_id & 255 == 2:
            if struct.unpack_from("<H", request.data)[0] == 0x7016:
                return frames + [status(1, faults=1)]
        return frames
    bus.hook = hook
    with pytest.raises(RuntimeError, match="fault"):
        group.prepare(limits(group))
    assert 3 not in kinds(bus)
    assert group.last_error == {"phase": "prepare:target_readback", "pending_motors": ["a"]}


def test_fault_behind_last_control_status_blocks_position_requests(rig):
    group, bus, _ = prepared(rig)
    def hook(request, frames):
        if request.arbitration_id >> 24 == 18 and request.arbitration_id & 255 == 2:
            return frames + [status(1, 2, faults=1)]
        return frames
    bus.hook = hook
    with pytest.raises(RuntimeError, match="fault"):
        group.send_positions({"a": .5, "b": .5}, timeout_s=.02)
    assert kinds(bus) == [18, 18]
    assert group.last_error == {"phase": "control:target_status", "pending_motors": ["a"]}


def test_disable_final_tail_faults_override_every_confirmed_motor(rig):
    group, bus, _ = rig
    def hook(request, frames):
        if request.arbitration_id & 255 == 2:
            return frames + [status(1, faults=1), status(2, kind=21), status(1), status(2)]
        return frames
    bus.hook = hook
    outcomes = group.disable(timeout_s=.02)
    assert all(v is not None for v in outcomes.values())
    assert group.last_disable_outcomes == outcomes
    assert group.last_error == {"phase": "disable", "pending_motors": ["a", "b"]}


@pytest.mark.parametrize("operation", ["control", "disable"])
def test_final_inspection_cannot_get_another_deadline(rig, operation):
    group, bus, clock = prepared(rig) if operation == "control" else rig
    start = clock.now
    def tail_cost(timeout):
        if timeout == 0. and bus.sent and not bus.queue:
            if operation == "disable" or kinds(bus)[-1] == 18:
                # Consume exactly the remaining group budget in the final
                # nonblocking inspection, not a new per-call timeout.
                clock.now = start + .02
    bus.recv_hook = tail_cost
    if operation == "disable":
        outcomes = group.disable(timeout_s=.02)
        assert all(v is not None for v in outcomes.values())
    else:
        with pytest.raises(TimeoutError):
            group.send_positions({"a": .5, "b": .5}, timeout_s=.02)
        assert group.last_cycle["total_s"] == pytest.approx(.02)
    assert clock.now - start == pytest.approx(.02)
    assert group._failed and not group._prepared


@pytest.mark.parametrize("motor", [1, 2])
@pytest.mark.parametrize("direction", [-1, 1])
def test_post_enable_tracks_verified_hold_target_not_only_initial(rig, motor, direction):
    group, bus, _ = rig
    target = .25 + direction * .009
    position = .25 - direction * .009
    def hook(request, frames):
        if request.arbitration_id & 255 == motor:
            kind = request.arbitration_id >> 24
            if kind == 18 and struct.unpack_from("<H", request.data)[0] == 0x7016:
                bus.values[motor][0x7016] = target
            elif kind == 3:
                bus.values[motor][0x7019] = position
        return frames
    bus.hook = hook
    # Target and post-enable position each pass the original .25 +/- .01
    # guards, but are .018 apart. Disabled position remains .25.
    with pytest.raises(ValueError, match="verified hold target"):
        group.prepare(limits(group))
    assert kinds(bus).count(3) == 2
    assert kinds(bus).count(4) == 2  # Failure does not implicitly clean up.
    assert group._failed and not group._prepared
    assert group.last_error["phase"] == "prepare:post_enable_guards"
    count = len(bus.sent)
    with pytest.raises(RuntimeError, match="Failed session"):
        group.send_positions({"a": .25, "b": .25}, timeout_s=.02)
    assert len(bus.sent) == count
    bus.hook = None
    assert group.disable(timeout_s=.02) == {"a": None, "b": None}


@pytest.mark.parametrize("direction", [-1, 1])
@pytest.mark.parametrize("at_tolerance", [False, True])
def test_post_enable_hold_target_within_or_at_tolerance_succeeds(rig, direction, at_tolerance):
    group, bus, _ = rig
    config = limits(group)
    if at_tolerance:
        # Exact binary fractions avoid ambiguity from float32 rounding at
        # the inclusive tolerance boundary.
        config["b"]["position_tolerance_rad"] = .015625
        target = .25 + direction * .0078125
        position = .25 - direction * .0078125
    else:
        target = .25 + direction * .009
        position = .25 + direction * .003
    def hook(request, frames):
        if request.arbitration_id & 255 == 2:
            kind = request.arbitration_id >> 24
            if kind == 18 and struct.unpack_from("<H", request.data)[0] == 0x7016:
                bus.values[2][0x7016] = target
            elif kind == 3:
                bus.values[2][0x7019] = position
        return frames
    bus.hook = hook
    assert group.prepare(config) == pytest.approx({"a": .25, "b": position})
    assert group._prepared and not group._failed


def test_runtime_tolerance_field_requires_explicit_sdk_translation(rig):
    group, bus, _ = rig
    config = limits(group)
    config["b"]["max_tracking_error_rad"] = config["b"].pop("position_tolerance_rad")
    with pytest.raises(ValueError, match="position_tolerance_rad"):
        group.prepare(config)
    assert not bus.sent and not group._failed
