"""CSP protocol tests only: no real sockets, CAN, or physical safety claims."""
from collections import deque
import gc
import math
import struct

import can
import pytest

from robstride_dynamics.csp import CspMotor
from robstride_dynamics import csp
from test_position import Clock, FakeTransport

LIMITS = dict(velocity_limit_rad_s=1.0, current_limit_a=2.0,
              torque_limit_nm=3.0, position_tolerance_rad=0.01)
REGISTERS = [0x7005, 0x7017, 0x7018, 0x700B, 0x7028, 0x7029, 0x7019]


def status(mode=0, motor=1, host=255, faults=0, kind=2, **kwargs):
    return can.Message(arbitration_id=(kind << 24) | (mode << 22) |
                       (faults << 16) | (motor << 8) | host,
                       is_extended_id=True, data=bytes(8), **kwargs)


class MotorFake(FakeTransport):
    def __init__(self, clock):
        super().__init__(clock)
        self.values = {0x7005: 0, 0x7017: 4., 0x7018: 5., 0x700B: 6.,
                       0x7028: 20000, 0x7029: 1, 0x7019: 0.25, 0x7016: 0.}
        self.mode = 0
        self.hook = None
        self.replies = self.respond

    def respond(self, request):
        kind = request.arbitration_id >> 24
        if kind == 17:
            register, = struct.unpack_from('<H', request.data)
            fmt = '<B' if register in (0x7005, 0x7029) else '<I' if register == 0x7028 else '<f'
            data = struct.pack('<HH', register, 0) + struct.pack(fmt, self.values[register]).ljust(4, b'\0')
            frames = [can.Message(arbitration_id=0x110001FF,
                                  is_extended_id=True, data=data)]
        else:
            if kind == 18:
                register, = struct.unpack_from('<H', request.data)
                fmt = '<B' if register == 0x7005 else '<f'
                self.values[register], = struct.unpack_from(fmt, request.data, 4)
            elif kind == 3:
                self.mode = 2
            elif kind == 4:
                self.mode = 0
            frames = [status(self.mode)]
        return self.hook(request, frames) if self.hook else frames


@pytest.fixture
def rig(monkeypatch):
    clock = Clock()
    bus = MotorFake(clock)
    monkeypatch.setattr(csp.time, 'monotonic', clock.monotonic)
    monkeypatch.setattr(can.interface, 'Bus', lambda **kwargs: bus)
    motor = CspMotor('fake', 1)
    assert not bus.sent
    motor.connect()
    assert not bus.sent
    yield motor, bus, clock
    motor.close()


def kinds(bus):
    return [frame.arbitration_id >> 24 for frame, _ in bus.sent]


def test_success_exact_order_and_raw_feedback(rig):
    motor, bus, _ = rig
    assert motor.prepare(**LIMITS) == 0.25
    assert kinds(bus) == [4] + [17]*7 + [18]*5 + [17]*8 + [3, 17]
    reads = [struct.unpack_from('<H', f.data)[0] for f, _ in bus.sent if f.arbitration_id >> 24 == 17]
    assert reads == REGISTERS + REGISTERS + [0x7016, 0x7019]
    writes = [f for f, _ in bus.sent if f.arbitration_id >> 24 == 18]
    assert [struct.unpack_from('<H', f.data)[0] for f in writes] == [0x7005, 0x7017, 0x7018, 0x700B, 0x7016]
    assert writes[0].data == bytes.fromhex('0570000005000000')
    assert motor.can_timeout_raw == 20000
    bus.values[0x7019] = -0.375
    assert motor.send_position(0.5) == -0.375  # Not the quantized Type-2 data.
    assert kinds(bus)[-2:] == [18, 17]
    assert all((f.arbitration_id & 0xFFFF) == 0xFF01 for f, _ in bus.sent)
    motor.disable()
    assert bus.sent[-1][0].data == bytes(8)
    with pytest.raises(RuntimeError, match='prepare'):
        motor.send_position(0.)
    count = len(bus.sent)
    motor.close()
    motor.close()
    assert len(bus.sent) == count and bus.shutdown_calls == 1


@pytest.mark.parametrize('key', list(LIMITS))
@pytest.mark.parametrize('value', [None, True, 0, -1, float('nan'), float('inf')])
def test_invalid_arguments_no_commands(rig, key, value):
    motor, bus, _ = rig
    with pytest.raises((ValueError, TypeError)):
        motor.prepare(**(LIMITS | {key: value}))
    assert not bus.sent and not motor._failed


def test_missing_limit_no_commands(rig):
    motor, bus, _ = rig
    with pytest.raises(TypeError):
        motor.prepare(current_limit_a=2, torque_limit_nm=3, position_tolerance_rad=.01)
    assert not bus.sent


@pytest.mark.parametrize('key', list(LIMITS)[:3])
def test_increased_limit_refused(rig, key):
    motor, bus, _ = rig
    with pytest.raises(ValueError, match='increase'):
        motor.prepare(**(LIMITS | {key: 10.}))
    assert 3 not in kinds(bus) and 18 not in kinds(bus)


@pytest.mark.parametrize('register,value', [(0x7028, 0), (0x7028, 2000),
    (0x7029, 0), (0x7029, 2), (0x7019, 4.), (0x7017, 0.),
    (0x7018, -1.), (0x700B, float('nan')), (0x7005, 4)])
def test_unsupported_settings_never_enable(rig, register, value):
    motor, bus, _ = rig
    bus.values[register] = value
    with pytest.raises(ValueError):
        motor.prepare(**LIMITS)
    assert 3 not in kinds(bus)


@pytest.mark.parametrize('register,value', [(0x7005, 1), (0x7017, 1.1),
    (0x7018, 2.1), (0x700B, 3.1), (0x7016, 0.5), (0x7019, 0.5),
    (0x7028, 30000), (0x7029, 0)])
def test_readbacks_gate_enable(rig, register, value):
    motor, bus, _ = rig
    def hook(request, frames):
        if request.arbitration_id >> 24 == 18 and struct.unpack_from('<H', request.data)[0] == 0x7016:
            bus.values[register] = value
        return frames
    bus.hook = hook
    with pytest.raises(ValueError):
        motor.prepare(**LIMITS)
    assert 3 not in kinds(bus) and motor._failed


@pytest.mark.parametrize('bad', [status(motor=2), status(host=3), status(kind=18)])
def test_uncorrelated_status_times_out(rig, bad):
    motor, bus, _ = rig
    bus.hook = lambda request, frames: [bad]
    with pytest.raises(TimeoutError):
        motor.prepare(**LIMITS)
    assert kinds(bus) == [4] and motor._failed


@pytest.mark.parametrize('bit', range(6))
def test_all_type2_fault_bits(rig, bit):
    motor, bus, _ = rig
    motor.prepare(**LIMITS)
    bus.hook = lambda request, frames: [status(2, faults=1 << bit)]
    with pytest.raises(RuntimeError, match='fault'):
        motor.send_position(.5)
    count = len(bus.sent)
    with pytest.raises(RuntimeError, match='Failed session'):
        motor.send_position(.5)
    assert len(bus.sent) == count


@pytest.mark.parametrize('change', [{'is_extended_id': False}, {'is_fd': True},
    {'is_remote_frame': True}, {'is_error_frame': True}, {'bitrate_switch': True},
    {'error_state_indicator': True}, {'dlc': 7}, {'data': b''}])
def test_malformed_status_poison(rig, change):
    motor, bus, _ = rig
    bad = status()
    for key, value in change.items():
        setattr(bad, key, value)
    bus.hook = lambda request, frames: [bad]
    with pytest.raises(ValueError, match='Malformed'):
        motor.prepare(**LIMITS)
    assert motor._failed and 3 not in kinds(bus)


@pytest.mark.parametrize('mode', [0, 1, 3])
def test_send_requires_motor_status(rig, mode):
    motor, bus, _ = rig
    motor.prepare(**LIMITS)
    bus.hook = lambda request, frames: [status(mode)]
    with pytest.raises(RuntimeError, match='mode'):
        motor.send_position(.5)
    assert motor._failed


@pytest.mark.parametrize('failure', ['interrupt', 'timeout', 'movement'])
def test_partial_enable_disable_exempt_poison(rig, failure):
    motor, bus, _ = rig
    def hook(request, frames):
        if request.arbitration_id >> 24 == 3:
            if failure == 'interrupt':
                raise KeyboardInterrupt()
            if failure == 'timeout':
                return []
            bus.values[0x7019] = 0.5
        return frames
    bus.hook = hook
    with pytest.raises((KeyboardInterrupt, TimeoutError, ValueError)):
        motor.prepare(**LIMITS)
    assert 3 in kinds(bus) and motor._failed and not motor._prepared
    bus.hook = None
    bus.queued.extend([status(motor=2)]*3)
    receives = len(bus.received)
    def before_receive(timeout):
        assert kinds(bus)[-1] == 4  # No pre-drain before disable send.
    bus.recv_hook = before_receive
    motor.disable()
    assert len(bus.received) > receives and motor._failed
    with pytest.raises(RuntimeError, match='Failed session'):
        motor.send_position(.5)


def test_disable_flood_is_bounded_and_close_independent(rig):
    motor, bus, clock = rig
    motor._failed = True
    bus.flood = status(motor=2)
    bus.hook = lambda request, frames: []
    start = clock.now
    try:
        with pytest.raises(TimeoutError):
            motor.disable(timeout_s=.01)
    finally:
        motor.close()
    assert kinds(bus) == [4] and clock.now - start <= .012
    assert bus.shutdown_calls == 1


def test_disable_send_error_and_cleanup(rig):
    motor, bus, _ = rig
    bus.send_error = OSError('send failure')
    try:
        with pytest.raises(OSError, match='send failure'):
            motor.disable()
    finally:
        motor.close()
    assert kinds(bus) == [4] and not bus.received and bus.shutdown_calls == 1


def test_stale_queue_drained_and_flood_before_position_refused(rig):
    motor, bus, _ = rig
    motor.prepare(**LIMITS)
    bus.queued.append(status(0))
    motor.send_position(.5)  # Discard stale Reset before write.
    count = len(bus.sent)
    bus.flood = status(motor=2)
    with pytest.raises((RuntimeError, TimeoutError)):
        motor.send_position(.5, timeout_s=.5)
    assert len(bus.sent) == count and motor._failed


def test_whole_operation_nonblocking_lock(rig):
    motor, bus, _ = rig
    def hook(request, frames):
        count = len(bus.sent)
        for op in (lambda: motor.prepare(**LIMITS), lambda: motor.send_position(.5),
                   motor.disable, motor.read_settings, motor.read_position, motor.close):
            with pytest.raises(RuntimeError, match='in progress'):
                op()
        assert len(bus.sent) == count
        return frames
    bus.hook = hook
    motor.prepare(**LIMITS)
    motor.send_position(.5)


def test_malformed_raw_register_and_fault_report(rig):
    motor, bus, _ = rig
    motor.prepare(**LIMITS)
    def hook(request, frames):
        if request.arbitration_id >> 24 == 17:
            frames[0].data = bytearray(b'\x19\x70')
        return frames
    bus.hook = hook
    with pytest.raises(ValueError):
        motor.send_position(.5)
    bus.hook = lambda request, frames: [status(kind=21)]
    with pytest.raises(RuntimeError, match='fault report'):
        motor.disable()


def test_deadline_includes_send_and_disables_without_queue_reads(rig):
    motor, bus, _ = rig
    bus.send_cost = .2
    with pytest.raises(TimeoutError):
        motor.prepare(**LIMITS)
    assert kinds(bus) == [4] and not bus.received


def test_no_destructor_commands_and_reconnect_requires_prepare(rig):
    motor, bus, _ = rig
    motor.prepare(**LIMITS)
    motor.close()
    motor.connect()
    with pytest.raises(RuntimeError, match='prepare'):
        motor.send_position(.5)
    count = len(bus.sent)
    other = CspMotor('fake', 2)
    del other
    gc.collect()
    assert len(bus.sent) == count


def test_target_read_timeout_cannot_enable(rig):
    motor, bus, _ = rig
    def hook(request, frames):
        if request.arbitration_id >> 24 == 17 and struct.unpack_from('<H', request.data)[0] == 0x7016:
            return []
        return frames
    bus.hook = hook
    with pytest.raises(TimeoutError):
        motor.prepare(**LIMITS)
    assert 3 not in kinds(bus) and motor._failed


@pytest.mark.parametrize('value', [1e100, 1e-100])
def test_unrepresentable_limit_no_commands(rig, value):
    motor, bus, _ = rig
    with pytest.raises(ValueError):
        motor.prepare(**(LIMITS | {'current_limit_a': value}))
    assert not bus.sent


def test_cleanup_one_joint_failure_does_not_touch_another(rig):
    motor, bus, clock = rig
    other_bus = MotorFake(clock)
    other = CspMotor('another-fake', 2)
    other.channel_handler = other_bus
    other._prepared = True
    bus.hook = lambda request, frames: [status(2)]
    bus.shutdown_error = OSError('shutdown failure')
    try:
        with pytest.raises(RuntimeError, match='mode'):
            motor.disable()
    finally:
        with pytest.raises(OSError, match='shutdown failure'):
            motor.close()
    assert motor.channel_handler is None and not motor._prepared
    assert other._prepared and not other_bus.sent and not other_bus.shutdown_calls
    other.close()
    assert not other_bus.sent and other_bus.shutdown_calls == 1


@pytest.mark.parametrize('value', [float('nan'), float('inf'), True, None, 4., -4.])
def test_invalid_position_no_write(rig, value):
    motor, bus, _ = rig
    motor.prepare(**LIMITS)
    count = len(bus.sent)
    with pytest.raises((ValueError, TypeError)):
        motor.send_position(value)
    assert len(bus.sent) == count
