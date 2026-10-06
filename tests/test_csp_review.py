"""Regression tests for runtime reference guards and non-discardable faults."""
import math
import struct

import pytest

from test_csp import LIMITS, kinds, rig, status
from test_position import assert_poisoned, connect_with, reply, setup

GUARD = dict(expected_position_rad=.25, position_min_rad=.20, position_max_rad=.30)


def test_guarded_prepare_preserves_order(rig):
    motor, bus, _ = rig
    assert motor.prepare(**LIMITS, **GUARD) == .25
    assert kinds(bus) == [4] + [17]*7 + [18]*5 + [17]*8 + [3, 17]


@pytest.mark.parametrize('provided', [
    {'expected_position_rad': .25}, {'position_min_rad': .2},
    {'position_max_rad': .3}, {'expected_position_rad': .25, 'position_min_rad': .2},
    {'expected_position_rad': .25, 'position_max_rad': .3},
    {'position_min_rad': .2, 'position_max_rad': .3},
])
def test_partial_guard_no_commands(rig, provided):
    motor, bus, _ = rig
    with pytest.raises(ValueError, match='all-or-none'):
        motor.prepare(**LIMITS, **provided)
    assert not bus.sent and not motor._failed


@pytest.mark.parametrize('key', list(GUARD))
@pytest.mark.parametrize('bad', [float('nan'), float('inf'), -float('inf'), True, '0.25'])
def test_guard_finite_numbers_before_commands(rig, key, bad):
    motor, bus, _ = rig
    with pytest.raises((TypeError, ValueError)):
        motor.prepare(**LIMITS, **(GUARD | {key: bad}))
    assert not bus.sent and not motor._failed


@pytest.mark.parametrize('guard', [
    GUARD | {'position_min_rad': .30}, GUARD | {'position_min_rad': .31},
    GUARD | {'position_min_rad': -math.pi - .01},
    GUARD | {'position_max_rad': math.pi + .01},
    GUARD | {'expected_position_rad': .19}, GUARD | {'expected_position_rad': .31},
    GUARD | {'expected_position_rad': .205}, GUARD | {'expected_position_rad': .295},
    GUARD | {'position_min_rad': .245, 'position_max_rad': .255},
])
def test_guard_order_and_clearance_before_commands(rig, guard):
    motor, bus, _ = rig
    with pytest.raises(ValueError):
        motor.prepare(**LIMITS, **guard)
    assert not bus.sent and not motor._failed


@pytest.mark.parametrize('phase', ['before_disable', 'on_disable', 'disabled_readback'])
@pytest.mark.parametrize('position,guard', [
    (.265, GUARD),  # Inside approved bounds, outside original reference tolerance.
    (.257, GUARD | {'position_max_rad': .265}),  # Within reference tolerance, lacks clearance.
    (.243, GUARD | {'position_min_rad': .235}),
    (.35, GUARD),  # Outside actual bounds.
])
def test_shifted_snapshot_no_enable(rig, phase, position, guard):
    motor, bus, _ = rig
    assert motor.read_settings()['raw_motor_position_rad'] == guard['expected_position_rad']
    bus.sent.clear()
    if phase == 'before_disable':
        bus.values[0x7019] = position
    def hook(request, frames):
        kind = request.arbitration_id >> 24
        if ((phase == 'on_disable' and kind == 4) or
                (phase == 'disabled_readback' and kind == 18 and
                 struct.unpack_from('<H', request.data)[0] == 0x7016)):
            bus.values[0x7019] = position
        return frames
    bus.hook = hook
    with pytest.raises(ValueError, match='runtime'):
        motor.prepare(**LIMITS, **guard)
    assert 3 not in kinds(bus) and motor._failed and not motor._prepared
    if phase != 'disabled_readback':
        assert 18 not in kinds(bus)


@pytest.mark.parametrize('phase', ['disabled_readback', 'post_enable'])
def test_guard_not_rebased_to_first_sdk_snapshot(rig, phase):
    motor, bus, _ = rig
    bus.values[0x7019] = .257  # Valid relative to .25 original expected reference.
    def hook(request, frames):
        kind = request.arbitration_id >> 24
        if ((phase == 'post_enable' and kind == 3) or
                (phase == 'disabled_readback' and kind == 18 and
                 struct.unpack_from('<H', request.data)[0] == 0x7016)):
            bus.values[0x7019] = .264  # Only .007 from SDK snapshot, but .014 from runtime.
        return frames
    bus.hook = hook
    with pytest.raises(ValueError, match='expected reference'):
        motor.prepare(**LIMITS, **GUARD)
    assert (3 in kinds(bus)) == (phase == 'post_enable')
    assert motor._failed and not motor._prepared
    bus.hook = None
    motor.disable()


def test_post_enable_clearance_check(rig):
    motor, bus, _ = rig
    def hook(request, frames):
        if request.arbitration_id >> 24 == 3:
            bus.values[0x7019] = .257  # Below tolerance, but beyond clearance-shrunken upper bound.
        return frames
    bus.hook = hook
    with pytest.raises(ValueError, match='clearance'):
        motor.prepare(**LIMITS, **(GUARD | {'position_max_rad': .265}))
    assert 3 in kinds(bus) and motor._failed and not motor._prepared


@pytest.mark.parametrize('kind,faults', [(21, 0)] + [(2, 1 << bit) for bit in range(6)])
@pytest.mark.parametrize('phase', ['drain', 'receive'])
def test_parameter_fault_frames_poison(setup, kind, faults, phase):
    reader, bus = connect_with(setup, [reply()])
    fault = status(kind=kind, faults=faults)
    if phase == 'drain':
        bus.queued.append(fault)
    else:
        bus.replies = [fault, reply()]
    with pytest.raises(RuntimeError, match='fault'):
        reader.read_position()
    assert len(bus.sent) == (phase == 'receive')
    assert_poisoned(reader, bus)


@pytest.mark.parametrize('kind', [2, 21])
@pytest.mark.parametrize('phase', ['drain', 'receive'])
@pytest.mark.parametrize('change', [
    {'is_extended_id': False}, {'is_fd': True}, {'is_remote_frame': True},
    {'is_error_frame': True}, {'bitrate_switch': True},
    {'error_state_indicator': True}, {'dlc': 7}, {'data': b''},
])
def test_malformed_fault_frames_not_discarded(setup, kind, phase, change):
    reader, bus = connect_with(setup, [reply()])
    fault = status(kind=kind, faults=1)
    for key, value in change.items():
        setattr(fault, key, value)
    if phase == 'drain':
        bus.queued.append(fault)
    else:
        bus.replies = [fault, reply()]
    with pytest.raises(ValueError, match='Malformed'):
        reader.read_position()
    assert_poisoned(reader, bus)


@pytest.mark.parametrize('unrelated', [
    status(kind=21, motor=2), status(faults=63, motor=2),
    status(kind=21, host=254), status(faults=63, host=254),
    status(kind=19, faults=63), status(kind=21, motor=2, is_fd=True),
    status(), status(2),
])
def test_parameter_unrelated_faults_and_clean_status_ignored(setup, unrelated):
    reader, bus = connect_with(setup, [unrelated, reply(.375)])
    bus.queued.append(unrelated)
    assert reader.read_position() == .375 and not reader._failed


@pytest.mark.parametrize('kind,faults', [(21, 0), (2, 1)])
def test_fault_after_write_status_before_parameter_poll(rig, kind, faults):
    motor, bus, _ = rig
    motor.prepare(**LIMITS)
    count = len(bus.sent)
    bus.hook = lambda request, frames: frames + [status(kind=kind, faults=faults)]
    with pytest.raises(RuntimeError, match='fault'):
        motor.send_position(.5)
    assert kinds(bus)[count:] == [18]  # Fault must stop parameter poll at drain.
    assert motor._failed and not motor._prepared


@pytest.mark.parametrize('kind,faults', [(21, 0), (2, 1)])
def test_queued_fault_before_enable_no_enable(rig, kind, faults):
    motor, bus, _ = rig
    def hook(request, frames):
        if request.arbitration_id >> 24 == 17 and struct.unpack_from('<H', request.data)[0] == 0x7016:
            return frames + [status(kind=kind, faults=faults)]
        return frames
    bus.hook = hook
    with pytest.raises(RuntimeError, match='fault'):
        motor.prepare(**LIMITS)
    assert 3 not in kinds(bus) and motor._failed and not motor._prepared


@pytest.mark.parametrize('kind,faults', [(21, 0), (2, 1)])
def test_queued_fault_before_disabled_writes_no_write(rig, kind, faults):
    motor, bus, _ = rig
    def hook(request, frames):
        if request.arbitration_id >> 24 == 17 and struct.unpack_from('<H', request.data)[0] == 0x7019:
            return frames + [status(kind=kind, faults=faults)]
        return frames
    bus.hook = hook
    with pytest.raises(RuntimeError, match='fault'):
        motor.prepare(**LIMITS)
    assert 18 not in kinds(bus) and 3 not in kinds(bus) and motor._failed


@pytest.mark.parametrize('fault', [status(kind=21), status(faults=1),
                                  status(kind=21, dlc=7), status(faults=1, is_fd=True)])
def test_disable_sends_first_then_queued_fault_fails(rig, fault):
    motor, bus, _ = rig
    motor._failed = True
    bus.queued.append(fault)
    def before_receive(timeout):
        assert kinds(bus) == [4] and timeout > 0  # No pre-drain.
    bus.recv_hook = before_receive
    with pytest.raises((RuntimeError, ValueError)):
        motor.disable()
    assert kinds(bus) == [4] and motor._failed
    motor.close()
    assert bus.shutdown_calls == 1


@pytest.mark.parametrize('unrelated', [status(kind=21, motor=2), status(faults=63, host=254),
                                      status(kind=19, faults=63)])
def test_command_drain_and_receive_ignore_unrelated_faults(rig, unrelated):
    motor, bus, _ = rig
    motor.prepare(**LIMITS)
    bus.queued.append(unrelated)
    bus.hook = lambda request, frames: [unrelated] + frames
    assert motor.send_position(.5) == .25 and not motor._failed
