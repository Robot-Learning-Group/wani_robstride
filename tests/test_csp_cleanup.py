"""Bounded cleanup confirmation using fake CAN only; no watchdog assumptions."""
import pytest

from test_csp import LIMITS, kinds, rig, status


def delayed_reset(monkeypatch, bus, clock, delay):
    """Model a backend waiting for Reset while honoring the receive timeout."""
    ready = clock.now + delay
    original_recv = bus.recv
    bus.hook = lambda request, frames: []

    def recv(timeout):
        assert kinds(bus) == [4]  # Send first, including in a poisoned session.
        assert timeout > 0  # Never pre-drain.
        if bus.queued:
            return original_recv(timeout)
        bus.received.append(timeout)
        if ready < clock.now + timeout:
            clock.now = ready
            return status()
        clock.now += timeout
        return None

    monkeypatch.setattr(bus, 'recv', recv)


@pytest.mark.parametrize('poisoned', [False, True])
@pytest.mark.parametrize('queued_motor', [False, True])
def test_disable_delayed_reset_with_selected_budget(rig, monkeypatch, poisoned, queued_motor):
    motor, bus, clock = rig
    motor._failed = poisoned
    motor._prepared = True
    motor._can_timeout_raw = 0  # Cleanup neither gates on nor assumes a watchdog.
    if queued_motor:
        bus.queued.append(status(2))
    bus.queued.extend([status(motor=2), status(host=254)])
    bus.send_cost = .015
    start = clock.now
    delayed_reset(monkeypatch, bus, clock, .065)
    motor.disable(timeout_s=.1)
    assert clock.now - start == pytest.approx(.065)  # Well beyond 20 ms.
    assert bus.sent[0][1] == pytest.approx(.1)
    assert bus.received[0] == pytest.approx(.085)  # Send shares the budget.
    assert motor._failed is poisoned and not motor._prepared
    assert kinds(bus) == [4] and bus.sent[0][0].data == bytes(8)


def test_disable_reset_after_selected_deadline_is_not_success(rig, monkeypatch):
    motor, bus, clock = rig
    start = clock.now
    delayed_reset(monkeypatch, bus, clock, .065)
    with pytest.raises(TimeoutError, match='CSP disable Reset status confirmation timed out'):
        motor.disable(timeout_s=.02)
    assert clock.now - start == pytest.approx(.02)
    assert motor._failed and not motor._prepared


@pytest.mark.parametrize('frames', [[], [status(2)], [status(motor=2)],
                                   [status(host=254)], [status(2), status(motor=2), status(host=254)]])
def test_disable_requires_target_reset_not_motor_or_other_ids(rig, frames):
    motor, bus, clock = rig
    bus.hook = lambda request, replies: frames
    start = clock.now
    with pytest.raises(TimeoutError, match='CSP disable Reset status confirmation timed out'):
        motor.disable(timeout_s=.1)
    assert clock.now - start == pytest.approx(.1)
    assert kinds(bus) == [4] and motor._failed and not motor._prepared


@pytest.mark.parametrize('bad', [status(2, faults=1 << bit) for bit in range(6)] +
                         [status(kind=21), status(2, dlc=7), status(2, is_fd=True)])
def test_disable_does_not_skip_faulted_or_malformed_motor_before_reset(rig, bad):
    motor, bus, _ = rig
    bus.queued.extend([status(2), bad])
    with pytest.raises((RuntimeError, ValueError), match='fault|Malformed'):
        motor.disable()
    assert kinds(bus) == [4] and motor._failed
    assert bus.queued  # The later Reset cannot override a fault.


@pytest.mark.parametrize('mode', [1, 3])
def test_disable_still_rejects_other_target_modes(rig, mode):
    motor, bus, _ = rig
    bus.hook = lambda request, frames: [status(mode), status()]
    with pytest.raises(RuntimeError, match='Unexpected Type-2 mode'):
        motor.disable()
    assert motor._failed


def test_disable_motor_flood_has_frame_bound_without_clock_progress(rig):
    motor, bus, clock = rig
    bus.recv_cost = 0
    bus.hook = lambda request, frames: []
    bus.flood = status(2)
    start = clock.now
    with pytest.raises(RuntimeError, match='Status receive frame budget exhausted'):
        motor.disable()
    assert len(bus.received) == 256 and clock.now == start
    assert kinds(bus) == [4] and motor._failed


def test_disable_send_overrun_has_disable_context(rig):
    motor, bus, _ = rig
    bus.send_cost = .11
    with pytest.raises(TimeoutError, match='CSP disable Reset status confirmation timed out'):
        motor.disable(timeout_s=.1)
    assert kinds(bus) == [4] and not bus.received


def test_enable_timeout_has_status_not_register_context(rig):
    motor, bus, _ = rig
    bus.hook = lambda request, frames: [] if request.arbitration_id >> 24 == 3 else frames
    with pytest.raises(TimeoutError, match=r'CSP command Type-3 status \(mode 2\) timed out'):
        motor.prepare(**LIMITS)
    assert motor._failed


@pytest.mark.parametrize('method,register', [('read_position', '7019'), ('read_settings', '7005')])
def test_register_timeouts_identify_register(rig, method, register):
    motor, bus, _ = rig
    bus.hook = lambda request, frames: []
    with pytest.raises(TimeoutError, match=f'Register read 0x{register} timed out'):
        getattr(motor, method)()
    assert kinds(bus) == [17] and motor._failed
