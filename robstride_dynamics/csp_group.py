"""Exclusive, bounded multi-motor CSP transport (import explicitly).

No transaction IDs exist: even correlated replies cannot prove freshness.
Use an exclusive bus owner and external safety measures. Failure revokes
preparation; the owner must explicitly disable (also after partial enable),
then close. Neither failure nor close implicitly sends cleanup commands.
Transport backends must honor timeout arguments; opening/closing is unbounded.
"""

from contextlib import contextmanager
import math
import struct
import threading
import time
from typing import Callable

import can

from .csp import CspMotor, _SETTINGS
from .position import PositionReader
from .protocol import CommunicationType as C, ParameterType as P
from .table import MODEL_MIT_POSITION_TABLE

__all__ = ["CspGroup"]


class CspGroup:
    """One SocketCAN connection, selected IDs, nonblocking operation exclusion.

    Settings/preparation use one timeout per register or command batch, not
    per motor. Control and disable each use one deadline for the entire call.
    Invalid arguments and rejected concurrent calls do not poison the session.
    Operation failures do: close/connect and prepare again before control.
    """

    _FRAME_BUDGET = 4096
    _FIELDS = {"velocity_limit_rad_s", "current_limit_a", "torque_limit_nm",
               "position_tolerance_rad", "position_min_rad", "position_max_rad",
               "expected_position_rad"}

    def __init__(self, channel: str, motor_ids: dict[str, int],
                 bitrate: int = 1000000, host_id: int = 255,
                 motor_models: dict[str, str] | None = None) -> None:
        if not isinstance(motor_ids, dict) or not motor_ids:
            raise ValueError("motor_ids must be a nonempty name-to-ID dict")
        for name, motor_id in motor_ids.items():
            if not isinstance(name, str) or not name:
                raise ValueError("Motor names must be nonempty strings")
            # Reuse address validation without connecting or issuing commands.
            PositionReader(channel, motor_id, bitrate, host_id)
        if len(set(motor_ids.values())) != len(motor_ids):
            raise ValueError("Motor IDs must be unique")
        if motor_models is not None:
            if not isinstance(motor_models, dict) or motor_models.keys() != motor_ids.keys():
                raise ValueError("motor_models must contain exactly the selected motor names")
            for name, model in motor_models.items():
                if not isinstance(model, str) or model not in MODEL_MIT_POSITION_TABLE:
                    raise ValueError(f"Unsupported motor model for {name}: {model!r}")
        self.channel, self.bitrate, self.host_id = channel, bitrate, host_id
        self._motor_ids = dict(motor_ids)
        self._motor_models = None if motor_models is None else dict(motor_models)
        self._names_by_id = {value: name for name, value in motor_ids.items()}
        self.channel_handler = None
        self._lock = threading.Lock()
        self._prepared = False
        self._failed = False
        self._watchdogs = {}
        self._cancel = None
        self._phase = "idle"
        self._pending = set()
        self.last_error = None
        self.last_cycle = None
        self.last_disable_outcomes = None
        self.last_status_temperature_c: dict[str, float] = {}
        self.last_status_monotonic_s: dict[str, float] = {}

    def _acquire(self):
        if not self._lock.acquire(blocking=False):
            raise RuntimeError("Another group operation is in progress")

    def connect(self) -> None:
        """Open one filtered SocketCAN bus; no motor commands."""
        self._acquire()
        try:
            if self.channel_handler is not None:
                raise RuntimeError("Already connected; close before reconnecting")
            self.channel_handler = can.interface.Bus(
                interface="socketcan", channel=self.channel, bitrate=self.bitrate,
                ignore_config=True, can_filters=[
                    {"can_id": (motor_id << 8) | self.host_id,
                     "can_mask": 0xFFFF, "extended": True}
                    for motor_id in self._motor_ids.values()],
            )
            self._failed = self._prepared = False
            self._watchdogs = {}
            self.last_error = self.last_cycle = self.last_disable_outcomes = None
            self.last_status_temperature_c = {}
            self.last_status_monotonic_s = {}
        finally:
            self._lock.release()

    def close(self) -> None:
        """Detach and shut down only; idempotent and no implicit disable."""
        self._acquire()
        try:
            handler, self.channel_handler = self.channel_handler, None
            self._prepared = False
            self._watchdogs = {}
            if handler is not None:
                handler.shutdown()
        finally:
            self._lock.release()

    def _start_phase(self, phase):
        self._phase = phase
        self._pending = set(self._motor_ids)

    def _record_error(self):
        self._failed = True
        self._prepared = False
        self.last_error = {"phase": self._phase,
                           "pending_motors": [n for n in self._motor_ids if n in self._pending]}

    @contextmanager
    def _operation(self, phase):
        self._acquire()
        try:
            self._start_phase(phase)
            try:
                if self.channel_handler is None:
                    raise RuntimeError("Not connected; call connect first")
                if self._failed:
                    raise RuntimeError("Failed session; close and reconnect before control")
                yield
            except BaseException:
                self._record_error()
                raise
        finally:
            self._cancel = None
            self._lock.release()

    def _remaining(self, deadline):
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            raise TimeoutError(f"{self._phase} timed out")
        return remaining

    def _before_io(self, deadline):
        if self._cancel is not None:
            self._cancel()
        return self._remaining(deadline)

    def _send(self, name, kind, data, deadline):
        timeout = self._before_io(deadline)
        self.channel_handler.send(can.Message(
            arbitration_id=(kind << 24) | (self.host_id << 8) | self._motor_ids[name],
            is_extended_id=True, data=data, check=True), timeout=timeout)
        self._remaining(deadline)

    def _recv(self, deadline, *, drain=False):
        timeout = self._before_io(deadline)
        frame = self.channel_handler.recv(timeout=0.0 if drain else timeout)
        self._remaining(deadline)
        return frame

    def _address(self, frame):
        identifier = frame.arbitration_id
        if identifier & 255 != self.host_id:
            return None
        return self._names_by_id.get((identifier >> 8) & 255)

    @staticmethod
    def _format(frame):
        if (not frame.is_extended_id or frame.is_error_frame or frame.is_remote_frame
                or frame.is_fd or frame.bitrate_switch or frame.error_state_indicator
                or not 0 <= frame.arbitration_id <= 0x1FFFFFFF
                or frame.dlc != 8 or len(frame.data) != 8):
            raise ValueError("Malformed target classical CAN reply")

    def _inspect(self, frame):
        name = self._address(frame)
        if name is None:
            return None, None
        kind = (frame.arbitration_id >> 24) & 31
        if kind in (C.OPERATION_STATUS, C.FAULT_REPORT, C.READ_PARAMETER):
            self._format(frame)
        if kind == C.READ_PARAMETER:
            _, reserved = struct.unpack_from("<HH", frame.data)
            if (frame.arbitration_id >> 16) & 255 or reserved:
                raise ValueError("Malformed target parameter reply header")
        if kind == C.FAULT_REPORT:
            raise RuntimeError(f"Motor {name} fault report")
        if kind == C.OPERATION_STATUS and (frame.arbitration_id >> 16) & 63:
            raise RuntimeError(f"Motor {name} Type-2 fault flags set")
        return name, kind

    def _drain(self, deadline):
        for _ in range(self._FRAME_BUDGET):
            frame = self._recv(deadline, drain=True)
            if frame is None:
                return
            try:
                self._inspect(frame)
            except BaseException:
                name = self._address(frame)
                if name is not None:
                    self._pending.add(name)
                raise
        raise RuntimeError("Receive queue drain frame budget exhausted")

    def _collect(self, deadline, *, mode=None, register=None, fmt=None, disable=False,
                 status_temperature_c=None, status_receive_monotonic_s=None,
                 status_position_rad=None):
        results = {}
        for _ in range(self._FRAME_BUDGET):
            frame = self._recv(deadline)
            if frame is None:
                raise TimeoutError(f"{self._phase} timed out: missing replies")
            name, kind = self._inspect(frame)  # Faults from *any* selected motor.
            if name is None:
                continue
            if register is not None:
                if kind != C.READ_PARAMETER:
                    continue
                reply_register, reserved = struct.unpack_from("<HH", frame.data)
                if (frame.arbitration_id >> 16) & 255 or reserved:
                    raise ValueError("Malformed target parameter reply header")
                if reply_register != register:
                    continue
                value, = struct.unpack_from(fmt, frame.data, 4)
                if fmt == "<f" and not math.isfinite(value):
                    raise ValueError(f"Nonfinite parameter reply from {name}")
            else:
                if kind != C.OPERATION_STATUS:
                    continue
                actual = (frame.arbitration_id >> 22) & 3
                if disable and actual == 2:
                    continue
                if actual != mode:
                    raise RuntimeError(f"Unexpected Type-2 mode from {name}; require {mode}")
                value = None
            if name in self._pending:
                if register is None:
                    position_u16, _, _, temperature_u16 = struct.unpack(">HHHH", frame.data)
                    temperature_c = temperature_u16 * 0.1
                    if not math.isfinite(temperature_c):
                        raise ValueError(f"Nonfinite status temperature from {name}")
                    received_s = time.monotonic()
                    self.last_status_temperature_c[name] = temperature_c
                    self.last_status_monotonic_s[name] = received_s
                    if status_temperature_c is not None:
                        status_temperature_c[name] = temperature_c
                    if status_receive_monotonic_s is not None:
                        status_receive_monotonic_s[name] = received_s
                    if status_position_rad is not None:
                        model = self._motor_models[name]
                        status_position_rad[name] = ((position_u16 / 0x7FFF - 1) *
                                                     MODEL_MIT_POSITION_TABLE[model])
                results[name] = value
                self._pending.remove(name)
            if not self._pending:
                # The last required reply is not necessarily the last queued
                # frame. Inspect its tail under this same phase deadline.
                self._drain(deadline)
                return {n: results[n] for n in self._motor_ids}
        raise RuntimeError("Receive frame budget exhausted")

    def _read_batch(self, register, fmt, timeout_s, phase, *, deadline=None):
        self._start_phase(phase)
        if deadline is None:
            deadline = time.monotonic() + timeout_s
        self._drain(deadline)
        for name in self._motor_ids:
            self._send(name, C.READ_PARAMETER, struct.pack("<HHL", register, 0, 0), deadline)
        return self._collect(deadline, register=register, fmt=fmt)

    def _command_batch(self, kind, payloads, mode, timeout_s, phase, *, drain=True,
                       capture_status_positions=False):
        self._start_phase(phase)
        deadline = time.monotonic() + timeout_s
        if drain:
            self._drain(deadline)
        for name in self._motor_ids:
            self._send(name, kind, payloads[name], deadline)
        positions = {} if capture_status_positions else None
        self._collect(deadline, mode=mode, disable=kind == C.DISABLE,
                      status_position_rad=positions)
        return positions

    def _write_batch(self, register, values, fmt, timeout_s, phase):
        payloads = {n: struct.pack("<HH", register, 0) +
                    struct.pack(fmt, v).ljust(4, b"\0") for n, v in values.items()}
        self._command_batch(C.WRITE_PARAMETER, payloads, 0, timeout_s, phase)

    def _validate_settings(self, settings, timeout_s):
        for name, values in settings.items():
            if values["run_mode"] not in (0, 1, 2, 3, 5):
                raise ValueError(f"Unsupported run_mode from {name}")
            if values["zero_state"] != 1:
                raise ValueError(f"CSP requires zero_state=1 from {name}")
            for key, _, _ in _SETTINGS[1:4]:
                if values[key] <= 0:
                    raise ValueError(f"Reported {key} must be positive from {name}")
            raw = values["can_timeout_raw"]
            if raw <= 0 or timeout_s >= raw / 20000.0:
                raise ValueError("Require positive CAN_TIMEOUT and timeout_s < raw/20000 nominal seconds")
            CspMotor._branch(values["raw_motor_position_rad"])

    def _settings(self, timeout_s, prefix):
        result = {n: {} for n in self._motor_ids}
        for key, register, fmt in _SETTINGS:
            values = self._read_batch(register, fmt, timeout_s, f"{prefix}:{key}")
            for name, value in values.items():
                result[name][key] = value
        self._start_phase(f"{prefix}:validate")
        self._validate_settings(result, timeout_s)
        self._watchdogs = {n: v["can_timeout_raw"] for n, v in result.items()}
        return result

    def read_settings(self, *, timeout_s: float) -> dict[str, dict]:
        """Read seven CSP settings, batching all motors per register deadline."""
        PositionReader._validate_timeout(timeout_s)
        with self._operation("read_settings"):
            return self._settings(timeout_s, "read_settings")

    def _exact_keys(self, values, label):
        if not isinstance(values, dict) or values.keys() != self._motor_ids.keys():
            raise ValueError(f"{label} must contain exactly the selected motor names")

    def prepare(self, limits: dict[str, dict], *, timeout_s: float = .02,
                check_cancel: Callable[[], None] | None = None) -> dict[str, float]:
        """Disable/configure/verify all before batch-enable; check raw tracking.

        Each motor requires the six explicit SDK limit fields plus
        expected_position_rad. Runtime owners translate max_tracking_error_rad
        to position_tolerance_rad; runtime field names are not SDK aliases.
        All snapshots must have tolerance clearance inside the approved bounds.
        Cancellation is checked before every send/recv, including queue drains.
        No automatic cleanup: call disable in finally, even after partial enable.
        """
        PositionReader._validate_timeout(timeout_s)
        self._exact_keys(limits, "limits")
        if self._motor_models is None:
            raise ValueError("motor_models are required for preparation and control")
        if check_cancel is not None and not callable(check_cancel):
            raise TypeError("check_cancel must be callable or None")
        config = {}
        encoded = {}
        guards = {}
        for name, values in limits.items():
            if not isinstance(values, dict) or values.keys() != self._FIELDS:
                raise ValueError(f"{name} requires exactly {sorted(self._FIELDS)}")
            config[name] = {k: CspMotor._number(v, k, positive=k in (
                "velocity_limit_rad_s", "current_limit_a", "torque_limit_nm",
                "position_tolerance_rad")) for k, v in values.items()}
            v = config[name]
            guard = (v["expected_position_rad"], v["position_min_rad"], v["position_max_rad"])
            if not -math.pi <= guard[1] < guard[2] <= math.pi:
                raise ValueError("Position bounds must be ordered within -pi..pi")
            CspMotor._check_position_guard(guard[0], guard, v["position_tolerance_rad"])
            guards[name] = guard
            encoded[name] = {k: CspMotor._float32(v[k]) for k, _, _ in _SETTINGS[1:4]}
            if any(x <= 0 for x in encoded[name].values()):
                raise ValueError("Limits must remain positive in float32")
        with self._operation("prepare"):
            self._prepared = False
            self._cancel = check_cancel
            self._start_phase("prepare:disable")
            outcomes, error = self._disable_batch(time.monotonic() + timeout_s,
                                                   cancellable=True)
            if error is not None:
                self._pending = {n for n, value in outcomes.items() if value is not None}
                raise error
            before = self._settings(timeout_s, "prepare:before")
            initial = {n: v["raw_motor_position_rad"] for n, v in before.items()}
            self._start_phase("prepare:initial_guards")
            for name, values in before.items():
                CspMotor._check_position_guard(initial[name], guards[name],
                                              config[name]["position_tolerance_rad"])
                for key, _, _ in _SETTINGS[1:4]:
                    if max(config[name][key], encoded[name][key]) > values[key]:
                        raise ValueError(f"Cannot increase reported {key} for {name}")
            self._write_batch(P.MODE[0], {n: 5 for n in self._motor_ids}, "<B",
                              timeout_s, "prepare:write_mode")
            for key, register, fmt in _SETTINGS[1:4]:
                self._write_batch(register, {n: encoded[n][key] for n in self._motor_ids},
                                  fmt, timeout_s, f"prepare:write_{key}")
            self._write_batch(P.POSITION_TARGET[0], initial, "<f", timeout_s,
                              "prepare:write_hold")
            after = self._settings(timeout_s, "prepare:readback")
            targets = self._read_batch(P.POSITION_TARGET[0], "<f", timeout_s,
                                       "prepare:target_readback")
            self._start_phase("prepare:pre_enable_guards")
            for name, values in after.items():
                tolerance = config[name]["position_tolerance_rad"]
                CspMotor._branch(targets[name])
                CspMotor._check_position_guard(values["raw_motor_position_rad"],
                                              guards[name], tolerance)
                if values["run_mode"] != 5:
                    raise ValueError(f"CSP mode readback mismatch for {name}")
                for key, _, _ in _SETTINGS[1:4]:
                    if values[key] != encoded[name][key] or values[key] > before[name][key]:
                        raise ValueError(f"Limit readback mismatch: {name}:{key}")
                if values["can_timeout_raw"] != before[name]["can_timeout_raw"]:
                    raise ValueError(f"CAN_TIMEOUT changed for {name}")
                if (abs(targets[name] - initial[name]) > tolerance or
                        abs(values["raw_motor_position_rad"] - initial[name]) > tolerance or
                        abs(targets[name] - values["raw_motor_position_rad"]) > tolerance):
                    raise ValueError(f"Disabled hold position readback mismatch for {name}")
            enable_positions = self._command_batch(
                C.ENABLE, {n: bytes(8) for n in self._motor_ids}, 2, timeout_s,
                "prepare:enable", capture_status_positions=True)
            positions = self._read_batch(P.MECHANICAL_POSITION[0], "<f", timeout_s,
                                         "prepare:post_enable_positions")
            self._start_phase("prepare:post_enable_guards")
            for name, position in positions.items():
                tolerance = config[name]["position_tolerance_rad"]
                CspMotor._branch(position)
                if abs(enable_positions[name] - position) > tolerance:
                    raise ValueError(f"Enable status/register position mismatch for {name}")
                CspMotor._check_position_guard(position, guards[name], tolerance)
                if abs(position - initial[name]) > tolerance:
                    raise ValueError(f"Post-enable movement exceeds tolerance for {name}")
                if abs(position - targets[name]) > tolerance:
                    raise ValueError(f"Post-enable position differs from verified hold target for {name}")
            self._prepared = True
            self.last_error = None
            return positions

    def send_positions(self, targets: dict[str, float], *, timeout_s: float) -> dict[str, float]:
        """Drain, write all, collect Motor statuses, read all raw positions.

        One shared deadline includes every drain/send/receive. No receive occurs
        inside either send burst. last_cycle records SDK monotonic timing:
        per-name target_send_monotonic_s, target_burst_s, total_s.
        """
        PositionReader._validate_timeout(timeout_s)
        self._exact_keys(targets, "targets")
        if self._motor_models is None:
            raise ValueError("motor_models are required for preparation and control")
        wire = {}
        for name, value in targets.items():
            number = CspMotor._number(value, name)
            CspMotor._branch(number)
            wire[name] = CspMotor._float32(number)
            CspMotor._branch(wire[name])
        with self._operation("control:guards"):
            if not self._prepared:
                raise RuntimeError("Successful prepare required before position sends")
            for raw in self._watchdogs.values():
                if timeout_s >= raw / 20000.0:
                    raise ValueError("timeout_s must be less than nominal watchdog")
            start = time.monotonic()
            deadline = start + timeout_s
            self.last_cycle = {"target_send_monotonic_s": {}, "target_burst_s": 0., "total_s": 0.,
                               "status_temperature_c": {},
                               "status_receive_monotonic_s": {},
                               "status_position_rad": {}}
            try:
                self._start_phase("control:drain")
                self._drain(deadline)
                self._start_phase("control:target_status")
                burst = time.monotonic()
                try:
                    for name in self._motor_ids:
                        self._remaining(deadline)
                        self._send(name, C.WRITE_PARAMETER,
                                   struct.pack("<HHf", P.POSITION_TARGET[0], 0, wire[name]), deadline)
                        # A completed python-can send means the frame was accepted
                        # by the transport/driver. Runtime trajectory timing must
                        # not use the earlier start of a potentially blocking call.
                        self.last_cycle["target_send_monotonic_s"][name] = time.monotonic()
                finally:
                    self.last_cycle["target_burst_s"] = time.monotonic() - burst
                self._collect(
                    deadline, mode=2,
                    status_temperature_c=self.last_cycle["status_temperature_c"],
                    status_receive_monotonic_s=self.last_cycle["status_receive_monotonic_s"],
                    status_position_rad=self.last_cycle["status_position_rad"],
                )
                self.last_error = None
                return dict(self.last_cycle["status_position_rad"])
            finally:
                self.last_cycle["total_s"] = time.monotonic() - start

    def _disable_batch(self, deadline, *, cancellable=False):
        """Fail-closed receive boundary, unconditional send-all, then outcomes.

        No timestamp comparison: CAN timestamps and monotonic time have different
        domains. A drained queue excludes known buffered replies only, never
        identical replies still in flight. Cleanup intentionally catches even
        KeyboardInterrupt/SystemExit so another motor's disable is not skipped.
        """
        handler = self.channel_handler
        results = {}
        first_error = None

        def remember(exc):
            nonlocal first_error
            if first_error is None:
                first_error = exc
            return f"{type(exc).__name__}: {exc}"

        def fail(name, exc):
            text = remember(exc)
            if results.get(name) is None:
                results[name] = text
            self._pending.discard(name)

        def fail_all(exc):
            for name in self._motor_ids:
                fail(name, exc)

        def before_io():
            # Once a preparation failure occurs, this is cleanup: cancellation
            # must no longer prevent further emergency sends/receives.
            if cancellable and first_error is None and self._cancel is not None:
                self._cancel()
            return self._remaining(deadline)

        def receive(*, drain):
            timeout = before_io()
            if handler is None:
                raise RuntimeError("Not connected")
            frame = handler.recv(timeout=0.0 if drain else timeout)
            self._remaining(deadline)
            return frame

        def inspect(frame):
            name = self._address(frame)
            try:
                return self._inspect(frame)
            except BaseException as exc:
                if name is None:
                    raise
                fail(name, exc)
                return name, None

        def drain_queue():
            # Keep the emergency pre-send boundary short even with a large
            # control budget. All calls are nonblocking and deadline bounded.
            for _ in range(min(256, self._FRAME_BUDGET)):
                frame = receive(drain=True)
                if frame is None:
                    return
                inspect(frame)
            raise RuntimeError("Disable receive queue drain frame budget exhausted")

        boundary = False
        try:
            drain_queue()
            boundary = True
        except BaseException as exc:
            fail_all(exc)
        finally:
            # Never let a failed/busy queue or an interruption suppress sends.
            # No receive is allowed between any of these send attempts.
            for name, motor_id in self._motor_ids.items():
                if cancellable and first_error is None and self._cancel is not None:
                    try:
                        self._cancel()
                    except BaseException as exc:
                        fail_all(exc)
                try:
                    if handler is None:
                        raise RuntimeError("Not connected")
                    handler.send(can.Message(
                        arbitration_id=(C.DISABLE << 24) | (self.host_id << 8) | motor_id,
                        is_extended_id=True, data=bytes(8), check=True),
                        timeout=max(0., deadline - time.monotonic()))
                except BaseException as exc:
                    fail(name, exc)

        if boundary:
            try:
                for _ in range(self._FRAME_BUDGET):
                    if not self._pending:
                        break
                    frame = receive(drain=False)
                    if frame is None:
                        raise TimeoutError("Disable unconfirmed: missing Reset reply")
                    name, kind = inspect(frame)
                    if kind != C.OPERATION_STATUS:
                        continue
                    mode = (frame.arbitration_id >> 22) & 3
                    if mode == 2:
                        continue
                    if mode != 0:
                        fail(name, RuntimeError("Unexpected Type-2 mode; require Reset"))
                    elif name in self._pending:
                        results[name] = None
                        self._pending.remove(name)
                if self._pending:
                    raise RuntimeError("Disable unconfirmed: receive frame budget exhausted")
                # Faults after the final Reset override even confirmed outcomes.
                try:
                    drain_queue()
                except BaseException as exc:
                    fail_all(exc)
            except BaseException as exc:
                remember(exc)
                for name in list(self._pending):
                    fail(name, exc)

        outcomes = {n: results[n] for n in self._motor_ids}
        return outcomes, first_error

    def disable(self, *, timeout_s: float) -> dict[str, str | None]:
        """Bounded nonblocking pre-drain, unconditional send-all, confirm Reset.

        ONE group deadline includes both drains, sends and collection. A failed
        receive boundary cannot confirm any motor but still attempts every send.
        Poison/cancellation are bypassed. BaseException (including interrupts)
        becomes an error string rather than aborting another motor's cleanup.
        last_disable_outcomes retains the returned per-motor mapping. No reply
        can prove freshness of an identical in-flight frame or physical stopping.
        """
        PositionReader._validate_timeout(timeout_s)
        self._acquire()
        try:
            self._prepared = False
            self._start_phase("disable")
            outcomes, error = self._disable_batch(time.monotonic() + timeout_s)
            self.last_disable_outcomes = dict(outcomes)
            if error is not None:
                self._pending = {n for n, value in outcomes.items() if value is not None}
                self._record_error()
            return outcomes
        except BaseException:
            self._record_error()
            raise
        finally:
            self._lock.release()
