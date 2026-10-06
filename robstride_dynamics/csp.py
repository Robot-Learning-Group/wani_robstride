"""Explicit, bounded private-protocol CSP control; no implicit motor commands.

RS02User Manual260713, sections 4.1.2–4.1.7 and 4.1.13: Type-2
faults occupy bits 16–21, mode bits 22–23; canTimeout 20000 is nominally
1 second. This is not verification of installed firmware or physical safety.
There is no transaction ID: a delayed identical status/read reply cannot be
proven fresh. Use an exclusive transport owner and external safety measures.
Runtime owners must explicitly disable, then close in a finally block.
"""

import math
import struct
import time

import can

from .position import PositionReader
from .protocol import CommunicationType as C, ParameterType as P

__all__ = ["CspMotor"]

_SETTINGS = (
    ("run_mode", P.MODE[0], "<B"),
    ("velocity_limit_rad_s", P.VELOCITY_LIMIT[0], "<f"),
    ("current_limit_a", P.CURRENT_LIMIT[0], "<f"),
    ("torque_limit_nm", P.TORQUE_LIMIT[0], "<f"),
    ("can_timeout_raw", P.CAN_TIMEOUT[0], "<I"),
    ("zero_state", P.ZERO_STATE[0], "<B"),
    ("raw_motor_position_rad", P.MECHANICAL_POSITION[0], "<f"),
)


class CspMotor(PositionReader):
    """Known-ID CSP session. Construction/connect/close send no commands.

    Failures poison the session and revoke preparation, including interruption
    after an enable may have reached the motor. Disable remains available even
    then. Each transaction has its own total deadline; prepare is multi-transaction.
    Transport backends must honor timeouts. Operations reject concurrent callers.
    """

    def __init__(self, channel: str, motor_id: int, bitrate: int = 1000000,
                     host_id: int = 255) -> None:
        """Configure a single address without opening CAN or sending commands."""
        super().__init__(channel, motor_id, bitrate, host_id)
        self._prepared = False
        self._can_timeout_raw = None

    @property
    def can_timeout_raw(self) -> int | None:
        """Last successfully observed watchdog register, not measured timing."""
        return self._can_timeout_raw

    def connect(self) -> None:
        """Open transport only; clear previous preparation metadata."""
        super().connect()
        self._prepared = False
        self._can_timeout_raw = None

    def close(self) -> None:
        """Close without implicit motor commands; the owner must disable first."""
        try:
            super().close()
        finally:
            if self.channel_handler is None:
                self._prepared = False
                self._can_timeout_raw = None

    @staticmethod
    def _number(value, name, *, positive=False):
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise TypeError(f"{name} must be a finite number")
        if not math.isfinite(value) or (positive and value <= 0):
            raise ValueError(f"{name} must be finite" + (" and positive" if positive else ""))
        return float(value)

    @staticmethod
    def _float32(value):
        try:
            result, = struct.unpack("<f", struct.pack("<f", value))
        except (OverflowError, struct.error) as exc:
            raise ValueError("Value is not representable as float32") from exc
        if not math.isfinite(result):
            raise ValueError("Value is not representable as finite float32")
        return result

    def _session(self):
        if self.channel_handler is None:
            raise RuntimeError("Not connected; call connect first")
        if self._failed:
            raise RuntimeError("Failed session; close and reconnect before control")

    def _settings_locked(self, timeout_s):
        # Public read_settings acquires the same non-reentrant transport lock.
        settings = {key: self._read_parameter(register, fmt, timeout_s)
                    for key, register, fmt in _SETTINGS}
        if settings["run_mode"] not in (0, 1, 2, 3, 5):
            raise ValueError("Unsupported run_mode")
        if settings["zero_state"] != 1:
            raise ValueError("CSP requires verified zero_state=1 (-pi..pi)")
        for key, _, _ in _SETTINGS[1:4]:
            if settings[key] <= 0:
                raise ValueError(f"Reported {key} must be positive")
        self._can_timeout_raw = settings["can_timeout_raw"]
        self._watchdog(timeout_s)
        self._branch(settings["raw_motor_position_rad"])
        return settings

    def _watchdog(self, timeout_s):
        if not self._can_timeout_raw or timeout_s >= self._can_timeout_raw / 20000.0:
            raise ValueError("Require positive CAN_TIMEOUT and timeout_s < raw/20000 nominal seconds")

    @staticmethod
    def _branch(position):
        if not -math.pi <= position <= math.pi:
            raise ValueError("Position outside verified -pi..pi branch")

    @staticmethod
    def _check_position_guard(position, guard, tolerance):
        if guard is None:
            return
        expected, lower, upper = guard
        if abs(position - expected) > tolerance:
            raise ValueError("Position shifted from runtime expected reference")
        if not lower + tolerance <= position <= upper - tolerance:
            raise ValueError("Position outside runtime approved bounds with tolerance clearance")

    def _command(self, kind, data, expected_mode, timeout_s, *, drain=True):
        """Caller owns lock. Disable deliberately bypasses poison and queue drain."""
        handler = self.channel_handler
        if handler is None:
            raise RuntimeError("Not connected; call connect first")
        deadline = time.monotonic() + timeout_s
        if drain:
            for _ in range(self._MAX_DRAIN_FRAMES):
                self._remaining(deadline)
                queued = handler.recv(timeout=0.0)
                self._remaining(deadline)
                if queued is None:
                    break
                self._check_fault_frame(queued)
            else:
                raise RuntimeError("Receive queue did not drain within 256 frames")
        handler.send(can.Message(
            arbitration_id=(kind << 24) | (self.host_id << 8) | self._motor_id,
            is_extended_id=True, data=data, check=True,
        ), timeout=self._remaining(deadline))
        self._remaining(deadline)
        for _ in range(self._MAX_DRAIN_FRAMES):
            frame = handler.recv(timeout=self._remaining(deadline))
            self._remaining(deadline)
            if frame is None:
                raise TimeoutError("No CSP status reply")
            self._check_fault_frame(frame)
            identifier = frame.arbitration_id
            if ((identifier >> 8) & 255) != self._motor_id or (identifier & 255) != self.host_id:
                continue
            reply_kind = (identifier >> 24) & 31
            if reply_kind != C.OPERATION_STATUS:
                continue
            if ((identifier >> 22) & 3) != expected_mode:
                raise RuntimeError(f"Unexpected Type-2 mode; require {expected_mode}")
            return
        raise RuntimeError("Status receive frame budget exhausted")

    def _write(self, register, value, fmt, mode, timeout_s):
        payload = struct.pack(fmt, value).ljust(4, b"\x00")
        self._command(C.WRITE_PARAMETER, struct.pack("<HH", register, 0) + payload,
                      mode, timeout_s)

    def prepare(self, *, velocity_limit_rad_s: float, current_limit_a: float,
                torque_limit_nm: float, position_tolerance_rad: float,
                timeout_s: float = 0.1, expected_position_rad: float | None = None,
                position_min_rad: float | None = None,
                position_max_rad: float | None = None) -> float:
        """Disable, verify conservative settings/hold target, enable, check position.

        All limits and tolerance are explicit positive finite values. Limits must
        not exceed reported settings (including float32 encoding). No watchdog,
        zero, ID, fault-clear, or flash writes occur. A failure does NOT implicitly
        disable: the owner must attempt disable even after partial enable.
        Optional expected/min/max positions must be supplied all-or-none. Each
        snapshot must remain within tolerance of the original expected reference
        and at least one tolerance inside the bounds, before and after enable.
        """
        self._validate_timeout(timeout_s)
        limits = [self._number(value, key, positive=True) for value, (key, _, _) in
                  zip((velocity_limit_rad_s, current_limit_a, torque_limit_nm), _SETTINGS[1:4])]
        encoded = [self._float32(value) for value in limits]
        if any(value <= 0 for value in encoded):
            raise ValueError("Limits must remain positive in float32")
        tolerance = self._number(position_tolerance_rad, "position_tolerance_rad", positive=True)
        guard_values = (expected_position_rad, position_min_rad, position_max_rad)
        guard = None
        if any(value is not None for value in guard_values):
            if any(value is None for value in guard_values):
                raise ValueError("Expected position and position bounds must be supplied all-or-none")
            guard = tuple(self._number(value, name) for value, name in zip(
                guard_values, ("expected_position_rad", "position_min_rad", "position_max_rad")))
            expected, lower, upper = guard
            if not -math.pi <= lower < upper <= math.pi:
                raise ValueError("Position bounds must be ordered within -pi..pi")
            self._check_position_guard(expected, guard, tolerance)
        self._acquire()
        try:
            self._session()
            self._prepared = False
            try:
                self._command(C.DISABLE, bytes(8), 0, timeout_s, drain=False)
                before = self._settings_locked(timeout_s)
                current = before["raw_motor_position_rad"]
                self._check_position_guard(current, guard, tolerance)
                for requested, wire, (key, _, _) in zip(limits, encoded, _SETTINGS[1:4]):
                    if max(requested, wire) > before[key]:
                        raise ValueError(f"Cannot increase reported {key}")
                self._write(P.MODE[0], 5, "<B", 0, timeout_s)
                for wire, (_, register, _) in zip(encoded, _SETTINGS[1:4]):
                    self._write(register, wire, "<f", 0, timeout_s)
                self._write(P.POSITION_TARGET[0], current, "<f", 0, timeout_s)
                after = self._settings_locked(timeout_s)
                self._check_position_guard(after["raw_motor_position_rad"], guard, tolerance)
                target = self._read_parameter(P.POSITION_TARGET[0], "<f", timeout_s)
                if after["run_mode"] != 5:
                    raise ValueError("CSP mode readback mismatch")
                for wire, (key, _, _) in zip(encoded, _SETTINGS[1:4]):
                    if not math.isclose(after[key], wire, rel_tol=2**-23, abs_tol=0.0) or after[key] > before[key]:
                        raise ValueError(f"Limit readback mismatch: {key}")
                if after["can_timeout_raw"] != before["can_timeout_raw"]:
                    raise ValueError("CAN_TIMEOUT changed during preparation")
                if abs(target - current) > tolerance or abs(after["raw_motor_position_rad"] - current) > tolerance:
                    raise ValueError("Disabled hold position readback mismatch")
                self._command(C.ENABLE, bytes(8), 2, timeout_s)
                position = self._read_parameter(P.MECHANICAL_POSITION[0], "<f", timeout_s)
                self._branch(position)
                self._check_position_guard(position, guard, tolerance)
                if abs(position - current) > tolerance:
                    raise ValueError("Post-enable movement exceeds position tolerance")
                self._prepared = True
                return position
            except BaseException:
                self._failed = True
                self._prepared = False
                raise
        finally:
            self._lock.release()

    def send_position(self, position_rad: float, *, timeout_s: float = 0.1) -> float:
        """Write loc_ref, require fault-free Motor status, read raw 0x7019 feedback."""
        self._validate_timeout(timeout_s)
        position = self._number(position_rad, "position_rad")
        self._branch(position)
        wire = self._float32(position)
        self._branch(wire)
        self._acquire()
        try:
            self._session()
            if not self._prepared:
                raise RuntimeError("Successful prepare required before position sends")
            try:
                self._watchdog(timeout_s)
                self._write(P.POSITION_TARGET[0], wire, "<f", 2, timeout_s)
                result = self._read_parameter(P.MECHANICAL_POSITION[0], "<f", timeout_s)
                self._branch(result)
                return result
            except BaseException:
                self._failed = True
                self._prepared = False
                raise
        finally:
            self._lock.release()

    def disable(self, *, timeout_s: float = 0.1) -> None:
        """Always attempt Type-4 (no fault clear), without pre-drain or poison gate.

        Reset status verification is bounded best effort; errors propagate. This
        cannot guarantee stopping a disconnected/faulted motor. A poisoned session
        remains poisoned after a successful disable.
        """
        self._validate_timeout(timeout_s)
        self._acquire()
        try:
            self._prepared = False
            try:
                self._command(C.DISABLE, bytes(8), 0, timeout_s, drain=False)
            except BaseException:
                self._failed = True
                raise
        finally:
            self._lock.release()
