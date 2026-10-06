"""Additive Lab API for bounded read-only position and settings requests.

Original SDK APIs remain unchanged. Import this module explicitly to use the
Lab-owned transaction and no-command cleanup implementation.
"""

import math
import struct
import threading
import time

import can

from .protocol import CommunicationType, ParameterType


class PositionReader:
    """Read raw mechanical position and settings without changing motor state.

    Only type-17 parameter reads are sent. No RobstrideBus instance,
    scan, enable, disable, parameter write, or destructor command is used.
    Operations reject concurrency rather than waiting for another operation.
    A failed request poisons the session: explicitly close and reconnect before
    retrying. The protocol has no transaction ID; even after draining queued
    frames, a delayed identical reply cannot be absolutely excluded.
    """

    _MAX_DRAIN_FRAMES = 256

    def __init__(
        self,
        channel: str,
        motor_id: int,
        bitrate: int = 1000000,
        host_id: int = 0xFF,
    ) -> None:
        """Configure a known motor ID; construction performs no CAN I/O."""
        if not isinstance(channel, str) or not channel:
            raise ValueError("channel must be a nonempty string")
        if type(motor_id) is not int or not 1 <= motor_id <= 0xFF:
            raise ValueError("motor ID must be an integer in 1..255")
        if type(host_id) is not int or not 0 <= host_id <= 0xFF:
            raise ValueError("host ID must be an integer in 0..255")
        if type(bitrate) is not int or bitrate <= 0:
            raise ValueError("bitrate must be a positive integer")
        self.channel = channel
        self.bitrate = bitrate
        self.host_id = host_id
        self._motor_id = motor_id
        self.channel_handler: can.BusABC | None = None
        self._failed = False
        self._lock = threading.Lock()

    def _acquire(self) -> None:
        if not self._lock.acquire(blocking=False):
            raise RuntimeError("Another reader operation is in progress")

    def connect(self) -> None:
        """Open SocketCAN without a handshake or any motor command.

        Raises RuntimeError if already connected or an operation is in progress.
        Transport opening, like shutdown, has no hard time bound.
        """
        self._acquire()
        try:
            if self.channel_handler is not None:
                raise RuntimeError("Already connected; close before reconnecting")
            self.channel_handler = can.interface.Bus(
                interface="socketcan", channel=self.channel, bitrate=self.bitrate,
                ignore_config=True,
            )
            self._failed = False
        finally:
            self._lock.release()

    def close(self) -> None:
        """Detach and shut down the transport; never send a motor command.

        Idempotent, including after a shutdown exception. The handler is cleared
        before shutdown; exceptions propagate and shutdown cannot be promised a
        hard time bound. An in-progress operation is rejected, not interrupted.
        """
        self._acquire()
        try:
            handler = self.channel_handler
            self.channel_handler = None
            if handler is not None:
                handler.shutdown()
        finally:
            self._lock.release()

    @staticmethod
    def _remaining(deadline: float, context: str) -> float:
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            raise TimeoutError(f"{context} timed out")
        return remaining

    def read_position(self, *, timeout_s: float = 0.1) -> float:
        """Return finite, uncalibrated mechanical position in radians.

        A positive finite timeout covers queue drain, send and receive using one
        monotonic deadline. Transport calls must honor their timeout arguments;
        Python cannot hard-bound a misbehaving backend. At most 256 preexisting
        frames are drained nonblocking; a nonempty queue at that limit fails
        without sending. Target Type-21 reports, Type-2 fault flags and malformed
        target status/fault frames fail even during drain. Unrelated traffic is
        ignored only until the deadline.

        TimeoutError indicates deadline expiry; ValueError indicates a malformed
        target reply; RuntimeError indicates a target fault or lifecycle error.
        Transport errors propagate. Any request failure requires close/connect;
        invalid arguments and rejected concurrent calls do not poison a session.
        """
        self._validate_timeout(timeout_s)
        self._acquire()
        try:
            return self._read_parameter(
                ParameterType.MECHANICAL_POSITION[0], "<f", timeout_s
            )
        finally:
            self._lock.release()

    @staticmethod
    def _validate_timeout(timeout_s: float) -> None:
        if isinstance(timeout_s, bool) or not isinstance(timeout_s, (int, float)):
            raise TypeError("timeout_s must be a positive finite number")
        if not math.isfinite(timeout_s) or timeout_s <= 0:
            raise ValueError("timeout_s must be a positive finite number")

    def read_settings(self, *, timeout_s: float = 0.1) -> dict[str, int | float]:
        """Read seven registers sequentially; return only a complete result.

        Each register gets its own bounded timeout (about 7 * timeout_s total).
        Values are raw motor settings, not defaults or a calibrated joint state.
        CAN timeout is an unsigned raw integer, with no assumed seconds conversion.
        The reads are not an atomic motor snapshot. Any request or validation
        failure poisons the session and requires close/connect before retrying.
        Invalid timeouts and rejected concurrent calls do not poison the session.
        """
        self._validate_timeout(timeout_s)
        self._acquire()
        try:
            settings = {}
            registers = (
                ("run_mode", ParameterType.MODE[0], "<B"),
                ("velocity_limit_rad_s", ParameterType.VELOCITY_LIMIT[0], "<f"),
                ("current_limit_a", ParameterType.CURRENT_LIMIT[0], "<f"),
                ("torque_limit_nm", ParameterType.TORQUE_LIMIT[0], "<f"),
                ("can_timeout_raw", ParameterType.CAN_TIMEOUT[0], "<I"),
                ("zero_state", ParameterType.ZERO_STATE[0], "<B"),
                ("raw_motor_position_rad", ParameterType.MECHANICAL_POSITION[0], "<f"),
            )
            for key, parameter, format in registers:
                value = self._read_parameter(parameter, format, timeout_s)
                try:
                    if key == "run_mode" and value not in (0, 1, 2, 3, 5):
                        raise ValueError(f"Unsupported run_mode raw value: {value}")
                    if key == "zero_state" and value not in (0, 1):
                        raise ValueError(f"Unsupported zero_state raw value: {value}")
                    if key in ("velocity_limit_rad_s", "current_limit_a",
                               "torque_limit_nm") and value < 0:
                        raise ValueError(f"Negative {key} reply: {value}")
                except BaseException:
                    self._failed = True
                    raise
                settings[key] = value
            return settings
        finally:
            self._lock.release()

    def _check_fault_frame(self, frame: can.Message) -> None:
        """Reject addressed faults even in queued traffic unrelated to a read.

        Caller holds the transport lock and owns deadline/poison handling.
        A Type-21 report is always a failure, including zero-valued reports;
        valid fault-free Type-2 status is not a parameter reply.
        """
        identifier = frame.arbitration_id
        if ((identifier >> 8) & 0xFF) != self._motor_id or (identifier & 0xFF) != self.host_id:
            return
        kind = (identifier >> 24) & 0x1F
        if kind not in (CommunicationType.OPERATION_STATUS, CommunicationType.FAULT_REPORT):
            return
        if (not frame.is_extended_id or frame.is_error_frame
                or frame.is_remote_frame or frame.is_fd
                or frame.bitrate_switch or frame.error_state_indicator
                or not 0 <= identifier <= 0x1FFFFFFF
                or frame.dlc != 8 or len(frame.data) != 8):
            raise ValueError("Malformed target classical CAN reply")
        if kind == CommunicationType.FAULT_REPORT:
            fault, warning = struct.unpack("<LL", frame.data)
            raise RuntimeError(
                f"Motor {self._motor_id} fault report: "
                f"fault=0x{fault:08x}, warning=0x{warning:08x}"
            )
        if (identifier >> 16) & 0x3F:
            raise RuntimeError(f"Motor {self._motor_id} Type-2 fault flags set")

    def _read_parameter(
        self, parameter: int, format: str, timeout_s: float
    ) -> int | float:
        """Run one correlated transaction while the caller holds the reader lock."""
        handler = self.channel_handler
        if handler is None:
            raise RuntimeError("Not connected; call connect first")
        if self._failed:
            raise RuntimeError("Failed session; close and reconnect before reading")
        try:
            deadline = time.monotonic() + timeout_s
            context = f"Register read 0x{parameter:04x}"

            def remaining():
                return self._remaining(deadline, context)

            for _ in range(self._MAX_DRAIN_FRAMES):
                remaining()
                queued = handler.recv(timeout=0.0)
                remaining()
                if queued is None:
                    break
                self._check_fault_frame(queued)
            else:
                raise RuntimeError("Receive queue did not drain within 256 frames")

            request = can.Message(
                arbitration_id=(CommunicationType.READ_PARAMETER << 24)
                | (self.host_id << 8) | self._motor_id,
                is_extended_id=True,
                data=struct.pack("<HHL", parameter, 0, 0),
                check=True,
            )
            handler.send(request, timeout=remaining())
            remaining()
            while True:
                frame = handler.recv(timeout=remaining())
                remaining()
                if frame is None:
                    raise TimeoutError(f"{context} timed out: no reply")
                self._check_fault_frame(frame)
                identifier = frame.arbitration_id
                source = (identifier >> 8) & 0xFF
                destination = identifier & 0xFF
                kind = (identifier >> 24) & 0x1F
                if source != self._motor_id or destination != self.host_id:
                    continue
                if kind != CommunicationType.READ_PARAMETER:
                    continue
                if (not frame.is_extended_id or frame.is_error_frame
                        or frame.is_remote_frame or frame.is_fd
                        or frame.bitrate_switch or frame.error_state_indicator
                        or not 0 <= identifier <= 0x1FFFFFFF
                        or frame.dlc != 8 or len(frame.data) != 8):
                    raise ValueError("Malformed target classical CAN reply")

                register, reserved = struct.unpack("<HH", frame.data[:4])
                if register != parameter:
                    continue
                if (identifier >> 16) & 0xFF or reserved != 0:
                    raise ValueError("Malformed target parameter reply header")
                value, = struct.unpack_from(format, frame.data, 4)
                if format == "<f" and not math.isfinite(value):
                    if parameter == ParameterType.MECHANICAL_POSITION[0]:
                        raise ValueError("Nonfinite mechanical position reply")
                    raise ValueError(f"Nonfinite parameter 0x{parameter:04x} reply")
                return value
        except BaseException:
            self._failed = True
            raise
