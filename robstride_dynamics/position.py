"""Additive Lab API for bounded read-only position requests.

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
    """Read raw mechanical position (radians), without changing motor state.

    Only type-17 reads of register 0x7019 are sent. No RobstrideBus instance,
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
    def _remaining(deadline: float) -> float:
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            raise TimeoutError("Mechanical position request timed out")
        return remaining

    def read_position(self, *, timeout_s: float = 0.1) -> float:
        """Return finite, uncalibrated mechanical position in radians.

        A positive finite timeout covers queue drain, send and receive using one
        monotonic deadline. Transport calls must honor their timeout arguments;
        Python cannot hard-bound a misbehaving backend. At most 256 preexisting
        frames are drained nonblocking; a nonempty queue at that limit fails
        without sending. Unrelated traffic is ignored only until the deadline.

        TimeoutError indicates deadline expiry; ValueError indicates a malformed
        target reply; RuntimeError indicates a target fault or lifecycle error.
        Transport errors propagate. Any request failure requires close/connect;
        invalid arguments and rejected concurrent calls do not poison a session.
        """
        if isinstance(timeout_s, bool) or not isinstance(timeout_s, (int, float)):
            raise TypeError("timeout_s must be a positive finite number")
        if not math.isfinite(timeout_s) or timeout_s <= 0:
            raise ValueError("timeout_s must be a positive finite number")
        self._acquire()
        try:
            handler = self.channel_handler
            if handler is None:
                raise RuntimeError("Not connected; call connect first")
            if self._failed:
                raise RuntimeError("Failed session; close and reconnect before reading")
            try:
                deadline = time.monotonic() + timeout_s
                for _ in range(self._MAX_DRAIN_FRAMES):
                    self._remaining(deadline)
                    queued = handler.recv(timeout=0.0)
                    self._remaining(deadline)
                    if queued is None:
                        break
                else:
                    raise RuntimeError("Receive queue did not drain within 256 frames")

                parameter = ParameterType.MECHANICAL_POSITION[0]
                request = can.Message(
                    arbitration_id=(CommunicationType.READ_PARAMETER << 24)
                    | (self.host_id << 8) | self._motor_id,
                    is_extended_id=True,
                    data=struct.pack("<HHL", parameter, 0, 0),
                    check=True,
                )
                handler.send(request, timeout=self._remaining(deadline))
                self._remaining(deadline)
                while True:
                    frame = handler.recv(timeout=self._remaining(deadline))
                    self._remaining(deadline)
                    if frame is None:
                        raise TimeoutError("No mechanical position reply")
                    identifier = frame.arbitration_id
                    source = (identifier >> 8) & 0xFF
                    destination = identifier & 0xFF
                    kind = (identifier >> 24) & 0x1F
                    if source != self._motor_id or destination != self.host_id:
                        continue
                    if kind not in (CommunicationType.READ_PARAMETER,
                                    CommunicationType.FAULT_REPORT):
                        continue
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
                    register, reserved = struct.unpack("<HH", frame.data[:4])
                    if register != parameter:
                        continue
                    if (identifier >> 16) & 0xFF or reserved != 0:
                        raise ValueError("Malformed target parameter reply header")
                    value, = struct.unpack("<f", frame.data[4:])
                    if not math.isfinite(value):
                        raise ValueError("Nonfinite mechanical position reply")
                    return value
            except BaseException:
                self._failed = True
                raise
        finally:
            self._lock.release()
