# Explicit CSP SDK API

Import explicitly; the original package exports and bus API are unchanged:

```python
from robstride_dynamics.csp import CspMotor

CspMotor(channel, motor_id, bitrate=1000000, host_id=255)
motor.connect()  # Transport only; no handshake or motor commands.
motor.close()    # Transport only; no implicit disable or destructor commands.
motor.read_settings(*, timeout_s=0.1)  # Inherited read-only preflight.
motor.read_position(*, timeout_s=0.1)  # Inherited raw 0x7019 read.
motor.prepare(
    *, velocity_limit_rad_s, current_limit_a, torque_limit_nm,
    position_tolerance_rad, timeout_s=0.1,
    expected_position_rad=None, position_min_rad=None, position_max_rad=None
) -> float
motor.send_position(position_rad: float, *, timeout_s=0.1) -> float
motor.disable(*, timeout_s=0.1) -> None
motor.can_timeout_raw  # int | None; last preparation readback, not timing measurement.
```

The signatures above describe the API, not an executable example. All limits and
tolerance must be explicitly supplied, positive and finite. Preparation first
sends disable (without clearing faults), requires a fault-free Type-2 Reset
status, reads the actual settings, and refuses increases relative to those
settings, including float32 encoding. It requires `zero_state=1`, positive
`CAN_TIMEOUT`, and positions within the `-pi..pi` branch. It does not set zero,
change IDs, write the watchdog, reset faults, or save to flash.

While disabled it writes mode 5, speed/current/torque limits and the current raw
position as the hold target. Each write requires a fault-free Reset status.
It rereads all seven settings plus `loc_ref`, checks mode, limits within float32
relative tolerance (`2**-23`), unchanged watchdog and positions within the
supplied tolerance. Only then does it enable, require Motor status, and read
`0x7019` to check post-enable movement. Both position-returning methods return
actual raw register radians, not Type-2 quantized position or calibrated joint
coordinates. `send_position` requires successful preparation, writes `loc_ref`,
requires fault-free Type-2 Motor status, and reads correlated `0x7019` feedback.

### Optional runtime position guard

Supply `expected_position_rad`, `position_min_rad` and `position_max_rad`
together, or omit all three to preserve direct SDK behavior. Validation occurs
before commands: values must be finite, bounds must satisfy
`-pi <= position_min_rad < position_max_rad <= pi`, and expected position must
lie in `[position_min_rad + tolerance, position_max_rad - tolerance]`.

Immediately after the disable/settings snapshot, after the disabled settings
readback (before enable), and after enable, the raw position must satisfy both
`abs(position - expected_position_rad) <= position_tolerance_rad` and the same
clearance-shrunken bounds. The original runtime expected reference is never
rebased to the SDK snapshot. Pre-enable violations poison the session and send
no enable; post-enable violations poison it and require explicit owner cleanup.
These arguments guard preparation snapshots, not subsequent position commands;
runtime remains responsible for its command bounds and scheduling.

## Deadlines, watchdog and ownership

The local **RS02User Manual260713**, section 4.1.13, explicitly documents
`canTimeout=20000` as 1 second. Preparation and position sends require
`timeout_s < can_timeout_raw / 20000` using this **nominal** mapping. Zero or
unsupported settings are refused. This is not installed-firmware identification,
an empirical watchdog measurement, or a guarantee that the watchdog stops the
motor. Runtime must validate its own scheduling and firmware assumptions;
`read_settings()` remains available before preparation for read-only preflight.

Each transaction bounds drain, send and receive using a single monotonic
deadline, not one deadline per transport call. Preparation and position sends
comprise multiple transactions: their total duration can exceed `timeout_s`.
Successful preparation uses 23 transactions; a position send uses two.
Parameter reads reuse `PositionReader`'s bounded queue drain and correlation.
Both parameter and command transactions run the shared target-fault check on
every drained and received frame, before ignoring unrelated message types.
An addressed Type-21 report (even with zero values) or Type-2 fault flag in bits
16–21 fails the transaction; malformed addressed Type-2/21 classical frames
also fail. Fault-free valid Type-2 status is ignored during parameter reads.
Wrong motor/host and unknown message types remain unrelated traffic. Target
faults are never silently discarded as stale queue contents.
Status transactions limit drain and receive to 256 frames as well as the deadline.
Backends must honor timeout arguments; connect and shutdown have no hard bound.
An exclusive transport owner is required; operations reject concurrent calls
rather than waiting and hold the lock throughout preparation/position sends.

Any transaction/verification failure or interruption poisons the session and
revokes preparation. Subsequent position sends are refused until close/connect
and a new successful preparation. Invalid arguments and rejected concurrent
calls send no commands and do not poison the session.

## Mandatory owner cleanup

The owner must attempt disable after any operational failure, **including an
interrupted or failed preparation**, because enable may already have reached
the motor. Always close even if disable fails:

```python
# motor is the explicitly configured CspMotor owned by this runtime.
motor.connect()
try:
    # Explicit preparation and scheduling belong here.
    ...
finally:
    try:
        motor.disable()
    finally:
        motor.close()
```

Disable bypasses session poison and sends Type-4 immediately without pre-draining
or clearing faults. Only a valid, fault-free Type-2 **Reset (0)** status addressed
from the configured motor to the configured host confirms it. Valid, fault-free
**Motor (2)** statuses may already be in flight; disable skips them and continues
waiting for Reset. Motor status alone is never a successful acknowledgment.
Other target modes, target faults (including any Type-21 report), malformed target
status/fault frames and transport errors fail confirmation. Wrong motor/host IDs
and unrelated message types cannot confirm disable. A queued target fault is
checked only after sending disable and still fails confirmation, even if a Reset
is queued behind it. Writes and enable retain their strict expected-mode checks.

The caller-selected `disable(timeout_s=...)` (default **0.1 s**) is a single
monotonic budget covering **send plus all receives**, not a fresh timeout for
each Motor status or a fixed 20 ms wait. At most 256 received frames are examined;
a flood may exhaust that cap before the deadline. Missing Reset raises
`TimeoutError`; frame-budget exhaustion raises `RuntimeError`. Backends must
honor timeouts for the wall-clock bound to hold. Deadline and no-reply errors
identify disable Reset confirmation, other CSP command status, or the specific
register read (e.g. `Register read 0x7019`), rather than labeling all timeouts as
mechanical-position requests.

Owners should supply an independent cleanup budget, not reuse a short streaming
transaction timeout. Disable does not gate on `CAN_TIMEOUT`, wait for watchdog
expiry, or assume watchdog success. Successful disable revokes preparation but
does not unpoison a failed session. Cleanup is scoped only to this motor/transport;
the SDK does not modify runtime code or automatically issue cleanup commands.

## Limits of verification

The protocol has no transaction ID or status echo of the written register.
Pre-draining reduces stale replies but cannot distinguish a delayed identical
status or parameter reply from a fresh response. Disable deliberately does not
pre-drain, so even its matching Reset status cannot prove the requested stop
occurred. A Type-2 mode is operational status, not proof of CSP mode; preparation
also reads the actual mode register. Limits/settings reads are not atomic, and
concurrent external controllers can invalidate them. A post-enable position
check can only detect movement after the fact; it cannot prevent all motion or
establish mechanical safety. No physical, watchdog, freshness, installed-firmware,
or fail-safe proof is claimed. Use independently validated hardware safety and
firmware-specific commissioning before real operation.

Tests use only fake CAN transports and a deterministic clock; no sockets or
hardware are opened.
