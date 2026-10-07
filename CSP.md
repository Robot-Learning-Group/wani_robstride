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

## Additive group API

Import `CspGroup` explicitly; `csp.py`, `position.py`, and package exports remain
unchanged. This session owns **one** SocketCAN bus with extended-frame filters
for the selected motor IDs and host destination:

```python
from robstride_dynamics.csp_group import CspGroup

# API signatures (not an executable control sequence):
CspGroup(channel, motor_ids: dict[str, int], bitrate=1000000, host_id=255)
group.connect()  # No commands.
group.read_settings(*, timeout_s) -> dict[str, dict]
group.prepare(limits: dict[str, dict], *, timeout_s=.02, check_cancel=None) -> dict[str, float]
group.send_positions(targets: dict[str, float], *, timeout_s) -> dict[str, float]
group.disable(*, timeout_s) -> dict[str, str | None]
group.last_status_temperature_c  # dict[name, float]
group.last_status_monotonic_s    # dict[name, float]
group.close()  # No commands, including after an error.
```

Both input dictionaries must contain **exactly** the configured motor names.
Each preparation entry must contain these seven fields: `velocity_limit_rad_s`,
`current_limit_a`, `torque_limit_nm`, `position_tolerance_rad`,
`position_min_rad`, `position_max_rad`, and `expected_position_rad`. These are
six explicit SDK limit fields plus the runtime's expected reference. Runtime
owners must translate `CspTestLimits.max_tracking_error_rad` to the SDK's
`position_tolerance_rad`; the other five limit field names match. Passing
`max_tracking_error_rad` directly is not supported as an SDK alias. The SDK does
not import runtime code. All arguments are validated before any
commands. Raw and float32-encoded control targets must lie within `-pi..pi`.
The approved bounds/expected reference guard preparation snapshots; runtime
remains responsible for subsequent command bounds and scheduling.

Preparation phases are **group-wide**: disable all and confirm Reset; batch
read each of the seven settings registers; validate every motor; write mode to
all; write each of the three limit registers to all; write current raw hold
targets to all; batch-read settings and hold targets; verify all readbacks and
position guards; batch-enable all and confirm Motor status; batch-read and
verify raw positions against the initial and runtime expected references and
the verified hold-target readback.
No motor is enabled while another is still being configured. Reported settings
must have zero state 1, positive limits/watchdog, and a finite raw position on
the verified branch. Limit increases are refused for both requested and encoded
values; limit readbacks must exactly match their float32 encoding. Watchdogs
must remain unchanged. All position snapshots require tolerance clearance
inside the approved bounds. Before enable, the hold-target readback must also
be within tolerance of the latest disabled raw-position snapshot, not merely
within tolerance of the original initial reference. After enable, raw position
must also be directly within tolerance of the verified hold-target readback,
in addition to the initial-movement and runtime position guards. Checking both
values only against the initial reference would otherwise allow them to differ
by up to twice the tolerance.

Each preparation command/register batch has **one shared deadline**, including
drain, all sends and all receives, independent of motor count. Successful
preparation uses 23 such phases, so its total can approach `23 * timeout_s`;
it is not a single-deadline call. Read-only settings use seven shared register
deadlines and return the same seven keys as `CspMotor`. Group CSP settings
validation requires `zero_state=1` and `timeout_s < can_timeout_raw / 20000`.
The watchdog mapping is nominal, not empirical firmware verification.

A control cycle instead uses **one deadline for the entire call**: drain queued
traffic; write all targets **without receiving between sends**; collect a
fault-free Type-2 Motor status from every motor; drain buffered traffic again
under that same deadline; request all raw positions **without receiving between
sends**; collect finite register-correlated `0x7019` replies. Each batch also
performs a bounded, nonblocking queue inspection after its final required reply,
using the original deadline. This catches selected faults/malformed frames queued
behind an otherwise complete response; an undrainable tail fails closed. Out-of-order replies are accepted; duplicates cannot confirm
another motor. Faults or malformed selected status/fault frames fail even when
queued or received during another motor's register/status phase. Classical
frame and parameter-header validation matches `position.py`. Control/preparation
register and command drains/collections have a 4096-frame cap, sufficient for
12-motor batches but bounded even when a fake/backend clock does not advance.
Disable pre-drains and tail inspections use the smaller of 256 and the configured
frame budget; disable status collection uses the configured 4096-frame budget.

`check_cancel()` is called before every preparation send and receive, including
nonblocking drains. It should raise to abort. Cancellation, transport failures,
faults, verification failures and interruptions poison the session and revoke
preparation. Operations reject concurrent callers without interleaving commands.
Close/connect and prepare again before resuming control.

Accepted command statuses also update `last_status_temperature_c` and
`last_status_monotonic_s`. Temperature is decoded from the Type-2 `>HHHH`
payload's final unsigned 16-bit field using the original bus API's exact
`temperature_u16 * 0.1` °C scale. Only a selected, well-formed, fault-free
Type-2 status accepted as a still-pending required response to a normal write,
enable, or control-target batch updates these dictionaries. Queue drains,
unrelated or duplicate frames, fault frames, malformed frames, and explicit
or preparation cleanup disable collection do not update them. Thus cleanup
cannot overwrite the final enabled telemetry. Both dictionaries are cleared
on every successful `connect()`.

`last_cycle` records SDK `time.monotonic()` timings and control status telemetry:

- `target_send_monotonic_s`: motor-name dictionary recorded immediately after each
  transport `send()` returns successfully—i.e. after python-can accepts the frame,
  not before a potentially blocking send attempt. These are host-side submission
  completion times, not proof of motor receipt or synchronized execution.
- `target_burst_s`: elapsed target-write burst time (including a failed burst).
- `status_temperature_c`: a per-cycle name-to-temperature copy populated only by
  the control target statuses accepted in that cycle.
- `status_receive_monotonic_s`: matching host monotonic acceptance timestamps.
- `total_s`: entire control batch duration, also updated on failure.

`last_error` identifies a failing `phase` and ordered `pending_motors` list;
disable failures list all unconfirmed motors. `last_disable_outcomes` retains
the most recent explicit disable's name-to-outcome mapping (cleared on connect).
These are transport diagnostics, not hardware timing or freshness guarantees.

**Cleanup is explicit**, including after cancellation or partial enable. Use a
separate cleanup budget and always close, even if disable fails:

```python
group.connect()
try:
    positions = group.prepare(limits, check_cancel=check_cancel)
    # The owner schedules subsequent group.send_positions(...) calls.
finally:
    try:
        outcomes = group.disable(timeout_s=0.1)
        # None means fault-free Reset was observed; strings mean unconfirmed.
        # The owner must handle every unconfirmed motor.
    finally:
        group.close()
```

Disable bypasses poison and cancellation. It first establishes a **bounded,
nonblocking receive-buffer boundary**: drain with `recv(timeout=0)` before sending,
discarding known buffered Reset replies rather than accepting them as new stop
acknowledgments. This drain uses at most 256 frames (or a smaller configured
budget) and the same total deadline as the rest of disable; it never waits for
traffic. A drain fault is retained for its motor; a failed/undrainable boundary
makes all confirmations fail closed. **Even if this drain faults, raises an
interruption, exhausts its frame budget, or consumes the deadline, every disable
send is still attempted.** Preparation's initial disable uses this same boundary
and unconditional send-all behavior, and propagates failures before configuration.

No receive occurs **between** disable sends. Send failures do not suppress other
motors' attempts; expired-budget sends use zero/nonblocking timeout. One total
deadline covers the pre-drain, all sends, reply collection and final queue
inspection. In-flight Motor status is skipped while waiting for Reset. Faults,
malformed replies, send failures and missing replies produce per-motor error
strings; an error is never overwritten by a later Reset. The final bounded queue
inspection can override already confirmed motors if faults/malformed frames are
queued behind the last Reset. An incomplete final inspection fails closed.

Explicit cleanup catches **`BaseException`**, including `KeyboardInterrupt` and
`SystemExit`, from transport sends/receives and represents it as an error string
instead of propagating it and skipping another motor's cleanup. Interruptions are
therefore intentionally suppressed during `disable()`; callers must inspect its
returned mapping or `last_disable_outcomes` and surface cleanup failures. This
does not affect propagation of preparation/control interruptions. Confirmed
motors return `None`; disable does not unpoison a failed session.

The protocol has **no transaction ID**: source/register correlation and drains
cannot prove that a matching reply is fresh, nor that an observed Reset was
caused by this disable. In particular, an identical in-flight reply may arrive
after the receive-buffer boundary; the SDK cannot distinguish it from a new
reply. CAN frame timestamps are wall-clock values and are not compared with
SDK `time.monotonic()` deadlines. Snapshots are non-atomic. Use an exclusive bus owner and
independent hardware safety; no physical safety, installed-firmware, watchdog,
synchronization or freshness guarantee is claimed.
