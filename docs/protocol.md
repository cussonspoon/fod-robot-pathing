# Pi ↔ ESP32 serial protocol — FOD Robot

**Version 1. Both sides implement this document. Neither side changes it
unilaterally.**

This file is the contract between two repos: the Pi-side navigation stack
(`fod-robot-pathing`, Spoon) and the ESP32 motor firmware (Teemy). Drop a copy in
each. If it is edited, bump `PROTO_VERSION` and tell the other side the same day.

`PROTO_VERSION = 1`

---

## 1. Link

USB CDC (the ESP32's native USB serial), **115200 baud, 8N1, no flow control.**

115200 is ample: peak load is ~50 command lines and ~50 telemetry lines per
second at well under 80 bytes each, roughly 8 kB/s against a 11.5 kB/s budget.
It is chosen over a faster rate because it is the setting that works everywhere
without tuning, and over the GPIO UART because USB CDC does not fight the Linux
serial console.

The Pi opens the port by stable path (`/dev/serial/by-id/...`), never
`/dev/ttyACM0`, which renumbers on replug.

---

## 2. Framing

**Newline-terminated ASCII lines.** `\n` (0x0A) terminates. `\r` is accepted and
ignored on receipt, never sent. Fields are separated by single spaces. Lines are
at most **120 bytes** including the terminator; a longer line is a protocol
error and is dropped, not truncated.

ASCII rather than a binary/COBS frame because it is debuggable with `screen` or
`cat` at 3 a.m. two days before an exam, and the bandwidth is not close to
binding. This is a deliberate trade.

Numbers: decimal, `.` as the decimal point, at most 3 decimal places, optional
leading `-`. No exponent notation. No locale-dependent formatting.

**Robustness rules, both directions:**

- An unrecognised first token → ignore the line, do not error, do not reset.
- A malformed field in a recognised command → ignore the whole line and emit a
  log line. **Never act on a partially parsed command.**
- Garbage before the first `\n` after connect is expected (boot messages, line
  noise). Both sides discard until the first clean line.
- Neither side may block waiting on the other. Both sides read non-blocking and
  buffer.

---

## 3. Pi → ESP32 (commands)

| Line | Meaning |
|---|---|
| `V <v> <omega>` | Velocity command. `v` in m/s, forward positive. `omega` in rad/s, counter-clockwise positive. |
| `S` | Immediate stop. Zero both wheels and brake. Does not disable. |
| `E` | Enable motors. |
| `D` | Disable motors (coast, drivers to high-Z). |
| `M <0\|1>` | Magnet drum motor off / on. |
| `?` | Request one immediate `T` telemetry line. |
| `H` | Handshake. ESP32 replies with one `I` line. |

Examples:

```
V 0.250 -0.400
V 0.000 0.000
S
M 1
```

### Limits

The ESP32 **clamps** rather than rejects. Out-of-range values are saturated to
the configured limits and bit 5 of the status flags is set for that telemetry
cycle. Clamping, not rejecting, because a rejected command in a 50 Hz loop
becomes a watchdog trip and a sudden stop.

| Quantity | Limit | Source |
|---|---|---|
| `v` | ±`V_MAX` | `config/robot.yaml` |
| `omega` | ±`OMEGA_MAX` | `config/robot.yaml` |

`V` sent while motors are disabled is accepted, clamped and stored, but produces
no motion until `E`. This lets the Pi's control loop keep the watchdog fed
during a pause.

---

## 4. ESP32 → Pi (telemetry)

Sent unsolicited at **50 Hz**, and once immediately on `?`.

```
T <seq> <t_ms> <ticks_l> <ticks_r> <v_meas> <omega_meas> <flags>
```

| Field | Type | Meaning |
|---|---|---|
| `seq` | uint16, wraps | Increments once per telemetry line. Gaps mean dropped lines. |
| `t_ms` | uint32, wraps | ESP32 millisecond clock at sample time. |
| `ticks_l` | int32, wraps | Cumulative left encoder ticks, signed, forward positive. |
| `ticks_r` | int32, wraps | Cumulative right encoder ticks, signed, forward positive. |
| `v_meas` | float | ESP32's own estimate, m/s. Cross-check only. |
| `omega_meas` | float | ESP32's own estimate, rad/s. Cross-check only. |
| `flags` | hex, no `0x` | Status bitfield, below. |

Example:

```
T 4821 903442 -128374 -128991 0.248 -0.402 03
```

**Ticks are cumulative and wrap as signed 32-bit.** The Pi differences them and
must handle wraparound explicitly; do not send deltas, because a dropped line
would then lose distance permanently. With cumulative counters a dropped line
costs nothing.

**Odometry is computed on the Pi, from raw ticks.** `v_meas` and `omega_meas`
exist only so the Pi can detect disagreement (a sign flip, a swapped encoder, a
wrong tick constant) early and loudly. They are not the odometry source.

### Status flags

| Bit | Mask | Meaning |
|---|---|---|
| 0 | `01` | Motors enabled |
| 1 | `02` | Watchdog has fired since last `E` |
| 2 | `04` | Overcurrent / driver fault latched |
| 3 | `08` | Obstacle sensor blocked (ToF below threshold) |
| 4 | `10` | Encoder fault (no ticks while commanded to move) |
| 5 | `20` | Last command was clamped |
| 6 | `40` | Battery low |
| 7 | `80` | Firmware in fault state, motion refused |

---

## 5. Other ESP32 → Pi lines

```
I <proto_version> <fw_version> <ticks_per_rev> <wheel_radius_mm> <track_width_mm>
L <level> <message>
```

Example:

```
I 1 fw-0.3.1 1440.000 32.500 200.000
```

**The two lengths are in millimetres, and only here.** Section 2 allows three
decimal places; the assertion below compares to 1e-6 m. In metres a 32.5 mm
wheel radius encodes as `0.032`, and no firmware could ever pass its own
handshake. Three decimals of a millimetre is exactly 1e-6 m, so the units are
chosen to make the tolerance representable rather than the tolerance loosened
to fit the units — a handshake that tolerates 0.5 mm on a 32.5 mm radius
tolerates 1.5%, which is the entire error it exists to catch. Both sides
convert at the wire boundary and hold metres everywhere else.

`I` is the handshake reply to `H`. **The Pi asserts on connect that
`proto_version` matches its own and that the three physical constants match
`config/robot.yaml` to within 1e-6 m, and refuses to run if they do not.** These
constants are duplicated across the two repos out of necessity — the firmware
needs them for its wheel PID, the Pi needs them for odometry — and a silent
divergence produces odometry that is subtly, consistently wrong in a way that
looks like a controller tuning problem for days. The handshake makes it a
startup failure instead.

`L` is a passthrough log line. `level` is one of `D`, `I`, `W`, `E`. The message
runs to end of line and may contain spaces. The Pi writes it into its own log
with an `esp32:` prefix. Log lines are for humans; **never encode state in them**
that the Pi is expected to parse.

---

## 6. The watchdog

**If the ESP32 has not received a valid `V`, `S`, `E`, `D` or `?` line for
300 ms, it must zero both wheel velocities, brake, and set flag bit 1.**

This is the most important requirement in this document. Without it, a crashed,
hung or SIGKILLed Pi process leaves the last velocity command executing forever,
which is a robot accelerating unattended into an active machine shop. It is not
a nice-to-have and it is not deferrable to "after the demo works".

**The watchdog arms on the first command received, not at power-on.** Between
boot and the Pi's first `V`/`S`/`E`/`D`/`?` the firmware has never been spoken
to, the motors are disabled, and there is nothing to protect against; arming at
boot would leave bit 1 set from power-on and make it useless as a diagnostic.
Once the Pi has spoken once, it is expected to keep speaking, and 300 ms of
silence trips it.

Recovery from a watchdog trip requires an explicit `E` from the Pi. It does not
resume automatically when commands return, because the reason commands stopped
is unknown and resuming into a 0.5 m/s command after a two-second gap is exactly
the failure the watchdog exists to prevent.

**Pi side of the contract:** publish `V` at a fixed 50 Hz whenever the link is
open, including `V 0.000 0.000` when stopped and including when the value has
not changed. Never skip a send as an optimisation. Never let the control loop
block on file, network or broker I/O.

**Both sides must test this deliberately.** `kill -9` the Pi process with the
robot moving and confirm it stops within 300 ms. Do it on the real chassis, not
in simulation, and do it before the exam rather than discovering it during one.

---

## 7. Startup sequence

1. Pi opens the port, waits 2 s for the ESP32 to finish booting, flushes input.
2. Pi sends `H`. ESP32 replies `I`.
3. Pi verifies protocol version and physical constants. Mismatch → abort with a
   loud error naming the mismatched field. Do not continue with a warning.
4. Pi begins the 50 Hz `V 0.000 0.000` heartbeat.
5. Pi sends `E` when it is ready to move.

On shutdown, `S` then `D`, and again from an `atexit` / signal handler so that
an unclean exit still stops the wheels.

---

## 8. Deliberately not in version 1

Raising any of these requires bumping `PROTO_VERSION`.

- Runtime PID tuning over the link. Tune by reflash; a live-tunable gain that
  disagrees with the committed source is a debugging trap.
- Servo / actuator commands beyond the magnet drum.
- Binary framing or CRC. Add only if line corruption is *observed*, not
  anticipated — and if it is, the fix is a checksum field appended to existing
  lines, not a reframe.
- Structured obstacle data. Bit 3 is a single blocked/clear signal; if nav needs
  distances, that is version 2.
- Any command that makes the ESP32 plan or decide. **The ESP32 executes
  velocities and reports encoders. It does not navigate.** If a feature request
  would give the firmware an opinion about where the robot should go, it belongs
  on the Pi.