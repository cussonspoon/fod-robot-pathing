# HARDWARE.md — numbers nav needs from the robot

**Teemy measures, fills this in, and edits `config/robot.yaml` directly.** He
holds the hardware, so he writes the number — nobody re-types a measurement into
a second place, because that is how 0.0847 becomes 0.0874 and nobody notices for
a week. The git history of `robot.yaml` is the calibration record.

`config/robot.yaml` is the **only** file to edit in this repo. Everything else is
under active development; if something else looks wrong, say so rather than
fixing it.

Every value here is something the navigation code cannot guess. Where a number
is missing, `config.py` raises on startup rather than substituting a plausible
default — a wrong constant does not crash, it just makes the robot subtly and
consistently wrong in a way that looks like a tuning problem for two days.

**Measure, don't read off a datasheet.** Every item below says how. Datasheet
values for wheel radius and gear ratio are routinely 2–5% off once a tyre is
loaded and a gearbox has backlash, and 3% on wheel radius is 12 cm of error
across a 4 m drive.

Fill in the `measured` column, sign and date §7, and send it back.

---

## 0. Do these three first

They block the most work and take about twenty minutes between them.

1. **§2.4 encoder signs** — five seconds, and getting it wrong makes every other
   measurement in this document garbage.
2. **§3 camera mount, then freeze it** — nothing vision-driven can be calibrated
   until the mount stops moving.
3. **§6.1 serial by-id path** — Spoon cannot open the link without it.

---

## 1. Identity

| Item | Value |
|---|---|
| Chassis build / revision | |
| Date these measurements were taken | |
| Measured by | |
| Floor surface used (must be the arena floor, not tile or carpet) | |
| Battery state during measurement (full / half — affects wheel loading) | |

---

## 2. Drive geometry

This block is the entire basis of odometry. Everything the robot believes about
where it is comes from these four numbers.

### 2.1 Effective wheel radius `wheel_radius_m`

**How:** Mark a chalk line on one drive wheel and on the floor. Push the robot
in a straight line, with the battery and all payload fitted, until the wheel has
made exactly **10 full revolutions**. Measure the floor distance travelled.
`r = distance / (2π × 10)`.

**Not with calipers.** A loaded tyre's effective rolling radius is smaller than
its free radius, and that difference is the error you are trying to avoid.

| | |
|---|---|
| Distance over 10 revolutions (m) | |
| **`wheel_radius_m` (m, 4 decimals)** | |

**If wrong:** every distance is scaled. The robot overshoots or undershoots
every waypoint by a constant percentage, and coverage rows drift apart.

### 2.2 Track width `track_width_m`

**How, part 1:** tape measure between the two drive wheels' **contact patches**
(the middle of where each tyre touches the floor), not the hub faces, not the
outer edges.

**How, part 2 — this one matters more:** after §2.1 and §2.3 are known and the
firmware runs, command the robot to spin in place for exactly **10 full
rotations**, and compare what odometry reports to the actual 3600°.

```
track_width_true = track_width_measured × (degrees_reported / 3600)
```

| | |
|---|---|
| Tape-measured (m) | |
| Odometry-reported rotation over 10 physical turns (deg) | |
| **`track_width_m` (m, 4 decimals, corrected)** | |

**If wrong:** every heading is scaled. Straight lines curve, turns are short or
long, and the error compounds — this is the single largest source of
dead-reckoning failure on a differential drive. Do the spin test.

### 2.3 Encoder ticks per wheel revolution `ticks_per_rev`

Counts at the **wheel**, after the gearbox, including the quadrature multiplier
(×4 if both channels on both edges).

**How:** read the firmware's raw tick counter, rotate one wheel by hand through
exactly 10 revolutions, read again, divide the difference by 10. Do not compute
it from `CPR × gear_ratio` alone — verify it. Gearbox ratios are often quoted
rounded (e.g. "1:30" for 29.86:1).

| | |
|---|---|
| Ticks over 10 hand revolutions | |
| **`ticks_per_rev`** | |
| Quadrature mode used (×1 / ×2 / ×4) | |

### 2.4 Encoder and motor sign convention

**How:** command the robot forward at low speed. Read the `T` telemetry line.

| Check | Expected | Confirmed? |
|---|---|---|
| Driving **forward** → both `ticks_l` and `ticks_r` increase | yes | |
| Driving **forward** → neither counter decreases | yes | |
| Spinning **counter-clockwise** (viewed from above) → `ticks_r` increases faster than `ticks_l` | yes | |
| `ticks_l` is the **left** wheel viewed from behind, facing forward | yes | |

**If wrong:** a swapped or inverted encoder produces odometry that is confidently
mirrored. The robot drives away from every goal. It looks like a broken
controller and it is not. Fix the sign in firmware, not on the Pi.

### 2.5 Velocity limits and deadband

**How:** command increasing `v` with the robot on the floor and watch
`v_meas` track the command. Then command *decreasing* `v` until the robot stops
moving despite a nonzero command.

| | Value | Notes |
|---|---|---|
| `v_max_mps` — highest `v` the wheels actually achieve | | measured, not spec |
| `omega_max_radps` — highest in-place spin rate | | |
| `v_min_mps` — lowest `v` that produces reliable motion | | PWM/friction deadband |
| `omega_min_radps` — lowest reliable spin rate | | |

**If `v_min` is missing:** the controller creeps toward a waypoint at a velocity
too small to overcome static friction, the robot sits still, and the loop thinks
it is still approaching. Every position controller needs to know where its
authority runs out.

### 2.6 Stopping distance

**How:** drive at `v_max`, send `S`, measure how far it travels after the
command.

| | Value |
|---|---|
| Stopping distance from `v_max` (m) | |
| Stopping time from `v_max` (s) | |

**If wrong:** obstacle-avoidance margins and the watchdog's 300 ms window are
both sized against this. It is also the number that tells you whether the robot
is safe near people.

---

## 3. Camera mount — freeze this before measuring

Everything vision-driven is calibrated against this geometry. **Once measured,
the mount must not move.** If it is adjusted, loosened, knocked, or re-printed,
every number in §3 and the whole ground calibration is void and must be redone.

The `base` frame origin is the **midpoint of the drive-wheel axle, projected
down onto the floor.** +x forward, +y left, +z up. All offsets below are from
that point.

| Item | Value | How |
|---|---|---|
| `cam_height_m` — optical centre above floor | | tape from floor to lens centre, robot loaded |
| `cam_tilt_deg` — down-tilt from horizontal | | phone inclinometer laid on a flat face of the mount, or on the lens barrel |
| `cam_offset_x_m` — forward of axle midpoint | | positive forward |
| `cam_offset_y_m` — left of centreline | | should be 0; **measure anyway** |
| `cam_roll_deg` — rotation about the optical axis | | should be 0; a twisted mount shears every projection |
| `capture_width_px` × `capture_height_px` | | the resolution `camera_hailo.py` actually captures at, not the 480 network input |
| Mount frozen on (date) | | |

Target band for tilt, from the CV repo's synthesis: **10–25° down** at a
15–30 cm mount height. Steeper angles are what taller cleaning robots use and
they don't transfer.

### 3.1 Field of view on the floor

Two numbers, both measured by putting a screw on the floor and walking it toward
and away from the robot while watching the live preview.

| Item | Value | Why nav needs it |
|---|---|---|
| `fov_near_limit_m` — closest floor distance still in frame | | length of the terminal blind leg; see CLAUDE.md §5 |
| `fov_far_limit_m` — furthest distance the detector still fires reliably | | rejection gate for implausible projections |
| `fov_width_at_lookahead_m` — ground width of the frame at 0.3 m | | one of the two candidate swath widths |

**If `fov_near_limit` is large** (say 0.5 m rather than 0.15 m), the robot is
dead-reckoning a third of every approach and the servo controller needs
redesigning. Measure this before `servo.py` gets written.

---

## 4. Collection drum

| Item | Value | Why |
|---|---|---|
| `drum_width_m` — swept width | | boustrophedon row spacing |
| `drum_offset_x_m` — longitudinal position relative to axle midpoint, signed | | terminal blind leg overshoot distance |
| `drum_clearance_mm` — magnet face to floor | | pickup gate testing |
| `drum_capture_width_m` — actual width it picks up over, if narrower than the drum | | measured, not assumed |

`drum_capture_width_m` and `drum_width_m` are frequently not the same number.
The one that matters for coverage is the width over which a screw is actually
lifted, which is a magnetic-field question, not a mechanical one. Measure it:
lay screws in a line across the drum's width at 2 cm spacing, push the robot
over them once, see which ones came up.

---

## 5. Obstacle sensing and power

| Item | Value |
|---|---|
| ToF sensor model | |
| ToF mount height above floor (m) | |
| ToF forward offset from axle midpoint (m) | |
| ToF beam direction / cone half-angle | |
| Distance threshold that sets flag bit 3 (m) | |
| Battery nominal voltage (V) | |
| Low-voltage warning threshold, flag bit 6 (V) | |
| Cutoff voltage (V) | |
| Robot total mass, loaded (kg) | |
| Footprint length × width (m) | |

Footprint is used for planner margins — the boustrophedon rectangle gets inset
by half the footprint plus a clearance so the robot doesn't clip the arena edge
on its turns.

---

## 6. Link

| Item | Value |
|---|---|
| `/dev/serial/by-id/...` full path | |
| USB CDC or GPIO UART? | |
| Baud (should be 115200 per protocol.md) | |
| Firmware version string reported in `I` | |
| `PROTO_VERSION` implemented | |

Get the by-id path with `ls -l /dev/serial/by-id/` on the Pi with the ESP32
plugged in. Do not send `/dev/ttyACM0` — it renumbers on replug and the run will
fail at a different time than the mistake.

---

## 7. Sign-off

The three constants in §2.1, §2.2 and §2.3 are also compiled into the firmware
for its wheel PID, and the `I` handshake asserts that both sides agree to within
1e-6. **If you change any of them in firmware, tell Spoon the same day**, or the
Pi will refuse to start and it will not be obvious why.

| | |
|---|---|
| Measured by | |
| Date | |
| Camera mount frozen (yes/no) | |
| §2.4 sign checks all confirmed (yes/no) | |

---

## 8. The file to edit

This is `config/robot.yaml` as it ships. Edit it in place, commit it with a
message naming what was measured, and leave anything unmeasured as `null` —
that raises on startup, which is the intended behaviour. **Do not replace a
`null` with a guess to make the error go away.** A crash is a question; a wrong
number is a week.

Re-measure and re-commit whenever the hardware changes. The camera block in
particular is void the moment the mount moves.

```yaml
# Measured on: YYYY-MM-DD   by: ____   chassis rev: ____
# Camera mount frozen: YYYY-MM-DD. If the mount moves, §3 and the ground
# calibration are void.

drive:
  wheel_radius_m: null        # §2.1, 10-revolution roll test
  track_width_m: null         # §2.2, corrected by the 10-rotation spin test
  ticks_per_rev: null         # §2.3, verified by hand rotation
  quadrature: null            # 1 | 2 | 4
  v_max_mps: null             # §2.5
  v_min_mps: null             # §2.5, deadband
  omega_max_radps: null
  omega_min_radps: null
  stopping_distance_m: null   # §2.6, from v_max

camera:
  height_m: null              # §3
  tilt_deg: null              # target band 10-25
  offset_x_m: null
  offset_y_m: null
  roll_deg: null
  capture_width_px: null      # must match the calibration frame_size
  capture_height_px: null
  fov_near_limit_m: null      # §3.1 -- sizes the terminal blind leg
  fov_far_limit_m: null
  fov_width_at_lookahead_m: null

drum:
  width_m: null               # §4
  capture_width_m: null       # measured pickup width, not mechanical width
  offset_x_m: null
  clearance_mm: null

obstacle:
  tof_height_m: null
  tof_offset_x_m: null
  tof_threshold_m: null

chassis:
  mass_kg: null
  footprint_length_m: null
  footprint_width_m: null
  planner_margin_m: null      # half footprint + clearance

power:
  nominal_v: null
  warn_v: null
  cutoff_v: null

link:
  port: null                  # /dev/serial/by-id/... never /dev/ttyACM0
  baud: 115200
  proto_version: 1
```

---

## 9. Not needed yet

Don't spend time on these before the exam — nav has no consumer for them.

- IMU mounting and calibration (only if odometry drift proves unmanageable)
- Lidar position and extrinsics (SLAM is Project 2)
- Charging contact geometry
- Precise motor torque or current curves