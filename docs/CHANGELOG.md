# Changelog

What landed, when, and why. Decisions that a future reader would otherwise have
to reverse-engineer from the code go here; decisions that are already explained
in a docstring do not.

Anything in here marked **[for Teemy]** or **[for Bthcorn]** is a change to a
shared contract and needs telling, not just recording.

---

## 0.1.0 — 2026-08-22

First working version. The repo went from three documents to the whole of
CLAUDE.md §4, with schedule items 0–6 built and tested. Item 7 (the real
chassis) is untouched.

### Built

| | |
|---|---|
| Scaffold | `pyproject.toml` (uv layout, Python 3.12), five `fodnav-*` console scripts, `src/` package |
| Config | `config.py` schema + loader; `robot.yaml` extracted **verbatim** from `HARDWARE.md` §8; `nav.yaml`; `sim_robot.yaml` |
| Geometry | `frames.py`, `ground.py` (homography fit, projection, three refusal paths) |
| Odometry | `odom.py` — int32 tick wraparound, exact arc integration, firmware cross-check |
| Links | `link/esp32.py` (both directions of the codec), `link/detections.py` (parse, JSONL, replay) |
| Control | `control.py`, `servo.py` + terminal blind leg, `target.py` association |
| Planning | `planner/boustrophedon.py` (pure), `planner/coverage.py` (swept cells) |
| Execution | `fsm.py`, `runner.py` (the 50 Hz loop), `runlog.py` |
| Simulation | `sim/` — chassis, camera, scene, harness, and a **fake ESP32 that implements `docs/protocol.md`** |
| Tools | `tools/fake_detections.py` |
| Tests | ~1370, all pure logic, 3.6 s |

### Changes to `docs/protocol.md` **[for Teemy]**

Both were found by writing the codec against the document and hitting a
contradiction. **Neither bumps `PROTO_VERSION`**, because no implementation of
v1 exists yet — but both change what the firmware has to do.

1. **§5 — the `I` handshake carries millimetres, not metres.**
   `I <proto> <fw> <ticks_per_rev> <wheel_radius_mm> <track_width_mm>`.
   §2 permits three decimal places; §5 asks the Pi to assert the constants match
   `robot.yaml` to 1e-6 m. In metres a 32.5 mm wheel radius encodes as `0.032`,
   so no firmware could ever pass its own handshake. Three decimals of a
   millimetre is exactly 1e-6 m. The alternative — loosening the tolerance to
   what metres can express — would tolerate 0.5 mm on a 32.5 mm radius, i.e.
   1.5%, which is the entire error the handshake exists to catch.
2. **§6 — the watchdog arms on the first command received, not at power-on.**
   Read literally, at boot the ESP32 has never received a `V`/`S`/`E`/`D`/`?`,
   so it would trip immediately and leave flag bit 1 set from power-on, making
   it useless as a diagnostic. Between boot and the Pi's first command there is
   nothing to protect against and the motors are disabled anyway. Once the Pi
   has spoken once, 300 ms of silence trips it as specified.

Also corrected: the header named this repo `fod-robot-nav`; it is
`fod-robot-pathing`.

### Decisions worth recording

- **`nav.yaml` is a separate file from `robot.yaml`.** CLAUDE.md §3 makes
  `robot.yaml` the single source of truth for *physical constants* and
  `HARDWARE.md` §8 prints it verbatim as the file Teemy edits. Putting gains in
  it would break both: his git history would stop being a calibration record,
  and the file would stop matching the document. Nav's tuning lives in
  `nav.yaml`, which nav owns.
- **Speeds in `nav.yaml` are dimensionless fractions of the measured `v_max`.**
  Nav does not know how fast this chassis goes. `cruise_fraction: 0.60` is a
  design choice; `0.27 m/s` would have been an invented measurement.
- **`sim_robot.yaml` is schema-identical to `robot.yaml`.** Same loader, same
  validation, so `--robot-config config/sim_robot.yaml` is a one-flag swap that
  exercises the same code. Its camera FOV numbers are *derived* from the
  fictional optics in `sim/camera.py` rather than chosen separately, and a test
  pins them — a fictional robot whose declared field of view disagreed with the
  camera it publishes through would trigger the blind-leg handover at the wrong
  distance and teach us something false.
- **The simulated firmware speaks the real protocol.** It would have been
  quicker to call the chassis model directly from the loop. Going through the
  codec means the framing, the malformed-line rules, the tick wraparound, the
  status flags and the 300 ms watchdog are covered by unit tests that run in
  milliseconds, instead of by a robot on a bench two days before the exam. It
  doubles as a reference implementation for Teemy to read.
- **Three modules beyond §4's layout**: `target.py`, `runner.py`, `runlog.py`.
  Reasons are in CLAUDE.md §4 and in each docstring.

### Bugs found while building — all now in CLAUDE.md §13

- **Pure pursuit skipped rows on the sweep.** With 15 cm row spacing and a 30 cm
  lookahead, the circle-intersection formulation targets two rows ahead; first
  symptom was one diagonal across the arena and a "sweep complete". Replaced
  with arc-length progress and a sliding search window. Coverage went from 13%
  to 99.5%.
- **`omega_min` was applied as a floor on turn rate while driving.** It is a
  stiction figure measured spinning in place from rest. Every small heading
  correction was bang-banging between ±0.15 rad/s — a visible shimmy. Now lifted
  only when `v` is zero; median |ω| while driving fell from 0.150 to 0.019 rad/s
  with no change in accuracy.
- **The vision timeout could never fire in simulation**, because the age came
  from the detection source's wall clock while the sim runs on virtual time.
  The loop now measures the heartbeat on its own clock, which is also the
  correct thing on hardware.
- **Replay rebased timestamps against the receipt envelope**, not `t_capture`,
  so a replayed frame still looked years stale to anything ageing it against
  odometry.

### Tuning

- `servo.k_v` 0.8 → 0.4. At 0.8 the proportional term never dropped below the
  cruise cap before the blind-leg handover, so the robot arrived at the latch at
  full speed. It now starts decelerating around 0.6 m.
- Added `servo.blind_leg_min_v_frac` (0.30 of cruise). A pure proportional law
  converges exponentially; without a floor the last few centimetres took longer
  than the rest of the leg and tripped the timeout. The drum is 18 cm wide, so
  that precision was never worth the seconds.
- The blind leg now uses the *same* control law as the servoing phase with the
  range coming from odometry, rather than a separate constant — otherwise there
  is a speed discontinuity exactly at the handover.

### Not built, deliberately

- Speed control over `follow_path` — needs a mechanism that is not inference
  latency, and none is measured (CLAUDE.md §11).
- Diverting a coverage sweep to a detection — that hybridises the two paradigms
  and would answer §11 by accident.
- A real pinhole fallback for the ground projection. What exists is the exact
  plane-induced homography of a pinhole camera, which is the same map for floor
  points and is what the simulated camera uses. A true fallback needs a
  checkerboard `K` and a distortion model, and until the homography proves
  unstable that work is not worth doing (CLAUDE.md §7).
- Clicking markers in `fodnav-calib-ground`. The runtime is
  `opencv-python-headless`; correspondences go in as JSON or CSV.

### Still `null`, still correct

`config/robot.yaml` is entirely unmeasured. Every code path that needs one of
those numbers raises at startup naming the field and its `HARDWARE.md`
procedure. That is the intended behaviour and it is tested.
