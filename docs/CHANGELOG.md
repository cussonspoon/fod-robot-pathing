# Changelog

What landed, when, and why. Decisions that a future reader would otherwise have
to reverse-engineer from the code go here; decisions that are already explained
in a docstring do not.

Anything in here marked **[for Teemy]** or **[for Bthcorn]** is a change to a
shared contract and needs telling, not just recording.

---

## 0.2.1 — 2026-09-30 — a frozen camera no longer reads as a live one

**Safety fix.** Found by the integration harness
([JuniorSE15/fod-robot](https://github.com/JuniorSE15/fod-robot),
`docs/findings-upstream.md` #1 and #2), confirmed here.

His `detail()` returns the last *completed* frame on every call.
`LibraryVisionSource.poll()` offered whatever it returned as a new frame, on
every 50 Hz tick. So when his capture thread stalled -- process alive, no new
frames -- nav still saw a frame every tick, the `loop.vision_timeout_ms`
heartbeat could never lapse, and `TargetSet` re-stamped the frozen box as
fresh. In the harness the robot chased a frozen CONFIRM box through the real
screw. It also logged every poll, so `detections.jsonl` held each frame ~1.7
times.

### Changed

- **`LibraryVisionSource` offers a frame only when `frame_id` is new**, and
  nothing before his first frame (`age` is `inf`). Error frames always go
  through. A stall now stops the robot 0.4 s after the last real frame, the
  same as a camera that disappears.
- **One log line per frame**, not per poll.
- **`LibraryVisionSource.attach()`** reads from any object shaped like an
  entered `Vision`; `start()` is still the only path that imports `fodcv`.
- **`sim.scene.SimVision`** models his library as a *pull*: `detail()` keeps
  returning the last frame, with `age` growing. `SimHarness(library_vision=True)`
  drives the Pi's `LibraryVisionSource` through it.

### Packaging: nav is now a released wheel

Published on GitHub Releases as `fod_robot_pathing-0.2.1-py3-none-any.whl`, so
the integration harness can install a version instead of cloning the repo.
Three things had to change for an installed copy to work at all (harness
findings #3 and #4):

- **`config/` ships inside the wheel** as `fodnav/_config`, and
  `find_config_dir()` falls back to it last. Config-file arguments that do not
  exist relative to the working directory are looked up by name in the config
  directory, so `fodnav-sim` works from `~`. `fodnav-run --ground-calib`
  defaults to `ground_homography.json` in the config directory, not
  `./config/`.
- **Dependency floors are the Pi's apt versions**: `numpy>=1.24`,
  `opencv-python-headless>=4.6`. The full suite passes against the installed
  wheel on numpy 1.24.4 and opencv-headless 4.6.0.66.
- **`.python-version` is 3.11**, matching `requires-python` and the Pi.

The package version was still `0.1.0` through 0.2.0; it now reads `0.2.1`.

### Why the existing test missed it

`test_vision_dying_mid_approach_stops_the_robot_short` passed because the sim
fed vision through `QueueVisionSource`, which *pushes* frames and simply goes
quiet when they stop. The real library is polled and never goes quiet. The
mission tests that depend on frame arrival now run through both sources.

---

## 0.2.0 — 2026-09-10 — the vision interface was never going to be MQTT

`fod-vision v0.3.0` was released on 6 September: **a Python library you import,
not a service you subscribe to.** No topic, no socket, no daemon. The MQTT
schema this repo was built against — requested in CLAUDE.md §8, stubbed by
`tools/fake_detections.py`, parsed by `link/detections.py`, and covered by 36
tests — described an interface that never existed and was never going to.

His guide and contract are vendored at
`docs/vendor/fod-vision-v0.3.0-INTEGRATION.md`.

### What that cost, and what it did not

It cost the parser, its tests, the fake publisher and the sim's message shape.
It did **not** cost the projection, the odometry, either controller, the
planner, the FSM, the loop, the protocol codec or the chassis simulator.

That is the rule in CLAUDE.md §0 paying for itself: every field name from the
other side lived in one module, so a wrong guess about the whole interface was
a one-module rewrite rather than a repo-wide one. **Keep doing that.**

### Changed

- **`link/detections.py` deleted; `link/vision.py` added.** His contract:
  `Target(id, state, action, cls, conf, box, centroid)`, `VisionFrame`, and a
  `LibraryVisionSource` that imports `fodcv` **lazily inside `start()`** — the
  same quarantine `esp32.py` uses for pyserial, and what keeps `import fodnav`
  working on a laptop.
- **His `detail()` dict is the log format, verbatim.** Nav defines no wire
  schema at all now. One shape, he owns it, and there is nothing left for the
  two sides to disagree about.
- **`target.py` lost roughly two thirds of itself.** He associates on an 80 px
  radius, applies EMA confidence hysteresis (CONFIRM at 0.5, latched to 0.25)
  and hands back stable track ids. Nav did all three. Two hysteresis loops in
  series each lag the other, so ours went; what remains is projection,
  staleness and ageing his measurement forward on odometry between his 30 Hz
  and our 50 Hz.
- **Filtering is on `state` and `action`, never `cls`.** There is no `nail`
  class and never was; his are `bolt`/`nut`/`screw`/`washer`, they flip between
  frames on one object, and they disappear entirely when the single-class arena
  dataset lands. A test asserts a rename to `metal_fastener` changes nothing.
- **`Target.ground_px` is `((x0+x1)/2, y1)`** — the bottom edge of `box`, not
  his `centroid`. Same trap as CLAUDE.md §3, now one tempting field away.
- **Camera switched to his real optics**: 1280×720, 66° lens.
- **Python relaxed to `>=3.11`** and `paho-mqtt` dropped. The vision library
  must load apt's 3.11 `picamera2` and `hailo_platform`, so nav runs in a 3.11
  venv with `--system-site-packages`, in the same process. Nothing here ever
  needed 3.12 — every file already parsed as 3.11.
- **`tools/fake_detections.py` → `tools/fake_vision_log.py`**, writing his
  `detail()` shape as JSONL for `fodnav-replay`.

### The finding that matters

**His 66° lens more than doubles the terminal blind leg.** Our fiction assumed a
102° wide lens; Camera Module 3's standard lens is 66°, and a narrower lens sees
less floor close in:

| mount | near limit | blind leg |
|---|---|---|
| 0.22 m at 18° (the old fiction) | 0.381 m | **0.441 m** |
| 0.18 m at 25° (now in `sim_robot.yaml`) | 0.230 m | 0.290 m |

Holding the blind leg where it was needs a **lower, more steeply tilted mount** —
25° is the edge of the 10–25° band `RESULT.md` allows. That makes PRD **O-3**
the most contended number in the project: it stops his `lookahead` being a
placeholder and stops our ground calibration being possible, and neither side
moves without it.

Side effect: the detection swath narrowed from ~3× the drum width to ~1.8×, so
the §9 "which width is coverage" gap is smaller than the fiction implied.

### Two bugs the work surfaced

- **Replay silently dropped 57% of every log.** The source queue is bounded at
  64 — correct for a live camera, where the oldest frames are the ones worth
  losing — and replay inherited it. A 150-frame log read back as 64. Replay is
  now unbounded, with a test that would have caught it.
- **The JSONL log's write guard was too narrow.** It caught `OSError` and
  `TypeError`; a closed handle raises `ValueError`. A broken log must never stop
  a robot, so the guard is now deliberately broad.

### Still true

`config/robot.yaml` is still almost entirely `null`, and **nothing here has run
against his library or on a Pi.** This is built against a published, measured
spec rather than an invented one — a real improvement — but the first time his
code and ours meet is still ahead. The cheapest way to close that gap is thirty
seconds of recorded `detail()` output from his board, which `fodnav-replay`
reads directly.

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
