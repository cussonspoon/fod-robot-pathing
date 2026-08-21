# CLAUDE.md — fod-robot-pathing

Navigation, path planning and motion control for the FOD Robot. This repo is the
**robot application**: it consumes detections, decides where to go, and drives.

It is not the vision repo. Vision lives in
[`Bthcorn/fod-robot-cv-poc`](https://github.com/Bthcorn/fod-robot-cv-poc) and is
owned by Bthcorn. Motor firmware lives in Teemy's ESP32 repo. This repo owns
everything between the two.

The Python package is `fodnav` (short import name); the repo is
`fod-robot-pathing`. Console scripts are prefixed `fodnav-`.

Read `docs/protocol.md` before touching anything in `src/fodnav/link/`.

---

## 0. Current state — read first

**The repo is scaffolded and §4 exists.** Schedule items 0–6 are built and
tested (§15); item 7, integration on the real chassis, is not. Do not
re-scaffold. `docs/CHANGELOG.md` records what landed and when.

The whole stack runs on a laptop with no camera, no Pi and no robot:

```bash
uv run fodnav-sim --set mission.mode=target --target 1.4 0.35   # chase a nail
uv run fodnav-sim --duration 400                                # sweep the arena
uv run pytest                                                   # ~1400 tests, <4 s
```

`fodnav-sim` runs the *real* control loop, FSM, protocol codec and projection
maths. Three things are simulated, because only three have no laptop
equivalent: the clock, the serial transport, and the camera. A simulated
firmware (`sim/firmware.py`) implements `docs/protocol.md` including the
watchdog, so the codec, the framing, the tick wraparound and the 300 ms timing
are covered by ordinary unit tests rather than by a robot on a bench.

**`config/robot.yaml` is still entirely `null`.** That is correct and it is the
point: it is Teemy's file, the loader raises on any `null` a code path actually
needs, and the error names the field and its `docs/HARDWARE.md` procedure. For
a laptop run, `config/sim_robot.yaml` describes a robot that does not exist —
every number in it is invented, which is legitimate only because nothing in it
claims to be a measurement. **Never copy a value out of it into `robot.yaml`.**

`docs/SIM_FINDINGS.md` has what the simulator says so far, with the caveats
that matter. Two of its results bear directly on §11.

**Dependency baseline. Do not exceed it without asking.** Two CPU cores and a
Raspberry Pi.

```
runtime: numpy, pyserial, paho-mqtt, pyyaml, opencv-python-headless
dev:     pytest, matplotlib (sim plots only, never imported by runtime code)
```

`opencv-python-headless`, not `opencv-python` — the only OpenCV call in the
runtime is `findHomography`, and the GUI build drags in X11 for nothing. No
scipy: `numpy` covers every linear-algebra need here. No robotics framework, no
ROS, no async runtime. If something seems to need one, that is a signal the
design drifted, not that a dependency is missing.

**Two specs in this repo are proposals, not agreements.**
`docs/protocol.md` has not been reviewed by Teemy and the detection schema in §8
has not been implemented by Bthcorn. Build against both, but write the code so
a change to either touches one module (`link/esp32.py`, `link/detections.py`)
and nothing else. Do not scatter field names through the codebase.

`docs/protocol.md` has been amended twice since it was drafted, both times
because writing the codec against it found a contradiction. Neither bumps
`PROTO_VERSION`, because nothing implements v1 yet — but Teemy needs to know
about both, and they are listed in `docs/CHANGELOG.md`.

---

## 1. Ownership boundaries

| Concern | Owner | Repo |
|---|---|---|
| Detector training, export, benchmarking | Bthcorn | `fod-robot-cv-poc` |
| Camera capture + inference process on the Pi | Bthcorn | `fod-robot-cv-poc` (`pi/camera_hailo.py`) |
| Pixel → floor-metre projection | **this repo** | `src/fodnav/ground.py` |
| Odometry, control, planning, FSM | **this repo** | `src/fodnav/` |
| Serial protocol *definition* | **this repo** | `docs/protocol.md` |
| ESP32 firmware *implementation* | Teemy | separate repo |
| Wheel PID, PWM, encoder ISR | Teemy | separate repo |

The projection sits here, not in the CV repo, because it depends on the physical
camera mount — the CV repo must stay mount-agnostic so its benchmarks remain
portable. If you find yourself wanting to add extrinsics to the CV repo, stop.

---

## 2. Hard constraints (do not design around these; design *within* them)

**Two Python interpreters, and they cannot be merged.** `pi/camera_hailo.py` runs
on the Pi's **system** Python 3.11 because `python3-picamera2` is an apt package
built against 3.11; the project venv is 3.12, a different C ABI, and no amount of
pip will fix it. Consequence: **vision and navigation are separate OS processes.**
Do not propose a single-process design. Do not add `picamera2` or
`hailo_platform` to this repo's dependencies.

**Two CPU cores, not four.** The CV repo measured the two-core case deliberately
as the budget for nav + SLAM: Hailo INT8 holds 17.8 ms whether given four cores
or two, so vision genuinely does not need them. Nav gets cores 2–3. Keep the
control loop inside that. If a design needs more than two cores, it is the wrong
design.

**Perception latency is not a constraint.** End-to-end capture→boxes is 33.4 ms,
camera-bound, ~3× headroom on the accelerator. At the 0.3 m lookahead that
permits ~9 m/s, which is far beyond anything this chassis will drive. Do not
write code, comments, or documents that treat inference latency as the thing
capping sweep speed. That was true of the NCNN/OpenVINO era and is no longer true.

**The watchdog is not optional.** See §6.

---

## 3. Conventions

Fix these before writing anything else. Every bug that costs a day in this class
of project is a frame or a unit bug.

**Units.** Metres, radians, seconds, everywhere, without exception. No degrees in
code — convert at the display boundary only. No centimetres. No milliseconds
except in the wire protocol, where they are explicitly named `_ms`.

**Angles.** Wrapped to `(-π, π]`. There is exactly one wrap function,
`fodnav.frames.wrap_angle`. Never write `atan2` results into state without
wrapping.

**Frames.** Right-handed, z up, yaw positive counter-clockwise from +x
(REP-103 convention).

- `world` — fixed. Origin at the arena's designated corner, +x along the long
  axis, +y left. All planned waypoints are in `world`.
- `odom` — the robot's integrated pose. Continuous and smooth but drifting.
  Starts coincident with `world`.
- `base` — robot body. Origin at the **midpoint of the drive-wheel axle,
  projected onto the floor**. +x forward, +y left, +z up. Not the chassis
  centre, not the camera.
- `cam` — camera optical frame, standard optical convention: +z out of the lens,
  +x right, +y down. Related to `base` by a fixed mount transform.

**Image coordinates.** Pixels, top-left origin, +x right, +y down, in the
**original captured frame size**, never the letterboxed 480×480 network input.
Detection messages carry `frame_size` so this is checkable — assert it on receipt.

**The ground point of a detection is the bottom-centre of its bounding box**, not
the centroid. The bottom edge is where the object contacts the floor, and the
floor is the plane being intersected. Using the centroid introduces a range error
proportional to object height and it will look like a calibration problem.

**Configuration.** One file, `config/robot.yaml`, is the single source of truth
for physical constants (wheel radius, track width, ticks/rev, camera mount height
and tilt, drum width and offset). Never hardcode a physical constant in a module.

Gains, tolerances, rates and mode switches are *not* physical constants and live
in `config/nav.yaml`, which nav owns. Keeping them out of `robot.yaml` is what
lets that file stay a calibration record whose git history means something, and
keeps it matching the block printed in `docs/HARDWARE.md` §8 that Teemy fills
in. Where a controller needs a speed, `nav.yaml` states a *fraction* of the
measured `v_max` rather than a number in m/s — nav does not know how fast this
chassis goes and must not pretend to.

`config/sim_robot.yaml` is schema-identical to `robot.yaml` and entirely
invented (§0). Same loader, same validation, so the simulated path exercises
exactly the code the real one does.

**Teemy owns the *values* in `config/robot.yaml`.** He holds the hardware, he
measures, he edits and commits that file directly — see `docs/HARDWARE.md` for
the procedures. Nobody re-types a measured number into a second place. Its git
history is the calibration record.

`config/robot.yaml` is the **only** file he edits in this repo. Nav owns the
schema, the loader and the validation; if a new key is needed, he asks for it.
His firmware also duplicates three of these constants as compile-time values, and
the `I` handshake in `docs/protocol.md` asserts the two sides agree.

---

## 4. Architecture

Three processes on the Pi.

```
  camera_hailo.py                fodnav-run                   ESP32
  (system py3.11)                (venv py3.12)                (firmware)
        |                             |                            |
        |  MQTT fod/detections        |   UART, docs/protocol.md   |
        |  JSON, ~30 Hz          -->  |  V/S/E/D cmds @50 Hz  -->  |
        |                             |  <-- T telemetry @50 Hz    |
```

MQTT because Mosquitto is already in the stack and loopback publish is
sub-millisecond. A UNIX domain socket is an acceptable swap if jitter ever
matters; the subscriber is behind an interface in `link/detections.py` precisely
so that swap is local.

Every detection message received is appended to a JSONL log. `fodnav-replay`
feeds a log back through the same interface, so the whole nav stack is testable
against real vision output with no camera, no Pi, and no robot.

### Package layout

```
src/fodnav/
  frames.py          frame conventions, transforms, wrap_angle. Pure, no deps.
  ground.py          pixel bbox -> (x, y) metres in base frame
  odom.py            differential-drive odometry from encoder ticks
  control.py         go_to_pose, pure pursuit, unicycle -> (v, omega) -> wheels
  servo.py           bearing-based visual servo controller
  target.py          association + confidence hysteresis -> one thing to chase
  planner/
    boustrophedon.py rect + swath -> waypoint list. PURE FUNCTION.
    coverage.py      swept-cell bookkeeping
  link/
    esp32.py         protocol codec + serial transport + watchdog feed
    detections.py    MQTT subscriber, JSONL logger, replay source
  sim/
    unicycle.py      kinematic sim with realistic odometry error
    camera.py        a fictional lens; floor metres -> pixels
    scene.py         objects in world -> detection messages
    firmware.py      a fake ESP32 that implements docs/protocol.md
    harness.py       assembles the whole stack against simulated hardware
  fsm.py             mode + state machine
  runner.py          the 50 Hz control loop, real and simulated
  runlog.py          the per-run log directory (§12)
  config.py          config loading and validation
  cli/               one module per console script
tests/
config/robot.yaml       measured constants, Teemy's, all null until he measures
config/nav.yaml         gains, tolerances, modes. Nav's.
config/sim_robot.yaml   a robot that does not exist
docs/protocol.md
```

Three of those are not in the original plan and each earns its place:

- **`target.py`** — association is needed by both modes, so it belongs to
  neither `servo.py` nor the FSM.
- **`runner.py`** — CLI modules are argparse-only, so the control loop had to
  live in the package. It is also what lets the loop be tested: swap the clock
  and the transport and a five-minute mission runs in a fraction of a second,
  watchdog timing included.
- **`runlog.py`** — §12 requires a run directory; it is not the loop's job.

Mirror the CV repo's CLI convention: one module per command, argparse only, then
a call into the package module that does the work. Console scripts declared in
`pyproject.toml` under `[project.scripts]`. Use `uv`.

Commands: `fodnav-sim`, `fodnav-teleop`, `fodnav-calib-ground`, `fodnav-replay`,
`fodnav-run`.

---

## 5. The two controllers, and why there are two

`move_to(x, y)` in `world` and visual servoing are **not** alternative
implementations of the same thing. Both are needed, they are each about forty
lines, and using the wrong one for the wrong job is the main way this demo fails.

**Visual servo — for driving to a detected object.** Project the detection to
`base`, take `bearing = atan2(y, x)` and `range = hypot(x, y)`, drive
`omega = k_theta * bearing` and `v = k_v * (range - stop_range)` with `v` scaled
down as `|bearing|` grows so the robot does not sprint while turning. Re-detect
every frame at 30 Hz. **This is immune to odometry drift entirely**, which is why
it is the right controller for the advisor's throw-a-nail demo: a 4 m
dead-reckoned drive on differential wheels over shop concrete accumulates heading
error, and heading error is exactly what makes the robot arrive next to the nail
instead of on it. "Throw again, robot follows" requires continuous re-targeting
anyway.

**`move_to(x, y)` — for planned waypoints.** Coverage waypoints are positions in
`world` with no visual feature to servo on, so this one integrates odometry and
accepts the drift. Turn-to-face, drive with heading P-correction, arrive within
`goal_radius`. `follow_path()` runs pure pursuit over a polyline for the long
straight legs of a boustrophedon row.

### The terminal blind leg — read this before debugging "it stops short"

The camera looks ~0.3 m ahead of the robot; the drum is under or behind the
axle. **The target leaves the camera's field of view before the drum reaches
it.** The servo controller therefore cannot run all the way to contact.

Handle it explicitly: when `range` drops below `fov_near_limit` (the closest
distance the camera can still see the floor, a measured value in `robot.yaml`),
latch the last good `base`-frame estimate and switch to an open-loop odometry
drive of the remaining distance plus the drum offset. It is a short leg, so
drift over it is negligible. What is not negligible is discovering this on demo
day.

---

## 6. Safety

**The ESP32 zeroes velocity if it has not received a `V` command in 300 ms.**
This is the single most important requirement in the whole system, it lives in
Teemy's firmware, and it is specified in `docs/protocol.md`. A Pi process that
crashes mid-drive without it produces a robot that keeps accelerating into a
machine shop. Test it deliberately: SIGKILL `fodnav-run` while the robot is
moving and confirm it stops.

Nav's side of the contract: the control loop publishes `V` at a fixed 50 Hz even
when the commanded velocity is zero. Never let the loop stall on I/O. Never
"skip" a command because nothing changed.

Additionally: `S` on any exception in the control loop, on loss of the detection
heartbeat beyond `vision_timeout_ms`, and in an `atexit` handler.

---

## 7. Calibrating the ground projection

Two implementations. Build the first; document the second as the fallback.

**Primary — planar homography.** Tape at least six markers on the floor at known
positions in `base`, capture one frame, click or auto-detect their pixel
positions, solve for the 3×3 homography `H` mapping image pixels to floor metres
(`cv2.findHomography`). No camera intrinsics needed and it absorbs lens
distortion approximately over the calibrated region. One afternoon of work.
`fodnav-calib-ground` runs this and writes `config/ground_homography.json`.

Note what it does *not* do: there is no window to click in, because the runtime
dependency is `opencv-python-headless` and dragging in X11 for one calibration
would be a poor trade. Read the pixel coordinates in any image viewer and pass
them in as a small JSON or CSV. The correspondences are the calibration record
and they are stored in the output file; the clicking is not.

It also prints the fit residual in millimetres, which is the number that says
whether the afternoon produced a calibration or produced a warning. Four points
give an exact fit with no residual to look at, which is indistinguishable from
a perfect one — that is why §7 says six.

**Fallback — pinhole plus extrinsics.** Calibrate `K` with a checkerboard,
back-project the pixel ray, intersect with `z = 0` in `base`. Use this only if
the homography is unstable, or if you need to reason about points outside the
calibrated patch.

Both are valid **only for the mount geometry they were calibrated at.** The
calibration file must record camera height, tilt, and capture resolution, and
loading it must assert those match `robot.yaml`. Any change to the mount
invalidates it. Extrapolating past the calibrated region degrades fast — reject
detections whose projected range exceeds `max_valid_range` rather than trusting
a number the calibration cannot support.

Guard the horizon: a ray that does not point downward has no floor intersection.
Reject rather than returning a huge or negative range.

---

## 8. Detection message schema

**This topic does not exist yet.** `camera_hailo.py` currently renders a preview
and publishes nothing. What follows is the schema being *requested* of Bthcorn.
Until he implements it, the only source of detections is the stub below — treat
every consumer as if the real publisher could differ in detail, and keep all
parsing inside `link/detections.py`.

`tools/fake_detections.py` publishes this schema at 30 Hz with a configurable
scenario: a static target at a given floor position, a target that moves, an
empty scene, and a dropout that stops publishing so the vision-timeout path can
be exercised. It exists, and it makes the entire nav stack runnable on a laptop
with no camera, no Pi and no robot. It deliberately shares no message-building
code with `link/detections.py` — that module parses, this one pretends to be
somebody else's process — so only the topic name and schema version are
imported, and those are the two things that must not drift.

`sim/scene.py` does the same job inside the simulator, without a broker, from
objects placed in `world`.

Published on MQTT topic `fod/detections`, QoS 0, retain false. Schema version 1:

```json
{
  "schema": 1,
  "t_capture": 1756000000.123,
  "t_publish": 1756000000.156,
  "frame_id": 4821,
  "frame_size": [2304, 1296],
  "dets": [
    {"cls": "bolt", "conf": 0.87, "bbox": [1102, 812, 61, 44]}
  ]
}
```

`bbox` is `[x, y, w, h]` in original-frame pixels, top-left origin. `t_capture`
is the sensor timestamp, not the publish time — nav needs it to age the estimate
against odometry.

Nav-side rules:

- **Treat `nail`, `screw` and `bolt` as one target class.** The CV repo's own
  results record that screws are reliably found and reliably mislabelled as
  `bolt`, and PRD FR-3 specifies a single `metal_fastener` class anyway.
  Localisation is solved; naming is not. Do not build nav logic that depends on
  which of the three came back.
- **Ignore `unknown`.** It is 53% of the training data, a grab-bag of four
  shapes, and the class that fires on furniture. The CV repo suppresses it by
  default; nav must not resurrect it.
- Assert `frame_size` matches the calibrated resolution. If it does not, the
  homography is invalid and every projection is silently wrong.
- An empty `dets` list is a valid message and is **not** the same as no message.
  No message for `vision_timeout_ms` means vision is dead → stop.

Association across frames: nearest-neighbour in `base` with a gate of
`assoc_max_jump_m`, plus confidence hysteresis. The CV repo has a tracker with
hysteresis whose approach is worth reading before writing this one.

---

## 9. Boustrophedon planner

Pure function. No I/O, no state, no robot. This is the easiest thing in the repo
to test exhaustively and there is no excuse for it not being.

```python
def plan(rect, swath_w, overlap, start_corner, orientation) -> list[tuple[float, float]]
```

Row spacing is `swath_w * (1 - overlap)`. Run rows parallel to the **long** axis
to minimise turns. Emit only row endpoints as waypoints; the controller handles
the in-place turns between them.

`swath_w` is a config parameter with two defensible values and they are not the
same number: the **drum width** if coverage means collection, the **camera
ground-footprint width at the lookahead** if coverage means detection. Which one
is correct depends on the unresolved evaluation question (§11). Make it a
parameter, default to the drum width, and write down the choice in the run log.

Property tests: every point in the rectangle is within `spacing/2` of some path
segment; no waypoint outside the rectangle; waypoint count matches the closed
form; reversing `start_corner` mirrors the path.

**Do not build SLAM-driven coverage over an unknown map for the exam.** That
needs BreezySLAM bring-up, occupancy-grid tuning and cell decomposition, and the
PRD's own timeline has no slot for it. Coverage over a *known* rectangle — the
3×3 m seeded grid — is a function over four corners and a swath width. The
advisor has already granted the coverage-optimisation deferral to Project 2;
this takes the mapping half of the same deferral.

---

## 10. The simulator

Not a luxury. It is how the planner and both controllers get written and tested
before the chassis exists, and how they stay tested afterwards.

Unicycle integration, plus the error sources that actually matter:

- encoder tick quantisation
- **per-wheel scale mismatch** (e.g. left wheel 1.5% larger effective radius) —
  this produces systematic curvature and is the dominant real-world odometry
  error on differential drive, far more than gaussian noise
- gaussian tick noise
- a small constant heading bias
- optional wheel slip events

Without the scale mismatch the sim will tell you `move_to` over 4 m works fine
and the hardware will disagree. Model it.

---

## 11. Open questions — do not silently resolve these in code

**The evaluation paradigm is unresolved between the team and the advisor.** PRD
v5 says vision sets sweep *speed* over a boustrophedon coverage path. The advisor
on 5 Aug described vision *steering* the robot to a thrown nail, and separately
approved coverage in the same meeting. No ruling yet.

Two simulation results are worth having in hand for that conversation, both in
`docs/SIM_FINDINGS.md` and both caveated there. First, a full sweep of the 3×3 m
arena is **46.6 m of driving** (on the drum-capture swath), ten times a single
approach, and holding
coverage above 90% needs the two wheels matched to about **0.2%** — which is
tighter than a roll test alone delivers and is the whole reason for the spin
test in `HARDWARE.md` §2.2. Second, the two readings of "coverage" give sweeps
that differ by a factor of three (§9). Neither result decides the question.
Both change what a coverage demo would have to promise.

This does not block nav work, because `move_to()` is the executor for both:
target-chasing feeds it a vision-derived waypoint, coverage feeds it a planned
one. Build the controller either way. But **do not** write documentation, log
schemas, or evaluation harnesses that assume one paradigm won. Keep both modes in
`fsm.py` behind a config switch.

**The mechanism behind adaptive speed control needs rethinking.** The original
argument was that inference latency caps safe speed. Hailo has removed that
constraint (§2). If speed control survives as the thesis, it must rest on
something else — magnet dwell time falling with speed, or motion blur degrading
recall. Neither is measured.

Motion blur specifically: it depends on **exposure time**, which nobody has
measured. The 20.7 ms in the CV repo's table is capture-pipeline latency, not
exposure, and using it as exposure would overstate blur several-fold. Read
`ExposureTime` from picamera2 metadata under arena lighting before anyone puts a
blur number in a document.

**Camera down-tilt is unresolved (PRD O-3).** The CV repo's synthesis is 10–25°
at the 15–30 cm mount height. Tilt is an input to the ground calibration, so it
must be fixed and recorded before calibrating, and re-calibrated if it changes.

---

## 12. Working practice

Match the CV repo's standards; they are high and the project is better for it.

**Never invent a number.** If a constant is not measured, it goes in
`robot.yaml` as `null` with a comment naming the measurement procedure, and the
loader raises on it. A plausible placeholder that silently becomes load-bearing
is worse than a crash. The CV repo's phrasing for this is "no number beats a
flattering one" — hold to it.

**Tests are pure-logic and fast.** No hardware, no serial port, no broker,
seconds not minutes: transforms round-trip, homography against known points,
odometry integrates a known arc to closed form, boustrophedon coverage
properties, protocol codec round-trips including malformed and truncated lines,
tick wraparound arithmetic, watchdog timing.

**Every run writes a log directory**: the detection JSONL, the command/telemetry
stream, the resolved config, and the git SHA. A demo that worked once and cannot
be reproduced is worth nothing at the exam.

**Ask before adding a dependency.** Two cores and a Pi.

---

## 13. Gotchas

Each of these costs a session. Add to this list when you find a new one.

- **`/dev/ttyACM0` renumbers on replug.** Use `/dev/serial/by-id/...`. Put the
  stable path in `robot.yaml`.
- **If you use the GPIO UART instead of USB-CDC**, the Linux serial console
  holds it. Disable the getty and remove `console=serial0` from `cmdline.txt`,
  or the ESP32 receives boot spew as commands.
- **The Pi 5 defaults to the `ondemand` governor**, idling at 1.5–1.6 GHz
  against a 2.4 GHz max, and `cpupower` is not installed on this board. Control
  loop jitter measured under `ondemand` is fiction. Write the sysfs nodes
  directly; it resets on reboot.
- **A live desktop session silently taxes timing** and nothing obvious catches
  it. Check `ps -eo pcpu,comm --sort=-pcpu` before trusting a timing run.
- **`uv` is not on `PATH` in a non-interactive ssh.** Use `~/.local/bin/uv`.
- **Do not `import picamera2`, `libcamera` or `hailo_platform` here.** Wrong
  interpreter, and it will appear to work on a dev machine and fail on the Pi.
- **Textbook pure pursuit skips rows on a boustrophedon.** Row spacing is
  ~15 cm and a sensible lookahead is 30 cm, so the circle of radius `lookahead`
  crosses the next two rows as well, and "the furthest intersection" is two rows
  ahead. First symptom is one long diagonal across the arena followed by the
  robot declaring the sweep complete. `control.py` carries progress as an **arc
  length** with a sliding search window instead, which cannot skip forward onto
  a row it has not driven or snap back onto the one it just finished.
- **`omega_min` is a stiction figure, not a floor on turn rate.** HARDWARE.md
  §2.5 measures it spinning *in place from rest*. Once both wheels are turning,
  an arbitrarily small difference between them is achievable — there is no
  static friction left to break. Applying it as a floor while driving makes
  every small heading correction bang-bang between ±`omega_min`, which is a
  visible shimmy down the whole length of a row. `saturate()` lifts it only when
  `v` is zero.
- **Measure the vision heartbeat on the control loop's own clock.** The
  detection source stamps arrivals with `time.monotonic()`, which under
  simulation has no relationship to the run, so a vision-timeout test passes for
  the wrong reason and the dropout path is never actually exercised. It is also
  the right thing on hardware: it is the loop that has to notice.
- **Three decimal places on the wire cannot express a 1e-6 tolerance.** A
  32.5 mm wheel radius in metres encodes as `0.032`, so the `I` handshake could
  never pass. The two lengths travel as millimetres for exactly this reason; see
  `docs/protocol.md` §5. Whenever a wire format and an assertion disagree about
  precision, one of them is wrong — do not loosen the assertion to fit.

---

## 14. Out of scope for the exam build

Say no to these, in code review and in planning:

- SLAM and mapping of an unknown room (Project 2)
- Coverage replanning / not re-sweeping cleared zones (Project 2, deferred by
  the advisor in writing)
- Non-ferrous handling, MQTT→InfluxDB heatmap, facilities reporting
- The 2D/3D live dashboard
- Charging dock and return-to-charge
- Any refactor of the CV repo

---

<!-- ============================================================
     DELETE THIS SECTION AFTER THE EXAM. It is a schedule, not a
     contract, and it will be wrong before it is old.
     ============================================================ -->

## 15. Current schedule (exam ~2 Sep 2026 — delete after)

Ordered by dependency, not by importance.

0. ~~Scaffold per §0, and `tools/fake_detections.py`.~~ **Done.**
1. ~~`frames.py`, `config.py`, `robot.yaml`, and the sim.~~ **Done.**
2. ~~`ground.py` + `fodnav-calib-ground`.~~ **Done in software.** The
   calibration itself cannot be taken until the mount is frozen and measured
   (`HARDWARE.md` §3), and it is void the moment the mount moves.
3. `docs/protocol.md` finalised **with Teemy** — *still open, and still the long
   pole*. The codec is written against it and the simulated firmware implements
   it, but that is one side agreeing with itself. He has not reviewed the
   document, and it has been amended twice since drafting (§0). **Hand him the
   doc.**
4. ~~`control.py` + `move_to` + `fodnav-teleop`, validated in sim.~~ **Done.**
5. ~~`servo.py` + the terminal blind leg.~~ **Done in sim**, 0.6–1.7 cm at the
   drum with the error model on. Unproven on hardware, and the number it depends
   on most — `camera.fov_near_limit_m` — has not been measured yet.
6. ~~`planner/boustrophedon.py`.~~ **Done**, with the property tests §9 asks for.
7. Integration on the real chassis, watchdog kill-test, one recorded run.
   **Not started. Everything below is blocked on it.**

Item 3 was always the long-pole item because it depends on someone else, and it
still is. Nothing in this repo can un-block it.

What is actually blocking now, in order:

- **Teemy's measurements landing in `config/robot.yaml`.** Until then nothing
  runs against real hardware — by design, loudly, with the procedure named.
  `HARDWARE.md` §0 lists the three that block the most work.
- **Teemy's review of `docs/protocol.md`**, including the two amendments.
- **The camera mount frozen and measured**, which gates the ground calibration,
  which gates every vision-driven behaviour on the real robot.
- Then item 7: `fodnav-run --dry-run`, `fodnav-teleop` to check the §2.4 sign
  conventions, the watchdog kill-test on the real chassis, one recorded run.

Deliberately not built, and not to be built without a decision first:

- **Speed control over `follow_path`.** If the adaptive-speed thesis survives
  §11 it needs a mechanism that is not inference latency, and none is measured.
  Building it now would be building it against the argument Hailo already
  demolished.
- **Diverting a coverage sweep to a detection.** That is a hybrid of the two
  paradigms and inventing it here would answer §11 by accident.