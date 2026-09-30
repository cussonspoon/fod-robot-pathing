# fod-robot-pathing

Navigation, path planning and motion control for the FOD robot. This repo is the
**robot application**: it consumes detections, decides where to go, and drives.

It is not the vision repo — vision lives in [`Bthcorn/fod-robot-cv-poc`](https://github.com/Bthcorn/fod-robot-cv-poc)
and motor firmware lives in Teemy's ESP32 repo. This repo owns everything
between the two.

Python package: `fodnav`. Console scripts: `fodnav-*`.

## What the package does

It is the part of the robot that decides where to go and drives there.

1. **Reads the camera's detections** from Bthcorn's `fod-vision` library: boxes
   in pixels, each with a track id and `CONFIRM` / `CAUTION`.
2. **Turns a box into a spot on the floor**, in metres from the robot, using the
   bottom edge of the box and a ground calibration.
3. **Decides what to do**:
   - **target** mode chases the nearest confirmed fastener and drives the drum over it;
   - **coverage** mode sweeps a rectangle row by row.
4. **Drives the wheels** by sending `V <speed> <turn>` to the ESP32 over serial,
   50 times a second.
5. **Stops safely**:
   - when vision is lost;
   - on any error;
   - when the camera freezes (see v0.2.1 under What's new).

   The ESP32's watchdog stops the wheels if nav goes silent for 300 ms.
6. **Records every run**: detections, commands, config and git SHA, in a log folder.

It also runs **with no hardware at all**: a simulator stands in for the robot,
the camera and the ESP32, so everything above can be tried on a laptop.

| Command | What it does |
|---|---|
| `fodnav-sim` | The whole stack against a simulated robot, camera and ESP32 |
| `fodnav-replay` | Feed a recorded camera log back through nav: what would it have done? |
| `fodnav-run` | The real thing, on the Pi: camera → decisions → ESP32 |
| `fodnav-teleop` | Drive the robot by keyboard, to check wiring and directions |
| `fodnav-calib-ground` | Build the pixel → floor calibration from taped markers |

## Install

Latest: **[v0.2.2](https://github.com/cussonspoon/fod-robot-pathing/releases/tag/v0.2.2)**.
Each release is a wheel on
[GitHub Releases](https://github.com/cussonspoon/fod-robot-pathing/releases),
the same way `fod-vision` ships. No need to clone this repo to run it.

**Laptop** (simulator, replay, testing):

```bash
pip install https://github.com/cussonspoon/fod-robot-pathing/releases/download/v0.2.2/fod_robot_pathing-0.2.2-py3-none-any.whl
fodnav-sim --set mission.mode=target --target 1.4 0.35     # works from any directory
```

**Raspberry Pi**, inside the Python 3.11 venv created with `--system-site-packages`:

```bash
sudo apt install python3-serial python3-yaml
pip install --no-deps https://github.com/cussonspoon/fod-robot-pathing/releases/download/v0.2.2/fod_robot_pathing-0.2.2-py3-none-any.whl
```

`--no-deps` is required on the Pi:

- numpy and OpenCV come from apt with `python3-picamera2`;
- pip does not recognise apt's OpenCV;
- so without the flag it installs a second copy underneath the camera software.

**A real run** needs your own config folder and model file:

```bash
fodnav-run --config-dir /path/to/config --set detections.hef=/path/to/best.hef
```

The package carries a copy of `config/`, so the simulator runs anywhere. That
copy's `robot.yaml` has no measured values on purpose, so a real run refuses to
start until you point it at a folder holding a measured `robot.yaml`, `nav.yaml`
and `ground_homography.json`.

## What's new

**v0.2.2**

- **`stream.jsonl` is saved every tick** (every 20 ms), not every 0.5 s.
  - A live display reading nav's log is now at most one tick behind the
    serial line.
  - It is a setting: `log.stream_flush_every` in `nav.yaml`.
  - See [Reading nav's logs live](#reading-navs-logs-live-for-dashboards).

**v0.2.1**

- **Safety: a frozen camera now stops the robot.**
  - The vision library keeps returning its last picture when the camera
    freezes, and nav used to treat each read as a new picture, so it kept
    driving toward a frozen image.
  - Nav now accepts a picture only when its frame number changes, so a freeze
    stops the robot 0.4 s after the last real picture.
  - Found by the integration harness
    ([JuniorSE15/fod-robot](https://github.com/JuniorSE15/fod-robot)).
- **Installable package:**
  - config included;
  - runs from any folder;
  - version minimums match the Pi (numpy ≥ 1.24, opencv ≥ 4.6, Python 3.11).
- **First real camera recordings replayed** (`cv_tests/2026-09-24/`):
  - nav reads all 2130 frames from Bthcorn's board, identically to his library;
  - notes for him: one object often gets two tracks, and a whole-screen box can
    be chased at start-up.
- **Simulator:** its camera now behaves like the real library, so tests catch
  this kind of bug. 1378 tests.

Full history: [`docs/CHANGELOG.md`](docs/CHANGELOG.md).

## Reading nav's logs live (for dashboards)

If you build a display on top of a run, know how fresh each file is. Each one
is written on a different schedule:

| File | Written to disk | So its last line is |
|---|---|---|
| `stream.jsonl`: state, `v`, `omega`, `reason` (range and bearing), pose | every tick; `log.stream_flush_every` in `nav.yaml` (before v0.2.2: every 25 ticks) | up to **20 ms old** |
| `detections.jsonl`: vision's `detail()`, one per frame | every frame | up to ~33 ms old |
| The serial line itself (`V ...`, if you tap it) | as it is sent | live, ≤ 20 ms |

Every line of `stream.jsonl` is **one moment**: the `reason` (what nav saw) and
the `v`/`omega` (what it sent) belong together.

- **Take a decision and its command from the same line.**
- Don't pair a `stream.jsonl` line with a live `V` from the serial line.
  - Since v0.2.2 they are at most a tick apart, but it is still two moments.
  - It would drift again if the flush interval were raised or the disk slowed.
  - Before v0.2.2 they could be half a second apart, which is where a live
    panel showed `0.60 m` next to `V 0.199` although nothing was wrong.
- Show the line's own `t` next to it, so the delay is visible.

## Where things are written down

| | |
|---|---|
| Design rules, conventions, ownership, scope | [`CLAUDE.md`](CLAUDE.md) |
| What is built, what is blocking, the schedule | [`docs/STATUS.md`](docs/STATUS.md) |
| Pi ↔ ESP32 serial contract | [`docs/protocol.md`](docs/protocol.md) |
| Every number Teemy measures, and how | [`docs/HARDWARE.md`](docs/HARDWARE.md) |
| What landed, when, and why | [`docs/CHANGELOG.md`](docs/CHANGELOG.md) |
| The vision library's own contract (vendored) | [`docs/vendor/`](docs/vendor/) |
| What the simulator says, with caveats | [`docs/SIM_FINDINGS.md`](docs/SIM_FINDINGS.md) |

Read `docs/protocol.md` before touching `src/fodnav/link/`.

## Status — v0.2.2, 2026-09-30

Everything between the detector and the motors is built and tested in
simulation. Nothing has run on the real chassis.

| | |
|---|---|
| Package layout (CLAUDE.md §4) | complete |
| Schedule items 0–6 | done |
| Item 7 — real chassis, watchdog kill-test, recorded run | **not started** |
| Tests | ~1370, pure logic, under four seconds |
| `config/robot.yaml` | **entirely `null`** — nothing measured yet, and that is correct |

Blocked on, in order: Teemy's measurements landing in `config/robot.yaml`; his
review of `docs/protocol.md` (which has been **amended twice** — see the
changelog); the camera mount being frozen and measured, which gates the ground
calibration and therefore every vision-driven behaviour on hardware.

In simulation, with the odometry error model on: the visual servo puts the drum
**0.6–1.7 cm** from a thrown fastener, while a 4 m dead-reckoned `move_to`
misses by **51 cm** and reports success. Read
[`docs/SIM_FINDINGS.md`](docs/SIM_FINDINGS.md) — including its caveats — before
quoting either number.

## Three processes, two repos, one robot

```
  fodnav-run                       (Pi 5, system python3.11 venv)
  |
  |-- fod-vision  (Bthcorn's)      capture thread @30 FPS -> Hailo-8 -> tracker
  |        ^  vision.detail()      pixels, stable ids, CONFIRM / CAUTION
  |
  |-- ground.py                    pixels -> metres in base   [ours; he will not]
  |-- fsm + control @50 Hz
           |  UART, docs/protocol.md
           v
        ESP32                      wheel PID, PWM, watchdog   (Teemy's)
```

The vision side is a **library, not a service** — you import it and it runs a
thread in your process. It must load apt's `picamera2` and `hailo_platform`,
built against the Pi's system Python 3.11, so nav runs on 3.11 in a venv created
with `--system-site-packages`. Do not add `picamera2`, `hailo_platform` or
`fod-vision` to this repo's dependencies: `link/vision.py` imports the library
lazily so everything here stays runnable on a laptop.

## Setup, for working on this repo

To *run* nav, install the package (above). To *change* it:

```bash
uv sync
```

Python **3.11** on the Pi (the vision library needs apt's 3.11 camera stack;
create the venv with `--system-site-packages`). Runtime dependencies are
`numpy`, `pyserial`, `pyyaml`, `opencv-python-headless` — and that list is a budget, not a starting
point. Two CPU cores and a Raspberry Pi. Ask before adding to it.

## Running with no hardware

The entire stack runs on a laptop with no camera, no Pi and no robot. Chase a
fastener thrown onto the floor at (1.4, 0.35):

```bash
uv run fodnav-sim --set mission.mode=target --target 1.4 0.35 --duration 30
```

Sweep the arena instead:

```bash
uv run fodnav-sim --duration 400
```

Watch the robot stop when the camera process dies:

```bash
uv run fodnav-sim --set mission.mode=target --target 1.4 0.35 --vision-dies-at 3
```

Those run the real control loop, the real FSM, the real protocol codec and the
real projection maths. Only three things are simulated, because only three have
no laptop equivalent: the clock, the serial transport, and the camera.

To make a recorded-looking vision log without a camera — in exactly the format
the real library produces, and the real robot records:

```bash
uv run python tools/fake_vision_log.py --scenario approach -o logs/fake.jsonl
```

It also takes `--scenario static|empty|dropout`, where `dropout` emits the
error frame a dead camera produces — because "no debris, keep patrolling" is
the one thing a broken camera must never look like.

Feed a recorded detection log back through the perception stack:

```bash
uv run fodnav-replay logs/<run>/detections.jsonl
```

### The fictional robot

`config/sim_robot.yaml` describes **a robot that does not exist**. Every number
in it is invented, which is legitimate only because nothing in it claims to be
a measurement. The real `config/robot.yaml` ships full of `null`s and the
loader raises on any one a code path actually needs — that is the intended
behaviour, not a bug to work around:

```
config/robot.yaml is missing 3 value(s) that odometry needs:

  drive.wheel_radius_m
      effective rolling wheel radius, loaded [m]
      -> docs/HARDWARE.md §2.1 (10-revolution roll test)
  ...
```

## The configuration split

| File | Holds | Owner |
|---|---|---|
| `config/robot.yaml` | measured physical constants | **Teemy** — he holds the hardware, he measures, he commits. Its git history is the calibration record. |
| `config/nav.yaml` | gains, tolerances, rates, mode switches | nav |
| `config/sim_robot.yaml` | a fictional chassis | nav, for tests and laptop runs |

`config/robot.yaml` is the only file Teemy edits here, and its shape is the
block printed verbatim in `docs/HARDWARE.md` §8. Nav owns the schema, the
loader and the validation; a new key is a conversation, not a commit.

**No number is invented.** An unmeasured constant is a `null` with the
measurement procedure named next to it, and the loader raises on first use
naming the field and pointing at the procedure. A plausible placeholder that
silently becomes load-bearing is worse than a crash.

## Commands

The five commands are listed under [What the package does](#what-the-package-does).
Each takes `--help`.

`fodnav-run --dry-run` loads the config, opens the link, runs the handshake and
stops — everything a real run checks except whether the robot drives well.

## Tests

```bash
uv run pytest
```

Pure logic, no hardware, no serial port, no broker: about 1370 tests in under
four seconds. The simulated firmware means the protocol codec, the watchdog
timing and the tick-wraparound arithmetic are all covered by ordinary unit
tests rather than by a robot on a bench.

## Safety, briefly

The ESP32 zeroes velocity if it has not received a `V` command in 300 ms. Nav's
side of that contract is a `V` published at a fixed 50 Hz whenever the link is
open, including `V 0.000 0.000`, including when nothing has changed. The control
loop never blocks on I/O and never skips a send as an optimisation.

Test it deliberately: `kill -9` the run process with the robot moving, on the
real chassis, and confirm it stops.
