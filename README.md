# fod-robot-pathing

Navigation, path planning and motion control for the FOD robot. This repo is the
**robot application**: it consumes detections, decides where to go, and drives.

It is not the vision repo — vision lives in [`Bthcorn/fod-robot-cv-poc`](https://github.com/Bthcorn/fod-robot-cv-poc)
and motor firmware lives in Teemy's ESP32 repo. This repo owns everything
between the two.

Python package: `fodnav`. Console scripts: `fodnav-*`.

## Where things are written down

| | |
|---|---|
| Design rules, conventions, ownership, scope | [`CLAUDE.md`](CLAUDE.md) |
| Pi ↔ ESP32 serial contract | [`docs/protocol.md`](docs/protocol.md) |
| Every number Teemy measures, and how | [`docs/HARDWARE.md`](docs/HARDWARE.md) |
| What landed, when, and why | [`docs/CHANGELOG.md`](docs/CHANGELOG.md) |
| What the simulator says, with caveats | [`docs/SIM_FINDINGS.md`](docs/SIM_FINDINGS.md) |

Read `docs/protocol.md` before touching `src/fodnav/link/`.

## Status — v0.1.0, 2026-08-22

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
  camera_hailo.py                fodnav-run                   ESP32
  (system py3.11)                (venv py3.12)                (firmware)
        |                             |                            |
        |  MQTT fod/detections        |   UART, docs/protocol.md   |
        |  JSON, ~30 Hz          -->  |  V/S/E/D cmds @50 Hz  -->  |
        |                             |  <-- T telemetry @50 Hz    |
```

Vision and navigation are **separate OS processes and cannot be merged**: the
camera stack is an apt package built against the Pi's system Python 3.11, and
this venv is 3.12. Do not add `picamera2` or `hailo_platform` here.

## Setup

```bash
uv sync
```

Python 3.12. Runtime dependencies are `numpy`, `pyserial`, `paho-mqtt`,
`pyyaml`, `opencv-python-headless` — and that list is a budget, not a starting
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

To publish the §8 detection schema on a real broker instead — for testing
against another process, or for recording a log:

```bash
uv run python tools/fake_detections.py --scenario static --x 0.8 --y 0.1
```

It also takes `--scenario moving|empty|dropout`, `--clutter` to add `unknown`
boxes that nav must ignore, and `--jsonl PATH` to record a replayable log.

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

| | |
|---|---|
| `fodnav-sim` | the whole stack against simulated hardware |
| `fodnav-run` | the real thing: MQTT vision, serial to the ESP32, 50 Hz |
| `fodnav-teleop` | drive by hand, with the heartbeat maintained |
| `fodnav-calib-ground` | solve the pixel-to-floor homography |
| `fodnav-replay` | feed a recorded detection log back through the stack |

`fodnav-run --dry-run` loads the config, opens the link, runs the handshake and
stops — everything a real run checks except whether the robot drives well.

## Tests

```bash
uv run pytest
```

Pure logic, no hardware, no serial port, no broker: about 1400 tests in under
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
