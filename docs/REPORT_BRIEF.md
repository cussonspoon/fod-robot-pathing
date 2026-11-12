# FOD Robot — briefing for report writing

Written for an assistant helping write reports, presentations and progress
updates about the FOD Robot. Read this first, then the files listed under
"Where to look". Last updated **2026-11-12**.

If this file and `docs/STATUS.md` disagree, STATUS.md is right.

---

## The project

A small robot that finds foreign-object debris (screws, nails, fasteners) on a
floor, drives to it and collects it with a magnetic drum. Year 4 team project.

| Person | Role | Repo |
|---|---|---|
| Teemy | Hardware, ESP32 motor firmware | `Teemy17/fod-robot-esp32` |
| Bthcorn | Vision: detector and the `fod-vision` library | `Bthcorn/fod-robot-cv-poc` |
| Spoon | Navigation: where to go and how to drive there | `cussonspoon/fod-robot-pathing` (this repo, package `fodnav`) |
| Spoon's friend | Integration harness that runs all three together | `JuniorSE15/fod-robot` |

Advisor: "aj". Between 5 and 12 November the team explored turning this into a
tennis-ball collecting robot. That is dropped; ignore any tennis material.

## How it fits together

1. **Vision** (Raspberry Pi 5 + Hailo-8 accelerator): camera at 30 FPS →
   detector → tracker → boxes in pixels, each with a stable track ID and a
   state (CONFIRM / CAUTION / IGNORE).
2. **Navigation** (this repo, same process on the Pi): projects the bottom edge
   of each box onto the floor in metres, decides where to go, and runs a 50 Hz
   control loop.
3. **Firmware** (ESP32): receives wheel commands over a serial link, runs
   wheel-speed PID, and stops the wheels if nav goes quiet. The serial protocol
   is nav's design (`docs/protocol.md`).

## What navigation has built

- Released as a Python wheel; latest **v0.2.3** (1 October 2026).
- ~1390 automated tests that run in about 4 seconds with no hardware.
- The whole stack runs on a laptop: the real control loop, state machine,
  protocol code and projection maths, with only the clock, the serial link and
  the camera simulated.
- **Two controllers**, because they solve different problems:
  - **Visual servo** to chase a detected object. It re-aims every frame, so it
    is immune to wheel-odometry drift.
  - **move_to / path following** for planned coverage waypoints, which have
    nothing to see, so it relies on odometry.
- **Terminal blind leg**: the camera loses sight of the object before the drum
  reaches it. Nav remembers where the object was and drives the last stretch on
  odometry.
- **Safety**: nav sends a wheel command every 20 ms even when stopped; the
  ESP32 stops the wheels if it hears nothing for 300 ms. Nav also sends Stop on
  any error, when vision goes silent, and on exit.

## The modes

Two settings in `config/nav.yaml` choose what the robot does.

### Mission (`mission.mode`): target | coverage | idle

**Target (the chase) — the default.** Wait for a screw, drive to it, collect
it, wait for the next ("throw again, robot follows"). Steps:

- **Search** — nothing in view. `mission.search` picks one of:
  - **hold** (default): stand still and watch. The demo throws the screw into
    view, so no search pattern is needed.
  - **scan**: turn slowly on the spot (25% of the measured top turn rate) to
    find a screw off to one side. Finds things beside the robot, not beyond
    camera range.
- **Servo** — a confirmed screw is in view; steer at it, slowing while turning.
- **Blind leg** — close up, drive the last stretch on odometry.
- **Arrived** — count it, back to Search.
- **Lost** — gone for more than 1 s, back to Search.

**Coverage (the sweep).** Drive a back-and-forth (boustrophedon) path over a
known rectangle, the 3 × 3 m arena. Row spacing comes from the drum width.
Detections are logged but not chased, on purpose: diverting a sweep to a screw
would settle the open chase-vs-coverage question by accident. Ends when the
path is done.

**Idle.** Stand still; for testing the link and camera.

### Safety stops — override every mode

Checked every tick before any mode logic; each stops the robot and logs why:

- the firmware reports a fault or refuses motion
- the firmware watchdog fired (needs an explicit re-enable)
- vision went silent past its timeout (an empty frame means "floor clear" and is
  fine; only missing frames count)
- the obstacle sensor is blocked, or the battery is low (if enabled)

Every state change is logged with a plain reason, such as "holding, no target in
view" or "coverage path complete".

The chase is the default and the planned demo, following the advisor's brief
("the robot should chase a screw; picking it up is optional"). Coverage stays
built because the advisor has not formally chosen between them.

## Results: real vs simulated

**Measured on real hardware or real data**

- Vision (Bthcorn): Hailo inference 17.8 ms; camera to boxes 33.4 ms end to end.
- Three real ~30 s recordings from Bthcorn's board (24 Sep), replayed through
  nav: 2130 frames, zero parsing problems, nav's view matched his on every
  frame (`cv_tests/2026-09-24/output/summary.md`). It found two vision issues:
  a whole-frame box that reaches CONFIRM at start-up, and one object often
  tracked as two, causing 52 of nav's 79 target switches.
- The integration harness ran `fodnav-run` on the Pi against the real camera
  and Hailo, with the motors simulated ("manual-cam"). The ground calibration
  for that bench mount was not done, so ranges were approximate.

**Simulation only — pending, to be verified on the real robot** (the sim uses
invented robot constants, so these are not robot results)

- 0.6–1.7 cm error at the drum after a chase (with the odometry error model on)
- 50/50 catches with targets inside the camera's range; 39/50 when scattered
  wider
- a 31 cm blind leg, and the coverage findings in `docs/SIM_FINDINGS.md`

## Hardware status

- Nothing has run on the real chassis yet.
- Teemy's firmware exists: branch `v1-encoders`, built against nav's protocol
  (24 Oct); config change on 10 Nov (full motor duty, right motor reversed).
- Robot constants (wheel size, track width, camera mount) are not measured.
  Until they are, the firmware reports zeros and nav refuses to start, by design.
- The harness pins nav v0.2.3 and firmware `86c7b32`, and has a real-robot
  quick start.

## What is blocking

1. Camera mount height and tilt (PRD O-3) are not measured. This blocks the
   ground calibration, the blind-leg distance and Bthcorn's lookahead setting.
2. Teemy measuring the robot constants (`docs/HARDWARE.md`).
3. A bench run of nav against Teemy's firmware, and his sign-off on the protocol.

Next milestone, on the real chassis: dry run → teleop direction check →
watchdog kill test (kill the program while driving; the robot must stop) → one
recorded run.

## Where to look

| Question | File |
|---|---|
| What is built, what is blocking, the schedule | `docs/STATUS.md` |
| Why each decision was made, release by release | `docs/CHANGELOG.md` |
| What the simulator found, with caveats | `docs/SIM_FINDINGS.md` |
| Real vision data replayed through nav | `cv_tests/2026-09-24/output/summary.md` |
| The robot measurements still needed | `docs/HARDWARE.md` |
| The Pi ↔ ESP32 protocol and watchdog | `docs/protocol.md` |
| The vision library's contract | `docs/vendor/fod-vision-v0.3.0-INTEGRATION.md` |
| Gains, modes, tolerances | `config/nav.yaml` |
| Design rules and architecture in full | `CLAUDE.md` |
| Install, patch notes, commands | `README.md` |

## Writing rules

- **A simulation-only number is pending work, not a result.** Tag it
  "[Pending: to be verified on the real robot]" and also list it under future
  work.
- **Never invent a number.** If something is not measured, say so.
- **Inference speed does not limit driving speed.** Hailo removed that limit;
  do not write otherwise. If adaptive speed control comes up, its new
  justification (magnet dwell time, motion blur) is unmeasured.
- **The chase-vs-coverage question is not formally settled.** Say "we demo the
  chase, per the advisor's brief", not that one approach won.
- **Out of scope:** SLAM or mapping unknown rooms, coverage replanning,
  dashboards, charging dock.
- **Style:** concrete examples, bullets, short and plain. Cut anything that
  sounds complicated rather than rephrasing it.

## Glossary

| Term | Meaning |
|---|---|
| FOD | Foreign object debris: loose screws, nails, fasteners on a floor |
| Odometry | Estimating position from wheel rotation; drifts over distance |
| Visual servo | Steering toward what the camera sees, re-aimed every frame |
| Blind leg | The last stretch driven on odometry after the camera loses the object |
| Ground projection / homography | Turning a pixel position into floor metres, using a one-off calibration |
| Watchdog | Firmware rule: no command for 300 ms means stop the wheels |
| CONFIRM | Vision's state for an object it is confident about; nav only chases these |
| Boustrophedon | Back-and-forth sweep pattern, like mowing a lawn |
| Harness | The friend's test rig that runs vision, nav and firmware together |
