# Status

**The volatile half of CLAUDE.md lives here**, so that the design rules can stay
stable while this changes weekly. If this file and CLAUDE.md ever disagree about
what is built, this one is right — and CLAUDE.md needs fixing.

Last updated: **2026-08-28**, v0.1.0.

---

## Where the code is

Everything in CLAUDE.md §4 exists. Schedule items 0–6 are built and tested;
item 7, integration on the real chassis, has not started.

```bash
uv run pytest                                                   # ~1370 tests, <4 s
uv run fodnav-sim --set mission.mode=target --target 1.4 0.35   # chase a fastener
uv run fodnav-sim --duration 400                                # sweep the arena
```

`config/robot.yaml` is **34 of 36 values still null**. That is the intended
state, not an outstanding task for nav: it is Teemy's file and nothing in it has
been measured. Everything that needs one of those numbers fails at startup
naming the field and its `docs/HARDWARE.md` procedure.

Git: v0.1.0 is on `main`. The docs split that created this file merged after
it. Start new work from `main`.

---

## Schedule (exam ~2 Sep 2026 — delete this section after)

Ordered by dependency, not importance.

0. ~~Scaffold, and `tools/fake_detections.py`.~~ **Done.**
1. ~~`frames.py`, `config.py`, `robot.yaml`, and the sim.~~ **Done.**
2. ~~`ground.py` + `fodnav-calib-ground`.~~ **Done in software.** The
   calibration itself cannot be taken until the mount is frozen and measured
   (`HARDWARE.md` §3), and it is void the moment the mount moves.
3. `docs/protocol.md` finalised **with Teemy** — *still open, still the long
   pole*. The codec is written against it and the simulated firmware implements
   it, but that is one side agreeing with itself. He has not reviewed the
   document, and it has been amended twice since drafting.
4. ~~`control.py` + `move_to` + `fodnav-teleop`, validated in sim.~~ **Done.**
5. ~~`servo.py` + the terminal blind leg.~~ **Done in sim**, 0.6–1.7 cm at the
   drum with the error model on. Unproven on hardware, and the number it leans
   on hardest — `camera.fov_near_limit_m` — has not been measured.
6. ~~`planner/boustrophedon.py`.~~ **Done**, with the property tests §9 asks for.
7. Integration on the real chassis, watchdog kill-test, one recorded run.
   **Not started. Everything below is blocked on it.**

## What is actually blocking, in order

- **Teemy's measurements landing in `config/robot.yaml`.** `HARDWARE.md` §0
  lists the three that unblock the most, about twenty minutes between them.
- **Teemy's review of `docs/protocol.md`**, including the two amendments, which
  change what his firmware must do.
- **Whether Teemy's firmware exists at all** — nobody has confirmed this. Worth
  asking before assuming the integration is a one-afternoon job.
- **The camera mount frozen and measured**, which gates the ground calibration,
  which gates every vision-driven behaviour on the real robot.
- **Bthcorn publishing the §8 detection topic.** Until then
  `tools/fake_detections.py` is the only publisher in existence.

Then item 7: `fodnav-run --dry-run`, `fodnav-teleop` to check the §2.4 sign
conventions, the watchdog kill-test on the real chassis, one recorded run.

## Deliberately not built

- **Speed control over `follow_path`.** If the adaptive-speed thesis survives
  CLAUDE.md §11 it needs a mechanism that is not inference latency, and none is
  measured.
- **Diverting a coverage sweep to a detection.** That hybridises the two
  paradigms and would answer §11 by accident.
- **A search-and-explore behaviour.** The robot chases what it can see, within
  the `camera.fov_far_limit_m` range gate. It does not go looking for a
  fastener it cannot currently place on the floor. `mission.search: scan` turns
  on the spot, which finds things beside it but not things beyond the gate.

## Known gaps in the work itself

- **The `cli/` layer has no tests.** Most of it is argparse-only as intended,
  but two pieces have real logic worth covering: `_apply_override` in
  `cli/_common.py` (parses `--set`) and `load_points` in `cli/calib_ground.py`
  (parses the calibration correspondences — the one that will handle real
  measured data on a Saturday afternoon).
- **`fodnav-teleop` has never been run against a real terminal**, only
  type-checked and started. It is the first thing that will touch the chassis.
- **matplotlib is a declared dev dependency and is imported nowhere.** Either
  use it for sim plots or drop it; CLAUDE.md §0 calls the dependency list a
  budget.

---

## Presentation materials (outside the repo)

Built for the advisor meeting on the chase demo. They compute with the real
constants from `config/sim_robot.yaml` and `config/nav.yaml`, and the browser
ports were checked against the Python (physics agree to 1.3 mm over an
8-second run; perception and control to 5e-7).

| | |
|---|---|
| Watch It Chase — animated, drop screws and watch it collect them | `claude.ai/code/artifact/faaa6e56-5c31-4404-a384-46f994ba9a85` |
| One Bolt, Seven Steps — one detection traced through the pipeline | `claude.ai/code/artifact/c0986eab-dced-49c6-9334-7051f08677e2` |
| fodnav Code Map — what every function takes and returns | `claude.ai/code/artifact/fd4dee13-1a2a-464b-935a-f6dfd2743a5d` |
| Chase Demo Prep Sheet — likely questions and honest answers | `claude.ai/code/artifact/a4a1d7bb-3fbe-47e7-82ea-e43faced925d` |

These are private links. If the constants in the config change, the numbers
baked into those pages go stale and they need republishing.

**The advisor's brief for the next meeting:** the robot should chase a screw;
whether it picks it up is optional. So the chase is what to show, not coverage.
