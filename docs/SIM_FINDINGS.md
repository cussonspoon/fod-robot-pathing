# What the simulator says so far

Results from the simulated stack, produced while building it. **Read the
caveats at the bottom before quoting any of these numbers** — they come from a
fictional robot, and only some of them survive contact with a real one.

Reproduce any of them with `fodnav-sim`; the exact invocations are given.

---

## 1. `move_to` over 4 m arrives confidently in the wrong place

A 1.5% effective-radius mismatch between the two drive wheels — the figure
CLAUDE.md §10 names as the error to model — over a single 4 m dead-reckoned
leg:

| | x | y | heading |
|---|---|---|---|
| Where odometry believed it was | 3.95 | 0.00 | +0.2° |
| Where it actually was | 3.94 | −0.51 | −15.2° |

**51 cm of position error and 15° of heading error**, and the odometry reports
none of it. The robot declares the waypoint reached and stops. Note the scale:
the miss is more than twice the width of the drum, so the fastener is not
picked up, and nothing in the log says anything went wrong.

This is not noise. Switch the scale mismatch off and leave the gaussian tick
noise on, and the same drive lands 7 cm out. The systematic term dominates the
random one by a factor of seven and it grows with distance.

```bash
uv run pytest tests/test_control.py -k odometry_arrives_but_the_robot_does_not
```

## 2. Visual servoing to the same target lands on it

Same chassis, same error model, same 30 Hz detector, target at a comparable
distance. The servo re-measures every frame, so drift has nothing to accumulate
into:

| Controller | Miss at the drum |
|---|---|
| `move_to` (dead reckoning) | 20–50 cm |
| Visual servo + terminal blind leg | **0.6–1.7 cm** |

That is the whole argument of CLAUDE.md §5, and it holds in simulation with the
detector degraded to 72% mean confidence, 30% of frames missed and 5 px of box
jitter.

```bash
uv run fodnav-sim --set mission.mode=target --target 1.4 0.35 --duration 30
```

## 3. The terminal blind leg is real and it is 33 cm

On the simulated mount geometry the target leaves the frame at 0.267 m and the
drum sits 0.060 m behind the axle, so **the robot drives 0.33 m after it can no
longer see what it is driving at.** In the traces the handover is visible as a
clean state change with no speed discontinuity:

```
 5.02 servoing    v=0.133  servoing, 0.272 m, +0.1 deg
 5.08 blind_leg   v=0.138  blind leg, 0.345 m to go
 8.92 arrived     v=0.000  drum over target after 0.345 m
```

`camera.fov_near_limit_m` is the measurement this depends on, and HARDWARE.md
§3.1 asks for it early for exactly this reason. If the real number comes back
near 0.5 m rather than 0.27 m, a third of every approach is open-loop and the
servo needs redesigning — that is worth knowing before the servo is trusted.

## 4. Open-loop coverage of the 3×3 m arena needs much better calibration than a 4 m drive does

A full boustrophedon sweep of the arena is **46.6 m of driving**, ten times the
length of a single approach, and the drift compounds the whole way. Sweeping
with the drum-*capture* swath (`drum.capture_width_m` = 18 cm, 15% overlap, so
rows 15.3 cm apart — 17 of them):

| Wheel-scale mismatch | Arena swept | Final odometry error |
|---|---|---|
| no errors at all (`--perfect`) | 99.5% | 0 cm |
| 0.0% | 94.0% | 25 cm |
| 0.1% | 94.3% | 20 cm |
| 0.2% | 90.8% | 57 cm |
| 0.5% | 82.0% | 159 cm |
| 1.0% | 69.5% | 267 cm |
| 1.5% | 51.5% | 280 cm |

Two things to read off this.

**To hold coverage above 90% the two wheels have to match to about 0.2%.** A
10-revolution roll test measures the radius to perhaps 1%; the spin test in
HARDWARE.md §2.2, which corrects the track width against a full 10 rotations,
is what closes the gap — and this table is why that procedure is not optional.

**Even perfectly matched wheels lose 5% of the arena**, because the other error
sources do not go away: the 0.0% row still has the shipped 0.5-tick noise and,
more importantly, a 0.002 rad/s unmodelled heading bias, which odometry cannot
see by construction. Over 46 m that alone is 25 cm of drift. Only the
`--perfect` row switches everything off, and no real robot gets that row.

If the measured mismatch turns out worse than ~0.5%, open-loop coverage of the
whole arena is not viable, and the honest options are a shorter sweep, an
absolute reference, or presenting coverage over a smaller patch. That is a
conversation to have with the advisor, not a gain to tune.

Each row, exactly:

```bash
uv run fodnav-sim --duration 600 --set planner.swath_source=drum_capture \
                  --set sim.wheel_scale_left=1.002
uv run fodnav-sim --duration 600 --set planner.swath_source=drum_capture --perfect
```

Note the `--set planner.swath_source=drum_capture`: the shipped default is
`drum`, which sweeps 22 cm rows instead of 18 cm and gives different numbers.
See §5 — this is exactly the trap that section is about.

## 5. The two readings of "coverage" differ by a factor of three

The unresolved question in CLAUDE.md §9 and §11, in numbers. Same arena, same
overlap, same planner — and note these are for the arena the planner actually
uses, the 3×3 m inset by `chassis.planner_margin_m` to 2.6×2.6 m:

| `planner.swath_source` | Swath | Rows | Driving |
|---|---|---|---|
| `drum` (collection, mechanical width) | 0.22 m | 14 | 38.8 m |
| `drum_capture` (collection, measured pickup width) | 0.18 m | 17 | 46.6 m |
| `camera` (detection, frame width at lookahead) | 0.64 m | 5 | 15.1 m |

A sweep that "covers the arena" under the detection reading drives **a third**
as far as one under the collection reading, and leaves two thirds of the floor
never passed over by a magnet. Every run records which was used, in
`summary.json` under `fsm.swath_source`. Do not compare two runs without
checking it.

## 6. Projecting the box centroid instead of its bottom edge

CLAUDE.md §3 says the ground point is the bottom-centre of the box and warns
that using the centroid "will look like a calibration problem". It does:

| Object | Bottom-centre error | Centroid error |
|---|---|---|
| Flat, at 0.4 m | −3.0 cm (constant) | +0.3 cm |
| Flat, at 1.1 m | −3.0 cm (constant) | +1.7 cm |
| 5 cm tall, at 0.7 m | −3.0 cm (constant) | +7.3 cm |
| 5 cm tall, at 1.1 m | −3.0 cm (constant) | **+12.5 cm** |

The bottom-centre error is a *constant* — half the object's length, because the
box's bottom edge is where the object's nearest part touches the floor. A
constant offset is something you can reason about. The centroid error grows
with both range and object height, which is indistinguishable from a bad
homography and would cost days.

---

## Caveats — what these numbers are not

* **The robot is fictional.** `config/sim_robot.yaml` is invented throughout:
  65 mm wheels, 200 mm track, a 102° lens at 22 cm and 18°. The *mechanisms*
  are geometric and scale, but every absolute number moves when the real
  measurements land. Nothing here should be re-typed into `config/robot.yaml`.
* **The chassis model is kinematic.** No motor lag, no acceleration limit, no
  tyre compliance. Real velocity steps are not instant, and the error that
  introduces is not modelled because nobody has measured a motor time constant.
* **The camera model is an ideal pinhole.** No lens distortion, no rolling
  shutter, no motion blur, and — importantly — no exposure time, so nothing
  here says anything about the blur question in CLAUDE.md §11. That still needs
  `ExposureTime` read from picamera2 metadata under arena lighting.
* **The detector is a geometry oracle with noise bolted on.** It never
  hallucinates a fastener that is not there, never mislabels one, and never
  misses one for a reason correlated with anything. A real detector does all
  three, and the association code is where that will show up first.
* **Coverage percentages are of the *inset* arena**, not the physical 3×3 m —
  the planner insets by `chassis.planner_margin_m` so the robot does not clip
  the walls on its turns.
