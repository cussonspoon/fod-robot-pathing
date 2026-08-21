"""``fodnav-sim`` -- run the whole nav stack against simulated hardware."""

from __future__ import annotations

import argparse
import json
import sys

from ..frames import Pose2D
from ..sim.harness import SimHarness
from ..sim.scene import SceneNoise
from ..sim.unicycle import SimParams
from ._common import add_config_args, add_log_args, die, load_configs, make_run_log


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        prog="fodnav-sim",
        description=__doc__,
        epilog="Everything except the chassis, camera and serial port is the real code.",
    )
    add_config_args(p, default_robot="config/sim_robot.yaml")
    add_log_args(p)
    p.add_argument(
        "--target", action="append", nargs=2, type=float, metavar=("X", "Y"), default=[],
        help="put a fastener on the floor at this world position; repeatable",
    )
    p.add_argument("--duration", type=float, default=120.0, help="simulated seconds")
    p.add_argument("--start", nargs=3, type=float, metavar=("X", "Y", "THETA"), default=[0, 0, 0])
    p.add_argument("--perfect", action="store_true",
                   help="switch off the odometry error model. Useful for isolating a "
                        "controller bug, useless for concluding a controller works.")
    p.add_argument("--vision-dies-at", type=float, default=None, metavar="SECONDS",
                   help="stop publishing detections at this time, to exercise the timeout")
    p.add_argument("--miss-rate", type=float, default=0.0, help="fraction of frames the detector misses")
    p.add_argument("--jitter-px", type=float, default=1.5)
    p.add_argument("--seed", type=int, default=None)
    p.add_argument("--json", action="store_true", help="print the summary as JSON")
    p.add_argument("--quiet", action="store_true")
    return p


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    robot, nav = load_configs(args)
    log = make_run_log(args, "sim", robot, nav)

    params = SimParams.perfect() if args.perfect else SimParams.from_config(nav)
    if args.seed is not None:
        params = SimParams(**{**params.__dict__, "seed": args.seed})
    noise = SceneNoise(
        miss_rate=args.miss_rate,
        jitter_px=args.jitter_px,
        seed=args.seed if args.seed is not None else 1,
    )

    harness = SimHarness(
        robot=robot,
        nav=nav,
        targets=[(x, y) for x, y in args.target],
        params=params,
        noise=noise,
        start_pose=Pose2D(*args.start),
        log=log,
        vision_dead_after_s=args.vision_dies_at,
    )
    if not args.quiet:
        # stderr, so that --json output stays pipeable.
        print(
            f"fodnav-sim: mode={harness.fsm.mode.value} robot={robot.source} "
            f"targets={len(harness.targets)} "
            f"errors={'off' if args.perfect else 'on'}",
            file=sys.stderr,
        )
        if log is not None:
            print(f"           log: {log.dir}", file=sys.stderr)

    try:
        stats = harness.run(duration_s=args.duration)
    finally:
        summary = harness.summary()
        if log is not None:
            log.close(summary)

    if args.json:
        print(json.dumps(summary, indent=2))
    elif not args.quiet:
        _report(harness, summary, stats)
    return 0


def _report(harness, summary, stats) -> None:
    print(f"\nstopped: {stats.stop_reason or 'duration reached'}")
    print(f"  sim time      {summary['sim_time_s']:8.2f} s over {stats.ticks} ticks")
    print(f"  driven        {summary['distance_driven_m']:8.2f} m")
    print(f"  odom error    {summary['odometry_error_m'] * 100:8.1f} cm at the end")
    print(f"  watchdog      {summary['watchdog_trips']} trips, {stats.overruns} loop overruns")
    print(f"  vision        {summary['frames_published']} frames published, "
          f"{summary['detections']['parsed']} parsed, {summary['detections']['malformed']} malformed")
    if "target_misses_m" in summary:
        for (x, y), miss in zip(harness.targets, summary["target_misses_m"]):
            verdict = "caught" if miss < harness._drum_w / 2 else "MISSED"
            print(f"  target ({x:.2f}, {y:.2f}): drum {miss * 100:5.1f} cm away -> {verdict}")
    if "coverage" in summary:
        c = summary["coverage"]
        print(f"  coverage      {c['fraction'] * 100:.1f}% of the arena "
              f"({summary['fsm']['swath_source']} swath, "
              f"{summary['fsm']['row_spacing_m'] * 100:.1f} cm rows)")
        print()
        print(harness.coverage.ascii_art(width=56))


if __name__ == "__main__":
    raise SystemExit(main())
