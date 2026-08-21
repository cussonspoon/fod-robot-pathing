#!/usr/bin/env python3
"""Publish the section 8 detection schema at 30 Hz, with no camera involved.

The vision topic does not exist yet. ``camera_hailo.py`` currently renders a
preview and publishes nothing, and the schema in CLAUDE.md section 8 is being
*requested* of Bthcorn rather than implemented by him. Until it lands this is
the only publisher there is, and it is what makes the entire nav stack runnable
on a laptop with no camera, no Pi and no robot.

It deliberately does **not** share message-building code with
``fodnav.link.detections``. That module parses; this one pretends to be
somebody else's process. Only the topic name and schema version are imported,
so those two cannot drift.

Scenarios::

    static    a target sitting still at a floor position, in the base frame
    moving    a target that approaches, or crosses the field of view
    empty     valid messages carrying no detections -- "the floor is clear",
              which is NOT the same as no message and must not be treated as it
    dropout   publishes, then stops dead, so the vision-timeout path gets run

Positions are metres in ``base``: +x forward, +y left, from the midpoint of the
drive-wheel axle. The pixels are synthesised by projecting through the
fictional camera in ``fodnav.sim.camera``, so a target that leaves the frame
stops being published -- which is exactly the terminal blind leg, and it is
better to meet it here than on demo day.

Examples::

    python tools/fake_detections.py --scenario static --x 0.8 --y 0.1
    python tools/fake_detections.py --scenario moving --motion approach
    python tools/fake_detections.py --scenario dropout --dropout-after 5
    python tools/fake_detections.py --scenario static --print --duration 1
    python tools/fake_detections.py --scenario moving --jsonl logs/fake.jsonl
"""

from __future__ import annotations

import argparse
import json
import math
import random
import sys
import time
from pathlib import Path

# Run from a source checkout without installing.
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from fodnav.config import load_robot_config  # noqa: E402
from fodnav.link.detections import DEFAULT_TOPIC, SCHEMA_VERSION  # noqa: E402
from fodnav.sim.camera import DEFAULT_HFOV_DEG, SimCamera, SimObject  # noqa: E402


def build_message(frame_id: int, t_capture: float, frame_size, dets: list[dict]) -> dict:
    """One message, built by hand, the way a foreign publisher would build it."""
    return {
        "schema": SCHEMA_VERSION,
        "t_capture": round(t_capture, 6),
        "t_publish": round(time.time(), 6),
        "frame_id": frame_id,
        "frame_size": [int(frame_size[0]), int(frame_size[1])],
        "dets": dets,
    }


def det_dict(cls: str, conf: float, bbox, rng: random.Random, jitter_px: float) -> dict:
    x, y, w, h = bbox
    if jitter_px:
        x += rng.gauss(0.0, jitter_px)
        y += rng.gauss(0.0, jitter_px)
        w = max(1.0, w + rng.gauss(0.0, jitter_px))
        h = max(1.0, h + rng.gauss(0.0, jitter_px))
    return {
        "cls": cls,
        "conf": round(conf, 3),
        "bbox": [round(v, 1) for v in (x, y, w, h)],
    }


def target_position(args, t: float) -> tuple[float, float]:
    """Where the target is, in ``base`` metres, at elapsed time ``t``."""
    if args.scenario != "moving":
        return (args.x, args.y)
    if args.motion == "approach":
        # Closes at a constant rate and then goes past. This is what the robot
        # sees while servoing, including the moment the target drops out of
        # frame short of the drum.
        return (max(0.0, args.x - args.speed * t), args.y)
    if args.motion == "cross":
        return (args.x, args.y + args.amplitude * math.sin(2.0 * math.pi * args.period_hz * t))
    # "wander": a slow random walk, for association-gate testing
    return (
        args.x + args.amplitude * math.sin(2.0 * math.pi * args.period_hz * t),
        args.y + args.amplitude * math.cos(2.0 * math.pi * args.period_hz * t * 0.7),
    )


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    ap.add_argument("--scenario", choices=("static", "moving", "empty", "dropout"), default="static")
    ap.add_argument("--motion", choices=("approach", "cross", "wander"), default="approach")
    ap.add_argument("--x", type=float, default=0.90, help="target x in base, metres (forward)")
    ap.add_argument("--y", type=float, default=0.05, help="target y in base, metres (left)")
    ap.add_argument("--speed", type=float, default=0.15, help="approach speed, m/s")
    ap.add_argument("--amplitude", type=float, default=0.25, help="lateral swing, metres")
    ap.add_argument("--period-hz", type=float, default=0.15)
    ap.add_argument("--cls", default="bolt", help="class label to publish (see CLAUDE.md §8)")
    ap.add_argument("--conf", type=float, default=0.87)
    ap.add_argument("--conf-noise", type=float, default=0.04)
    ap.add_argument("--jitter-px", type=float, default=1.5, help="per-frame box jitter")
    ap.add_argument("--miss-rate", type=float, default=0.0, help="fraction of frames the detector misses")
    ap.add_argument("--clutter", action="store_true",
                    help="also publish 'unknown' boxes; nav must ignore them")
    ap.add_argument("--dropout-after", type=float, default=5.0,
                    help="dropout scenario: seconds of publishing before going silent")
    ap.add_argument("--dropout-resume", type=float, default=0.0,
                    help="dropout scenario: resume after this many seconds of silence (0 = never)")
    ap.add_argument("--rate", type=float, default=30.0, help="publish rate, Hz")
    ap.add_argument("--duration", type=float, default=0.0, help="seconds to run, 0 = forever")
    ap.add_argument("--robot-config", default="config/sim_robot.yaml",
                    help="camera geometry to synthesise pixels through")
    ap.add_argument("--hfov-deg", type=float, default=DEFAULT_HFOV_DEG)
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=1883)
    ap.add_argument("--topic", default=DEFAULT_TOPIC)
    ap.add_argument("--print", dest="to_stdout", action="store_true",
                    help="print messages instead of publishing; no broker needed")
    ap.add_argument("--jsonl", default="", help="also append messages to a replayable JSONL log")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--quiet", action="store_true")
    args = ap.parse_args(argv)

    rng = random.Random(args.seed)
    robot = load_robot_config(args.robot_config)
    cam = SimCamera(robot, hfov_deg=args.hfov_deg)
    frame_size = (cam.width_px, cam.height_px)

    client = None
    if not args.to_stdout:
        import paho.mqtt.client as mqtt

        try:
            client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id="fake-detections")
        except AttributeError:  # paho 1.x
            client = mqtt.Client(client_id="fake-detections")
        client.connect(args.host, args.port, keepalive=10)
        client.loop_start()

    jsonl = None
    if args.jsonl:
        p = Path(args.jsonl)
        p.parent.mkdir(parents=True, exist_ok=True)
        jsonl = p.open("a", encoding="utf-8")

    if not args.quiet:
        print(
            f"fake_detections: scenario={args.scenario} rate={args.rate} Hz "
            f"topic={args.topic} frame={frame_size[0]}x{frame_size[1]} "
            f"camera={args.robot_config} (fictional optics, {args.hfov_deg:.0f} deg)",
            file=sys.stderr,
        )
        if args.scenario == "dropout":
            print(
                f"fake_detections: going silent at t={args.dropout_after:.1f}s. "
                f"Nav must stop, not coast.",
                file=sys.stderr,
            )

    period = 1.0 / args.rate
    t0 = time.monotonic()
    next_tick = t0
    frame_id = 0
    n_published = 0
    silent = False

    try:
        while True:
            now = time.monotonic()
            t = now - t0
            if args.duration and t >= args.duration:
                break

            if args.scenario == "dropout":
                past = t >= args.dropout_after
                resumed = (
                    args.dropout_resume > 0.0 and t >= args.dropout_after + args.dropout_resume
                )
                should_be_silent = past and not resumed
                if should_be_silent and not silent and not args.quiet:
                    print(f"fake_detections: silent at t={t:.2f}s", file=sys.stderr)
                if not should_be_silent and silent and not args.quiet:
                    print(f"fake_detections: publishing again at t={t:.2f}s", file=sys.stderr)
                silent = should_be_silent
                if silent:
                    next_tick += period
                    time.sleep(max(0.0, next_tick - time.monotonic()))
                    continue

            dets: list[dict] = []
            if args.scenario != "empty":
                tx, ty = target_position(args, t)
                bbox = cam.bbox_for(SimObject(x=tx, y=ty, cls=args.cls))
                if bbox is not None and rng.random() >= args.miss_rate:
                    conf = min(0.999, max(0.01, args.conf + rng.gauss(0.0, args.conf_noise)))
                    dets.append(det_dict(args.cls, conf, bbox, rng, args.jitter_px))
            if args.clutter:
                # 'unknown' is 53% of the training data and the class that fires
                # on furniture. Nav must drop these; publishing them is how we
                # find out whether it does.
                for cx, cy in ((1.35, 0.55), (1.10, -0.62)):
                    b = cam.bbox_for(SimObject(x=cx, y=cy, width_m=0.25, length_m=0.25, height_m=0.30))
                    if b is not None:
                        dets.append(det_dict("unknown", 0.4 + 0.4 * rng.random(), b, rng, 0.0))

            msg = build_message(frame_id, time.time(), frame_size, dets)
            payload = json.dumps(msg, separators=(",", ":"))
            if client is not None:
                client.publish(args.topic, payload, qos=0, retain=False)
            if args.to_stdout:
                print(payload)
            if jsonl is not None:
                jsonl.write(
                    json.dumps(
                        {"t_recv": time.time(), "topic": args.topic, "msg": msg},
                        separators=(",", ":"),
                    )
                    + "\n"
                )
            frame_id += 1
            n_published += 1

            next_tick += period
            sleep = next_tick - time.monotonic()
            if sleep < -0.5 * period:  # fell behind; resync rather than sprint
                next_tick = time.monotonic() + period
            elif sleep > 0:
                time.sleep(sleep)
    except KeyboardInterrupt:
        pass
    finally:
        if client is not None:
            client.loop_stop()
            client.disconnect()
        if jsonl is not None:
            jsonl.close()
        if not args.quiet:
            print(f"fake_detections: {n_published} messages published", file=sys.stderr)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
