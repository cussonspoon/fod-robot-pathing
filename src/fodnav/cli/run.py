"""``fodnav-run`` -- the real thing: MQTT vision, serial to the ESP32, 50 Hz.

Startup follows docs/protocol.md section 7 exactly: open the port by its stable
path, wait for the ESP32 to boot, flush, handshake, verify the protocol version
and the three physical constants against ``config/robot.yaml``, and only then
begin the heartbeat. A mismatch aborts. It is not a warning.

The robot does not move until vision is alive, because the vision timeout makes
no distinction between "the camera process died" and "the camera process has
not started yet" -- and neither should it.
"""

from __future__ import annotations

import argparse
import json
import sys

from ..config import ConfigError
from ..fsm import NavFsm
from ..ground import projector_from_config
from ..link.detections import JsonlDetectionLog, MqttDetectionSource
from ..link.esp32 import Esp32Link, HandshakeError, SerialTransport
from ..odom import Odometry
from ..runner import ControlLoop, RealClock
from ._common import add_config_args, add_log_args, die, install_safe_stop, load_configs, make_run_log


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(prog="fodnav-run", description=__doc__)
    add_config_args(p)
    add_log_args(p)
    p.add_argument("--ground-calib", default="config/ground_homography.json",
                   help="the floor projection, from fodnav-calib-ground")
    p.add_argument("--port", default=None, help="override link.port from robot.yaml")
    p.add_argument("--duration", type=float, default=None, help="stop after this many seconds")
    p.add_argument("--dry-run", action="store_true",
                   help="load the config, open the link, handshake, and stop. "
                        "Checks everything except whether the robot drives well.")
    p.add_argument("--no-enable", action="store_true",
                   help="run the loop and the heartbeat but never send E, so the "
                        "wheels stay dead. For watching the FSM on a bench.")
    return p


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        robot, nav = load_configs(args)
    except ConfigError as e:
        die(str(e))

    log = make_run_log(args, "run", robot, nav)
    try:
        projector = projector_from_config(robot, args.ground_calib)
    except Exception as e:
        die(f"ground calibration: {e}")
    if log is not None:
        log.copy_file(args.ground_calib)

    port = args.port or robot.get("link.port")
    baud = robot.get("link.baud")
    try:
        transport = SerialTransport(port, baud)
    except Exception as e:
        die(f"cannot open {port}: {e}")

    def on_log(line):
        text = f"esp32: [{line.level}] {line.message}"
        print(text, file=sys.stderr)
        if log is not None:
            log.note(text)

    link = Esp32Link(transport, robot=robot, on_log=on_log)
    # Registered before anything can move, so that every exit path stops the
    # wheels -- including the ones nobody plans for.
    install_safe_stop(link.shutdown)

    print(f"fodnav-run: {port} @ {baud}, waiting {nav.get('link.boot_wait_s')} s for boot",
          file=sys.stderr)
    try:
        info = link.open(
            boot_wait_s=nav.get("link.boot_wait_s"),
            handshake_timeout_s=nav.get("link.handshake_timeout_s"),
        )
    except HandshakeError as e:
        link.shutdown()
        die(str(e))
    print(f"fodnav-run: firmware {info.fw_version}, protocol {info.proto_version}, "
          f"constants agree", file=sys.stderr)
    if log is not None:
        log.note(f"handshake ok: {info}")

    if args.dry_run:
        link.close()
        print("fodnav-run: dry run complete -- config, calibration and link all check out.",
              file=sys.stderr)
        if log is not None:
            log.close({"dry_run": True, "firmware": info.fw_version})
        return 0

    det_log = None
    if log is not None:
        det_log = JsonlDetectionLog(log.dir / "detections.jsonl")
    detections = MqttDetectionSource(
        host=nav.get("detections.broker_host"),
        port=nav.get("detections.broker_port"),
        topic=nav.get("detections.topic"),
        log=det_log,
    )
    detections.start()

    odom = Odometry.from_config(robot)
    fsm = NavFsm(robot, nav, projector=projector)
    loop = ControlLoop(
        robot=robot, nav=nav, link=link, detections=detections,
        fsm=fsm, odom=odom, clock=RealClock(), log=log,
    )

    if not args.no_enable:
        link.enable()
    print(f"fodnav-run: mode={fsm.mode.value}, {nav.get('loop.rate_hz'):.0f} Hz. "
          f"Ctrl-C to stop.", file=sys.stderr)

    summary = {}
    try:
        stats = loop.run(duration_s=args.duration)
        summary["loop"] = stats.as_dict()
    finally:
        loop.safe_stop()
        link.close()
        detections.stop()
        if det_log is not None:
            det_log.close()
        summary.update(
            {
                "fsm": fsm.status(),
                "link": link.stats.as_dict(),
                "detections": detections.stats.as_dict(),
                "odom": {
                    "x": odom.pose.x, "y": odom.pose.y, "theta": odom.pose.theta,
                    "distance_m": odom.distance_m,
                },
            }
        )
        if log is not None:
            log.close(summary)
            print(f"fodnav-run: log written to {log.dir}", file=sys.stderr)
    print(json.dumps(summary, indent=2, default=str))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
