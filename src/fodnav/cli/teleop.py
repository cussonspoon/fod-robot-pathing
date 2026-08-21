"""``fodnav-teleop`` -- drive the robot by hand, with the heartbeat maintained.

The first thing anyone does with a new chassis, and the fastest way to find a
swapped encoder, a reversed motor or a track width that is 10% out. Watch the
telemetry line while you drive: forward should raise both tick counters,
turning left should raise the right one faster (HARDWARE.md section 2.4).

It is also the honest way to test the watchdog. Drive forward, then ``kill -9``
this process from another shell. The robot must stop within 300 ms. Do that on
the real chassis before the exam rather than during it.

Keys::

    w / s     forward / back      space   stop (zero both)
    a / d     left / right        e / x   enable / disable motors
    , / .     smaller / bigger step
    m         magnet drum on/off  q       quit
"""

from __future__ import annotations

import argparse
import select
import sys
import termios
import time
import tty

from ..config import ConfigError
from ..control import MotionLimits, saturate
from ..link.esp32 import Esp32Link, HandshakeError, SerialTransport
from ..odom import Odometry
from ._common import add_config_args, die, install_safe_stop, load_configs

HELP = """  w/s forward/back   a/d left/right   space stop
  e/x enable/disable  m magnet   ,/. step   q quit"""


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(prog="fodnav-teleop", description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    add_config_args(p)
    p.add_argument("--sim", action="store_true",
                   help="drive the simulated chassis instead of a serial port")
    p.add_argument("--port", default=None, help="override link.port from robot.yaml")
    p.add_argument("--step", type=float, default=0.05, help="velocity increment, m/s")
    p.add_argument("--omega-step", type=float, default=0.3, help="turn increment, rad/s")
    return p


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    if args.sim and args.robot_config is None:
        args.robot_config = "config/sim_robot.yaml"
    try:
        robot, nav = load_configs(args)
    except ConfigError as e:
        die(str(e))
    limits = MotionLimits.from_config(robot)

    if not sys.stdin.isatty():
        die(
            "teleop needs a terminal to read keys from. Run it directly rather than "
            "through a pipe.\nFor an unattended run use fodnav-run, or fodnav-sim "
            "for the simulated stack."
        )

    firmware = None
    if args.sim:
        from ..sim.firmware import build_sim_link
        from ..sim.unicycle import SimParams

        link, firmware = build_sim_link(robot, SimParams.from_config(nav))
        pump = lambda: firmware.step(nav.get("loop.dt_s"))  # noqa: E731
    else:
        port = args.port or robot.get("link.port")
        try:
            link = Esp32Link(SerialTransport(port, robot.get("link.baud")), robot=robot)
        except Exception as e:
            die(f"cannot open {port}: {e}")
        pump = None

    install_safe_stop(link.shutdown)
    try:
        info = link.open(
            boot_wait_s=0.0 if args.sim else nav.get("link.boot_wait_s"),
            handshake_timeout_s=nav.get("link.handshake_timeout_s"),
            pump=pump,
        )
    except HandshakeError as e:
        link.shutdown()
        die(str(e))

    odom = Odometry.from_config(robot)
    dt = nav.get("loop.dt_s")
    v = omega = 0.0
    step, omega_step = args.step, args.omega_step
    enabled = magnet = False
    quit_now = False

    print(f"teleop: firmware {info.fw_version}, v_max {limits.v_max} m/s")
    print(HELP)
    print("press e to enable the motors\n")

    fd = sys.stdin.fileno()
    interactive = True
    old = termios.tcgetattr(fd)
    try:
        if interactive:
            tty.setcbreak(fd)
        next_tick = time.monotonic()
        while not quit_now:
            for key in _read_keys(interactive):
                if key == "w":
                    v += step
                elif key == "s":
                    v -= step
                elif key == "a":
                    omega += omega_step
                elif key == "d":
                    omega -= omega_step
                elif key == " ":
                    v = omega = 0.0
                elif key == "e":
                    link.enable()
                    enabled = True
                elif key == "x":
                    link.disable()
                    enabled = False
                    v = omega = 0.0
                elif key == "m":
                    magnet = not magnet
                    link.magnet(magnet)
                elif key == ",":
                    step, omega_step = step / 2, omega_step / 2
                elif key == ".":
                    step, omega_step = step * 2, omega_step * 2
                elif key in ("q", "\x03"):
                    quit_now = True

            v = max(-limits.v_max, min(limits.v_max, v))
            omega = max(-limits.omega_max, min(limits.omega_max, omega))
            v_cmd, omega_cmd = saturate(v, omega, limits)

            # Unconditional, every tick, exactly as in the control loop: this
            # is what the watchdog is measuring.
            link.send_velocity(v_cmd, omega_cmd)
            if pump is not None:
                pump()
            for t in link.poll():
                odom.update(t.ticks_l, t.ticks_r, t=t.t_ms / 1000.0)

            _status(link, odom, v_cmd, omega_cmd, enabled, magnet, step)
            next_tick += dt
            time.sleep(max(0.0, next_tick - time.monotonic()))
    finally:
        if interactive and old is not None:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)
        link.close()
        print("\nstopped and disabled.")
    return 0


def _read_keys(interactive: bool) -> list[str]:
    if not interactive:
        return []
    keys = []
    while select.select([sys.stdin], [], [], 0.0)[0]:
        ch = sys.stdin.read(1)
        if not ch:
            break
        keys.append(ch.lower())
    return keys


def _status(link, odom, v, omega, enabled, magnet, step) -> None:
    t = link.last_telemetry
    flags = t.flags.describe() if t is not None else "no telemetry"
    ticks = f"{t.ticks_l:>9d} {t.ticks_r:>9d}" if t is not None else " " * 19
    sys.stdout.write(
        f"\rv {v:+.3f}  w {omega:+.3f}  step {step:.3f} | "
        f"{'EN' if enabled else 'dis'} {'MAG' if magnet else '   '} | "
        f"ticks {ticks} | x {odom.pose.x:+.2f} y {odom.pose.y:+.2f} "
        f"th {odom.pose.theta:+.2f} | {flags:<40}"
    )
    sys.stdout.flush()


if __name__ == "__main__":
    raise SystemExit(main())
