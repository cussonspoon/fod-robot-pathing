"""Argument groups and setup shared by the console scripts.

The CLI convention here mirrors the CV repo's: one module per command,
argparse only, then a call into the package module that does the work. Nothing
in ``cli/`` should contain logic worth testing -- if it does, it is in the
wrong place.
"""

from __future__ import annotations

import argparse
import atexit
import signal
import sys
from pathlib import Path

from ..config import Config, ConfigError, find_config_dir, load_nav_config, load_robot_config
from ..runlog import RunLog

__all__ = [
    "add_config_args",
    "add_log_args",
    "load_configs",
    "make_run_log",
    "install_safe_stop",
    "die",
]


def add_config_args(parser: argparse.ArgumentParser, default_robot: str | None = None) -> None:
    g = parser.add_argument_group("configuration")
    g.add_argument(
        "--robot-config",
        default=default_robot,
        help="physical constants (default: config/robot.yaml). "
        "Use config/sim_robot.yaml to run against a robot that does not exist.",
    )
    g.add_argument("--nav-config", default=None, help="gains and modes (default: config/nav.yaml)")
    g.add_argument("--config-dir", default=None, help="directory holding the config files")
    g.add_argument(
        "--set",
        action="append",
        default=[],
        metavar="PATH=VALUE",
        help="override one nav.yaml value for this run, e.g. --set mission.mode=target. "
        "Recorded in the run log. Not for physical constants.",
    )


def add_log_args(parser: argparse.ArgumentParser) -> None:
    g = parser.add_argument_group("run log")
    g.add_argument("--log-dir", default="logs", help="root for per-run log directories")
    g.add_argument("--run-name", default="", help="suffix for this run's directory")
    g.add_argument("--no-log", action="store_true", help="do not write a run directory")


def load_configs(args) -> tuple[Config, Config]:
    """Load both files and apply any ``--set`` overrides to the nav one."""
    cfg_dir = find_config_dir(args.config_dir)
    robot_path = Path(args.robot_config) if args.robot_config else cfg_dir / "robot.yaml"
    nav_path = Path(args.nav_config) if args.nav_config else cfg_dir / "nav.yaml"
    robot = load_robot_config(robot_path)
    nav = load_nav_config(nav_path)
    for override in getattr(args, "set", []):
        if "=" not in override:
            die(f"--set expects PATH=VALUE, got {override!r}")
        path, _, raw = override.partition("=")
        _apply_override(nav, path.strip(), raw.strip())
    return robot, nav


def _apply_override(nav: Config, path: str, raw: str) -> None:
    import yaml

    if path not in nav._schema:  # noqa: SLF001 - the CLI is the schema's owner-adjacent
        die(f"--set {path}: not a key in {nav.source}")
    try:
        value = nav._schema[path].coerce(path, yaml.safe_load(raw), "--set")  # noqa: SLF001
    except ConfigError as e:
        die(str(e))
    nav._values[path] = value  # noqa: SLF001


def make_run_log(args, name: str, robot: Config, nav: Config) -> RunLog | None:
    if getattr(args, "no_log", False):
        return None
    log = RunLog(root=args.log_dir, name=args.run_name or name)
    log.write_meta(argv=sys.argv)
    log.write_config(robot, nav)
    return log


def install_safe_stop(stopper) -> None:
    """Stop the wheels on the way out, however the process is leaving.

    A crashed or killed Pi process must not leave the last velocity command
    executing. The firmware watchdog is the real guarantee -- this is nav
    holding up its own end so the watchdog is a backstop rather than the plan.
    """
    atexit.register(stopper)

    def handler(signum, _frame):
        stopper()
        raise SystemExit(128 + signum)

    for sig in (signal.SIGINT, signal.SIGTERM):
        try:
            signal.signal(sig, handler)
        except (ValueError, OSError):
            pass  # not the main thread, or not supported here


def die(message: str, code: int = 2) -> None:
    print(f"error: {message}", file=sys.stderr)
    raise SystemExit(code)
