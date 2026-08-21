"""Every run writes a directory. A demo that worked once and cannot be
reproduced is worth nothing at the exam.

What goes in it (CLAUDE.md section 12): the detection JSONL, the
command/telemetry stream, the resolved config of both files, and the git SHA --
plus whether the tree was dirty at the time, because "the SHA" of a working
copy with uncommitted changes names something that was never built.

Nothing here is allowed to break a run. Every write is wrapped: a full disk or
a read-only card degrades the log, it does not stop the robot. And nothing here
blocks the control loop -- the per-tick stream is buffered and flushed on a
timer rather than on every line, because an fsync on an SD card is not a
20-millisecond operation.
"""

from __future__ import annotations

import json
import os
import platform
import subprocess
import sys
import time
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Any

from .config import Config

__all__ = ["RunLog", "git_describe"]


def git_describe(cwd: str | Path | None = None) -> dict[str, Any]:
    """The commit this ran at, and whether the tree was clean.

    A SHA on its own is a half-truth if the working copy had edits in it.
    """
    out: dict[str, Any] = {"sha": "", "dirty": None, "branch": ""}
    try:
        run = lambda *a: subprocess.run(  # noqa: E731
            a, cwd=cwd, capture_output=True, text=True, timeout=5
        )
        r = run("git", "rev-parse", "HEAD")
        if r.returncode == 0:
            out["sha"] = r.stdout.strip()
        r = run("git", "rev-parse", "--abbrev-ref", "HEAD")
        if r.returncode == 0:
            out["branch"] = r.stdout.strip()
        r = run("git", "status", "--porcelain")
        if r.returncode == 0:
            out["dirty"] = bool(r.stdout.strip())
    except Exception as e:  # not a git checkout, or no git
        out["error"] = str(e)
    return out


@dataclass
class RunLog:
    """One directory per run."""

    root: Path
    name: str = ""
    flush_every: int = 25  # ticks; 0.5 s at 50 Hz

    def __post_init__(self) -> None:
        stamp = datetime.now().strftime("%Y%m%d-%H%M%S")
        suffix = f"-{self.name}" if self.name else ""
        self.dir = Path(self.root) / f"{stamp}{suffix}"
        self.dir.mkdir(parents=True, exist_ok=True)
        self.t0 = time.time()
        self.n_ticks = 0
        self._stream = None
        self._since_flush = 0
        self._notes: list[str] = []
        try:
            self._stream = (self.dir / "stream.jsonl").open("w", encoding="utf-8")
        except OSError as e:
            self.note(f"could not open stream.jsonl: {e}")

    # -- static context -------------------------------------------------

    def write_meta(self, argv: list[str] | None = None, extra: dict | None = None) -> None:
        meta = {
            "started": datetime.now().isoformat(timespec="seconds"),
            "argv": list(argv if argv is not None else sys.argv),
            "cwd": os.getcwd(),
            "git": git_describe(),
            "python": sys.version.split()[0],
            "platform": platform.platform(),
            "host": platform.node(),
        }
        if extra:
            meta.update(extra)
        self._write_json("meta.json", meta)

    def write_config(self, robot: Config, nav: Config) -> None:
        """The *resolved* values, not the file. Comments do not run."""
        self._write_json(
            "config.json",
            {
                "robot": {"source": robot.source, "values": robot.resolved()},
                "nav": {"source": nav.source, "values": nav.resolved()},
            },
        )

    def copy_file(self, path: str | Path, name: str | None = None) -> None:
        """Snapshot something the run depended on -- the ground calibration."""
        try:
            src = Path(path)
            if src.is_file():
                (self.dir / (name or src.name)).write_bytes(src.read_bytes())
        except OSError as e:
            self.note(f"could not copy {path}: {e}")

    # -- the stream -----------------------------------------------------

    def tick(self, record: dict[str, Any]) -> None:
        """One line per control cycle. Never fsyncs on the hot path."""
        self.n_ticks += 1
        if self._stream is None:
            return
        try:
            self._stream.write(json.dumps(record, separators=(",", ":"), default=str) + "\n")
            self._since_flush += 1
            if self._since_flush >= self.flush_every:
                self._stream.flush()
                self._since_flush = 0
        except (OSError, ValueError) as e:
            self.note(f"stream write failed: {e}")
            self._stream = None

    def note(self, message: str) -> None:
        """Something a human should read afterwards."""
        line = f"[{time.time() - self.t0:8.3f}] {message}"
        self._notes.append(line)
        try:
            with (self.dir / "notes.log").open("a", encoding="utf-8") as fh:
                fh.write(line + "\n")
        except OSError:
            pass

    # -- close ----------------------------------------------------------

    def close(self, summary: dict[str, Any] | None = None) -> None:
        if self._stream is not None:
            try:
                self._stream.flush()
                self._stream.close()
            except OSError:
                pass
            self._stream = None
        payload = {
            "ended": datetime.now().isoformat(timespec="seconds"),
            "duration_s": round(time.time() - self.t0, 3),
            "ticks": self.n_ticks,
        }
        if summary:
            payload.update(summary)
        self._write_json("summary.json", payload)

    def _write_json(self, name: str, payload: dict) -> None:
        try:
            (self.dir / name).write_text(
                json.dumps(payload, indent=2, default=str) + "\n", encoding="utf-8"
            )
        except (OSError, TypeError) as e:
            self.note(f"could not write {name}: {e}")

    def __enter__(self) -> "RunLog":
        return self

    def __exit__(self, *exc: object) -> None:
        self.close()
