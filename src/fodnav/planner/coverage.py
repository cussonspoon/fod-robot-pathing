"""Swept-cell bookkeeping: how much of the arena the drum actually passed over.

Answers the question a coverage run has to answer at the exam -- "what fraction
did it sweep, and where are the holes" -- from the *executed* trajectory rather
than from the planned one. Those differ, and the difference is the result.

The map is marked from the drum's footprint, not the robot's position: the
robot passing over a strip is not the same as the drum sweeping it, and the
drum is behind the axle and narrower than the chassis.

This is measurement, not planning. Nothing here feeds back into where the robot
goes -- coverage *replanning* (not re-sweeping cleared zones) is Project 2 and
was deferred by the advisor in writing.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from ..config import Config
from ..frames import Pose2D
from .boustrophedon import Rect

__all__ = ["CoverageMap"]


@dataclass
class CoverageMap:
    """An occupancy grid of what has been swept.

    ``cell_m`` trades resolution against memory and means nothing physically;
    5 cm on a 3x3 m arena is 3600 cells, which is free.
    """

    rect: Rect
    cell_m: float = 0.05

    def __post_init__(self) -> None:
        if self.cell_m <= 0:
            raise ValueError(f"cell size must be positive, got {self.cell_m}")
        self.nx = max(1, math.ceil(self.rect.width / self.cell_m))
        self.ny = max(1, math.ceil(self.rect.height / self.cell_m))
        self.cells = bytearray(self.nx * self.ny)
        self._last: tuple[float, float, float] | None = None

    # -- marking --------------------------------------------------------

    def sweep(self, pose: Pose2D, drum_width_m: float, drum_offset_x_m: float = 0.0) -> None:
        """Mark whatever the drum covered at this pose.

        Call it every control tick. At 50 Hz and a third of a metre per second
        the robot advances 5 mm per tick, well under a cell, so marking the
        drum's segment each tick leaves no gaps. If a tick is missed -- a
        stalled loop, a replayed log at speed -- the pose will have jumped, and
        the segment is interpolated rather than leaving a stripe of floor
        recorded as unswept when it was not.
        """
        cx, cy = pose.transform_point(drum_offset_x_m, 0.0)
        if self._last is not None:
            lx, ly, ltheta = self._last
            jump = math.hypot(cx - lx, cy - ly)
            steps = int(jump / (0.5 * self.cell_m))
            if steps > 1:
                for i in range(1, steps):
                    f = i / steps
                    self._mark_segment(
                        lx + (cx - lx) * f,
                        ly + (cy - ly) * f,
                        ltheta + _shortest(pose.theta - ltheta) * f,
                        drum_width_m,
                    )
        self._mark_segment(cx, cy, pose.theta, drum_width_m)
        self._last = (cx, cy, pose.theta)

    def _mark_segment(self, cx: float, cy: float, theta: float, width_m: float) -> None:
        half = 0.5 * width_m
        n = max(2, int(width_m / (0.5 * self.cell_m)) + 1)
        s, c = math.sin(theta), math.cos(theta)
        for i in range(n):
            off = -half + width_m * i / (n - 1)
            # The drum lies across the robot's +y axis, at the drum's x offset.
            self._mark(cx - s * off, cy + c * off)

    def _mark(self, x: float, y: float) -> None:
        i = int((x - self.rect.x_min) / self.cell_m)
        j = int((y - self.rect.y_min) / self.cell_m)
        if 0 <= i < self.nx and 0 <= j < self.ny:
            self.cells[j * self.nx + i] = 1

    # -- reading --------------------------------------------------------

    @property
    def n_cells(self) -> int:
        return self.nx * self.ny

    def n_covered(self) -> int:
        return sum(self.cells)

    def fraction_covered(self) -> float:
        return self.n_covered() / self.n_cells

    def covered(self, x: float, y: float) -> bool:
        i = int((x - self.rect.x_min) / self.cell_m)
        j = int((y - self.rect.y_min) / self.cell_m)
        if not (0 <= i < self.nx and 0 <= j < self.ny):
            return False
        return bool(self.cells[j * self.nx + i])

    def uncovered_cells(self) -> list[tuple[float, float]]:
        """Centres of the cells nothing swept. The holes, in metres."""
        out = []
        for j in range(self.ny):
            for i in range(self.nx):
                if not self.cells[j * self.nx + i]:
                    out.append(
                        (
                            self.rect.x_min + (i + 0.5) * self.cell_m,
                            self.rect.y_min + (j + 0.5) * self.cell_m,
                        )
                    )
        return out

    def ascii_art(self, width: int = 60) -> str:
        """A picture for the run log. +y is up, as it is on the map."""
        step = max(1, self.nx // width)
        rows = []
        for j in range(self.ny - 1, -1, -step):
            rows.append(
                "".join(
                    "#" if self.cells[j * self.nx + i] else "."
                    for i in range(0, self.nx, step)
                )
            )
        return "\n".join(rows)

    def summary(self) -> dict[str, float]:
        return {
            "cells": float(self.n_cells),
            "covered": float(self.n_covered()),
            "fraction": self.fraction_covered(),
            "cell_m": self.cell_m,
        }

    @classmethod
    def for_arena(cls, rect: Rect, robot: Config, cell_m: float = 0.05) -> "CoverageMap":
        return cls(rect=rect, cell_m=cell_m)


def _shortest(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi
