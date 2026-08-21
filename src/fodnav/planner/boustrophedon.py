"""Boustrophedon coverage of a known rectangle. A pure function.

No I/O, no state, no robot, no clock. It is the easiest thing in this repo to
test exhaustively and there is no excuse for it not to be, so the properties it
must satisfy are stated here and checked in ``tests/test_boustrophedon.py``:

* every point in the rectangle is within ``spacing / 2`` of some path segment;
* no waypoint lies outside the rectangle;
* the waypoint count matches the closed form;
* reversing ``start_corner`` mirrors the path.

Rows run parallel to the **long** axis by default, because turns are the
expensive part and that minimises how many there are. Only row endpoints are
emitted; the controller does the in-place turns between them.

**What "coverage" means is not settled.** ``swath_w`` has two defensible
values and they are not the same number: the drum width if coverage means
*collection*, or the camera's ground footprint at the lookahead if it means
*detection*. On the simulated geometry those differ by a factor of three. Which
is correct depends on the unresolved evaluation question (CLAUDE.md section 11),
so it stays a parameter, defaults to the drum, and :func:`swath_width_from_config`
returns the choice along with the number so the run log can record which one
was used.

Out of scope, deliberately: coverage over an *unknown* map. That needs SLAM
bring-up, occupancy-grid tuning and cell decomposition, and the advisor has
already granted the deferral in writing. Coverage over a known rectangle is a
function of four corners and a swath width, and this is it.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from ..config import Config

__all__ = [
    "Rect",
    "plan",
    "row_count",
    "row_spacing",
    "path_length",
    "coverage_gaps",
    "swath_width_from_config",
]

_EPS = 1e-9
CORNERS = ("sw", "se", "nw", "ne")
ORIENTATIONS = ("long", "short", "x", "y")


@dataclass(frozen=True)
class Rect:
    """An axis-aligned arena rectangle, in ``world`` metres."""

    x_min: float
    y_min: float
    x_max: float
    y_max: float

    def __post_init__(self) -> None:
        if self.x_max <= self.x_min or self.y_max <= self.y_min:
            raise ValueError(
                f"degenerate rectangle: x [{self.x_min}, {self.x_max}], "
                f"y [{self.y_min}, {self.y_max}]"
            )

    @property
    def width(self) -> float:
        """Extent along x."""
        return self.x_max - self.x_min

    @property
    def height(self) -> float:
        """Extent along y."""
        return self.y_max - self.y_min

    @property
    def long_axis(self) -> str:
        return "x" if self.width >= self.height else "y"

    @property
    def short_axis(self) -> str:
        return "y" if self.long_axis == "x" else "x"

    @property
    def area(self) -> float:
        return self.width * self.height

    def contains(self, x: float, y: float, tol: float = 1e-9) -> bool:
        return (
            self.x_min - tol <= x <= self.x_max + tol
            and self.y_min - tol <= y <= self.y_max + tol
        )

    def inset(self, margin: float) -> "Rect":
        """Shrink by ``margin`` on every side.

        The arena rectangle the robot may drive its *axle midpoint* through is
        the physical arena inset by half the footprint plus a clearance --
        otherwise it clips the edge on its turns. ``chassis.planner_margin_m``
        is that number and it is measured, not guessed.
        """
        r = Rect(
            self.x_min + margin, self.y_min + margin,
            self.x_max - margin, self.y_max - margin,
        )
        return r


def row_spacing(swath_w: float, overlap: float) -> float:
    """Distance between adjacent rows.

    Overlap is a fraction of the swath, so ``spacing = swath_w * (1 - overlap)``.
    Overlap exists because the rows will not be where the planner thinks they
    are: odometry drift, a track-width error and a turn that ends a few degrees
    off all push the actual row sideways, and a gap between rows is a strip of
    floor that never got swept.
    """
    if swath_w <= 0:
        raise ValueError(f"swath width must be positive, got {swath_w}")
    if not 0.0 <= overlap < 1.0:
        raise ValueError(f"overlap must be in [0, 1), got {overlap}")
    return swath_w * (1.0 - overlap)


def row_count(span: float, spacing: float) -> int:
    """How many rows are needed to cover ``span``. The closed form.

    A row sweeps ``spacing`` of width centred on itself, so ``n`` rows cover
    ``n * spacing`` and the answer is the ceiling of the ratio. Never fewer
    than one: a strip narrower than a single swath still needs sweeping.
    """
    if spacing <= 0:
        raise ValueError(f"row spacing must be positive, got {spacing}")
    return max(1, math.ceil(span / spacing - _EPS))


def plan(
    rect: Rect,
    swath_w: float,
    overlap: float = 0.0,
    start_corner: str = "sw",
    orientation: str = "long",
) -> list[tuple[float, float]]:
    """Row endpoints for a boustrophedon sweep of ``rect``.

    ``orientation`` is ``long`` (rows along the rectangle's longer axis, the
    default and the one that minimises turns), ``short``, or an explicit ``x``
    or ``y``. ``start_corner`` is one of ``sw``, ``se``, ``nw``, ``ne``.

    Rows are distributed evenly between half a spacing inside each edge rather
    than stepped from one edge, so that the last row lands *inside* the
    rectangle instead of just past it. Stepping is the obvious implementation
    and it puts a waypoint outside the arena whenever the span is not an exact
    multiple of the spacing, which is almost always.
    """
    if start_corner not in CORNERS:
        raise ValueError(f"start_corner must be one of {CORNERS}, got {start_corner!r}")
    if orientation not in ORIENTATIONS:
        raise ValueError(f"orientation must be one of {ORIENTATIONS}, got {orientation!r}")

    axis = {
        "long": rect.long_axis,
        "short": rect.short_axis,
        "x": "x",
        "y": "y",
    }[orientation]

    spacing = row_spacing(swath_w, overlap)
    # Along-row extent, and the perpendicular the rows are stacked across.
    if axis == "x":
        along = (rect.x_min, rect.x_max)
        across = (rect.y_min, rect.y_max)
    else:
        along = (rect.y_min, rect.y_max)
        across = (rect.x_min, rect.x_max)

    span = across[1] - across[0]
    n = row_count(span, spacing)
    offsets = _row_offsets(across[0], across[1], n, spacing)

    # sw/se/nw/ne say which end of each axis the sweep begins at.
    start_low_along = start_corner in ("sw", "nw") if axis == "x" else start_corner in ("sw", "se")
    start_low_across = start_corner in ("sw", "se") if axis == "x" else start_corner in ("sw", "nw")

    if not start_low_across:
        offsets = list(reversed(offsets))

    waypoints: list[tuple[float, float]] = []
    forward = start_low_along
    for offset in offsets:
        a, b = (along[0], along[1]) if forward else (along[1], along[0])
        if axis == "x":
            waypoints += [(a, offset), (b, offset)]
        else:
            waypoints += [(offset, a), (offset, b)]
        forward = not forward  # as the ox turns
    return waypoints


def _row_offsets(lo: float, hi: float, n: int, spacing: float) -> list[float]:
    if n == 1:
        return [0.5 * (lo + hi)]
    first = lo + 0.5 * spacing
    last = hi - 0.5 * spacing
    if last < first:  # the span is under one spacing but n > 1 cannot happen; be safe
        return [0.5 * (lo + hi)]
    step = (last - first) / (n - 1)
    return [first + i * step for i in range(n)]


def path_length(waypoints: list[tuple[float, float]]) -> float:
    """Driven distance along the polyline, excluding the in-place turns."""
    return sum(
        math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(waypoints, waypoints[1:])
    )


def coverage_gaps(
    rect: Rect, waypoints: list[tuple[float, float]], spacing: float, samples: int = 60
) -> list[tuple[float, float]]:
    """Points in ``rect`` further than ``spacing / 2`` from every path segment.

    The property the planner exists to satisfy, as a function, so that it can
    be asserted in tests *and* checked against a real plan at run time before
    the robot is asked to drive it.
    """
    gaps: list[tuple[float, float]] = []
    half = 0.5 * spacing + 1e-9
    for i in range(samples + 1):
        x = rect.x_min + rect.width * i / samples
        for j in range(samples + 1):
            y = rect.y_min + rect.height * j / samples
            if _distance_to_path(waypoints, x, y) > half:
                gaps.append((x, y))
    return gaps


def _distance_to_path(waypoints: list[tuple[float, float]], x: float, y: float) -> float:
    best = float("inf")
    for a, b in zip(waypoints, waypoints[1:]):
        dx, dy = b[0] - a[0], b[1] - a[1]
        seg2 = dx * dx + dy * dy
        if seg2 < 1e-18:
            d = math.hypot(x - a[0], y - a[1])
        else:
            t = max(0.0, min(1.0, ((x - a[0]) * dx + (y - a[1]) * dy) / seg2))
            d = math.hypot(x - (a[0] + t * dx), y - (a[1] + t * dy))
        best = min(best, d)
    return best


def swath_width_from_config(robot: Config, nav: Config) -> tuple[float, str]:
    """Resolve ``planner.swath_source`` to a width, and say which one it was.

    Returns ``(width_m, source)``. The caller writes ``source`` into the run
    log. CLAUDE.md section 9 is explicit that the choice is unresolved and that
    it must be recorded per run rather than assumed -- the two candidates
    differ by a factor of three, so a result that does not say which was used
    cannot be compared with any other result.
    """
    source = nav.get("planner.swath_source")
    key = {
        "drum": "drum.width_m",
        "drum_capture": "drum.capture_width_m",
        "camera": "camera.fov_width_at_lookahead_m",
    }[source]
    (width,) = robot.require(key, needed_by=f"the coverage planner (swath_source={source})")
    return width, source
