"""Turning his tracked pixels into one thing on the floor to drive at.

This module used to do nearest-neighbour association and confidence hysteresis
itself, because the schema nav was written against carried nothing but raw
boxes. **The real library does both already**, and better:

* it matches tracks in pixels on an 80 px radius and hands back a **stable
  ``id``** that never repeats, so "the same screw across an approach" is his
  answer, not ours;
* it smooths confidence with an EMA and latches ``CONFIRM`` at 0.5 until the
  score falls under 0.25 -- the hysteresis pair this module used to own.

So what is left here is only the part he will not do, and says so:

* **project his pixels to metres on the floor** (``ground.py``), and
* **age that measurement forward on odometry**, because his thread runs at 30 Hz
  and our control loop at 50 Hz, so most ticks are working from a measurement up
  to 33 ms old -- about a centimetre of robot motion, which is enough to move
  where the terminal blind leg latches.

Re-implementing his tracking on top of his tracking would be two hysteresis
loops in series, each lagging the other. Do not add one back.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

from .config import Config
from .frames import Pose2D
from .ground import GroundPoint, GroundProjector
from .link.vision import CONFIRM, PICK, Target, VisionFrame, select_targets

__all__ = ["Track", "TargetSet", "TargetParams"]


@dataclass
class Track:
    """One of his targets, projected onto the floor.

    ``x``/``y`` are metres in ``base`` **as measured**, paired with the odometry
    pose at that moment so the estimate can be carried forward.
    """

    id: int                      # his id, stable and never reused
    x: float
    y: float
    conf: float
    cls: str                     # diagnostic only -- never branch on it
    state: str
    action: str
    odom_pose: Pose2D
    last_seen: float
    first_seen: float

    @property
    def range_m(self) -> float:
        return math.hypot(self.x, self.y)

    @property
    def bearing_rad(self) -> float:
        return math.atan2(self.y, self.x)

    @property
    def point(self) -> GroundPoint:
        return GroundPoint(self.x, self.y)

    def age_s(self, now: float) -> float:
        return now - self.last_seen

    def predict_base(self, odom_now: Pose2D) -> GroundPoint:
        """This track in the *current* base frame, aged forward on odometry.

        Uses only the tens of milliseconds since the measurement, so it does not
        compromise the servo's independence from drift -- that independence
        comes from re-measuring every frame, not from refusing to use odometry
        at all.
        """
        world = self.odom_pose.transform_point(self.x, self.y)
        x, y = odom_now.inverse_transform_point(*world)
        return GroundPoint(x, y)


@dataclass(frozen=True)
class TargetParams:
    """What is left to tune on this side. Deliberately short.

    ``acquire_conf``, ``drop_conf``, ``min_hits``, ``max_misses`` and
    ``assoc_max_jump_m`` used to live here. They are his now.
    """

    #: Drop a projected track this long after its last sighting. A backstop for
    #: frames stopping altogether; his own tracker drops a track after 5 misses.
    max_age_s: float = 0.5

    @classmethod
    def from_config(cls, nav: Config) -> "TargetParams":
        return cls(max_age_s=nav.get("detections.max_age_s"))


class TargetSet:
    """His confirmed targets, on the floor, in the robot's frame."""

    def __init__(self, params: TargetParams | None = None) -> None:
        self.params = params or TargetParams()
        self.tracks: dict[int, Track] = {}
        self.n_projection_rejects = 0
        self.n_filtered_out = 0

    def reset(self) -> None:
        self.tracks.clear()

    def update(
        self,
        frame: VisionFrame,
        projector: GroundProjector,
        odom_pose: Pose2D,
        now: float,
    ) -> list[Track]:
        """Fold one frame in. Returns the live tracks.

        Filtering is on ``state`` and ``action`` only -- his class names are
        diagnostic and are going away.
        """
        projector.check_frame_size(frame.frame_size)
        kept: set[int] = set()
        for t in select_targets(frame, states=(CONFIRM,), actions=(PICK,)):
            point = projector.project_pixel(*t.ground_px)
            if point is None:
                # Above the horizon, behind, or beyond the calibrated patch.
                self.n_projection_rejects += 1
                continue
            kept.add(t.id)
            existing = self.tracks.get(t.id)
            self.tracks[t.id] = Track(
                id=t.id, x=point.x, y=point.y, conf=t.conf, cls=t.cls,
                state=t.state, action=t.action, odom_pose=odom_pose, last_seen=now,
                first_seen=existing.first_seen if existing else now,
            )
        self.n_filtered_out += len(frame.targets) - len(kept)
        self.tracks = {
            i: tr for i, tr in self.tracks.items()
            if tr.age_s(now) <= self.params.max_age_s
        }
        return list(self.tracks.values())

    def best(self, odom_now: Pose2D, max_range_m: float = float("inf")) -> Track | None:
        """The nearest track within range, or ``None``.

        Nearest rather than most confident: it is the cheapest to reach and the
        one most likely still to be there on arrival.
        """
        best_track, best_range = None, float("inf")
        for tr in self.tracks.values():
            r = tr.predict_base(odom_now).range_m
            if r < best_range and r <= max_range_m:
                best_range, best_track = r, tr
        return best_track
