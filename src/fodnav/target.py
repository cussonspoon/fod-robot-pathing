"""Association and confidence hysteresis: many boxes per second, one target.

A detector at 30 Hz produces a stream of independent guesses. A controller
needs one thing to chase, that does not vanish because a single frame missed
it and does not jump to a different object because that one scored 0.02 higher.
This module is the difference.

Association is nearest-neighbour in the ``base`` frame with a gate, as
CLAUDE.md section 8 specifies. Note the assumption that makes that adequate:
the robot moves ~1 cm between frames at 30 Hz and 0.3 m/s, against a gate of
20 cm, so not compensating for the robot's own motion costs nothing. If sweep
speed ever rises far enough that inter-frame motion approaches the gate, the
fix is to carry tracks in ``odom`` instead -- 33 ms of odometry drift is
nothing, so it would not compromise the servo's drift-immunity -- but it is not
needed at these speeds and the simpler thing is the specified thing.

Confidence is hysteretic: a box must clear ``acquire_conf`` to start a track
and only has to stay above ``drop_conf`` to keep it. One threshold would make a
detector sitting near it flicker, and a flickering target thrashes the FSM.

The CV repo has a tracker with hysteresis whose approach is worth reading
before changing this one.
"""

from __future__ import annotations

import itertools
import math
from dataclasses import dataclass, field

from .config import Config
from .frames import Pose2D
from .ground import GroundPoint

__all__ = ["Track", "TrackerParams", "TargetTracker"]

_ids = itertools.count(1)


@dataclass
class Track:
    """One object being followed across frames, in the ``base`` frame."""

    x: float
    y: float
    conf: float
    id: int = field(default_factory=lambda: next(_ids))
    hits: int = 1
    misses: int = 0
    confirmed: bool = False
    first_seen: float = 0.0
    last_seen: float = 0.0
    cls: str = ""
    #: Where odometry thought the robot was when this track was last measured.
    #: Kept so that a consumer running faster than the detector can age the
    #: estimate forward -- which is what ``t_capture`` is in the message for.
    odom_pose: Pose2D = field(default_factory=Pose2D)

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

        The control loop runs at 50 Hz and the detector publishes at 30, so on
        most ticks the newest measurement is up to 33 ms old -- about a
        centimetre of robot motion, which is enough to shift where the blind
        leg latches. Rotating the stale estimate into the current frame costs
        nothing and uses only tens of milliseconds of odometry, so it does not
        compromise the servo's independence from drift.
        """
        world_pt = self.odom_pose.transform_point(self.x, self.y)
        x, y = odom_now.inverse_transform_point(*world_pt)
        return GroundPoint(x, y)


@dataclass(frozen=True)
class TrackerParams:
    acquire_conf: float = 0.5
    drop_conf: float = 0.3
    assoc_max_jump_m: float = 0.2
    min_hits: int = 3
    max_misses: int = 5

    @classmethod
    def from_config(cls, nav: Config) -> "TrackerParams":
        return cls(
            acquire_conf=nav.get("detections.acquire_conf"),
            drop_conf=nav.get("detections.drop_conf"),
            assoc_max_jump_m=nav.get("detections.assoc_max_jump_m"),
            min_hits=nav.get("detections.min_hits"),
            max_misses=nav.get("detections.max_misses"),
        )


class TargetTracker:
    """Nearest-neighbour association with a gate, plus confidence hysteresis."""

    def __init__(self, params: TrackerParams | None = None) -> None:
        self.params = params or TrackerParams()
        self.tracks: list[Track] = []

    def reset(self) -> None:
        self.tracks.clear()

    def update(
        self,
        observations: list[tuple[GroundPoint, float, str]],
        now: float,
        odom_pose: Pose2D = Pose2D(),
    ) -> list[Track]:
        """Fold one frame's projected detections in. Returns confirmed tracks.

        ``observations`` are already projected to the floor and already
        class-filtered -- this module has no opinion about which classes are
        targets, because CLAUDE.md section 8 has a strong one and it belongs
        where the message is parsed.
        """
        p = self.params
        unmatched = list(self.tracks)
        matched: set[int] = set()

        # Greedy nearest-neighbour, closest pair first, so that two detections
        # competing for one track resolve the way a human would expect.
        pairs: list[tuple[float, int, Track]] = []
        for oi, (pt, conf, _cls) in enumerate(observations):
            for tr in unmatched:
                d = math.hypot(pt.x - tr.x, pt.y - tr.y)
                if d <= p.assoc_max_jump_m:
                    pairs.append((d, oi, tr))
        pairs.sort(key=lambda t: t[0])

        claimed_tracks: set[int] = set()
        for _d, oi, tr in pairs:
            if oi in matched or tr.id in claimed_tracks:
                continue
            pt, conf, cls = observations[oi]
            tr.x, tr.y, tr.conf, tr.cls = pt.x, pt.y, conf, cls
            tr.odom_pose = odom_pose
            tr.hits += 1
            tr.misses = 0
            tr.last_seen = now
            if tr.hits >= p.min_hits and conf >= p.drop_conf:
                tr.confirmed = True
            matched.add(oi)
            claimed_tracks.add(tr.id)

        # Anything left over is a new object -- but only if it is confident
        # enough to be worth starting a track for.
        for oi, (pt, conf, cls) in enumerate(observations):
            if oi in matched or conf < p.acquire_conf:
                continue
            self.tracks.append(
                Track(x=pt.x, y=pt.y, conf=conf, cls=cls, first_seen=now, last_seen=now,
                      odom_pose=odom_pose, confirmed=p.min_hits <= 1)
            )

        # Tracks nobody claimed this frame. A confirmed track survives on the
        # loose threshold; an unconfirmed one is dropped as soon as it misses,
        # because a one-frame flicker is not an object.
        for tr in unmatched:
            if tr.id in claimed_tracks:
                continue
            tr.misses += 1
            if tr.conf < p.drop_conf or tr.misses > p.max_misses:
                tr.confirmed = False

        self.tracks = [
            tr for tr in self.tracks if tr.misses <= p.max_misses and tr.conf >= p.drop_conf
        ]
        return self.confirmed()

    def confirmed(self) -> list[Track]:
        return [t for t in self.tracks if t.confirmed]

    def best(self, max_range_m: float = float("inf")) -> Track | None:
        """The nearest confirmed target within range, or ``None``.

        Nearest rather than most-confident: the robot is going to drive to it,
        and the nearest one is both the cheapest to reach and the one most
        likely to still be there when it arrives.
        """
        candidates = [t for t in self.confirmed() if t.range_m <= max_range_m]
        return min(candidates, key=lambda t: t.range_m, default=None)
