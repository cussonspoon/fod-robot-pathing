"""A world with objects in it, rendered into detection messages.

Closes the vision loop on a laptop. The scene holds objects at fixed positions
in ``world``; each frame it works out where they are relative to the robot's
**true** pose, images them through the fictional camera, and emits a message in
the CLAUDE.md section 8 schema. Nav then parses, projects, associates and drives
-- the whole pipeline, with nothing stubbed except the detector itself.

The failure modes a detector actually has are options here, because a servo
that only works against a perfect detector is not a result: dropped frames,
confidence noise, box jitter, and ``unknown`` clutter that nav must ignore.

What it does *not* fake is the geometry. Objects leave the frame when the
geometry says they leave the frame, which is how the terminal blind leg shows
up in simulation instead of on demo day.
"""

from __future__ import annotations

import random
from dataclasses import dataclass, field

from ..frames import Pose2D
from ..link.vision import CAUTION, CONFIRM, IGNORE, PICK
from .camera import SimCamera, SimObject

__all__ = ["SimScene", "SceneNoise"]


@dataclass(frozen=True)
class SceneNoise:
    """How badly the fictional detector behaves."""

    conf: float = 0.87
    conf_noise: float = 0.04
    jitter_px: float = 1.5
    miss_rate: float = 0.0
    seed: int = 0

    @classmethod
    def perfect(cls) -> "SceneNoise":
        return cls(conf_noise=0.0, jitter_px=0.0, miss_rate=0.0)


@dataclass
class SimScene:
    """Objects at fixed positions in ``world``, imaged from the robot's pose."""

    camera: SimCamera
    objects: list[SimObject] = field(default_factory=list)
    noise: SceneNoise = field(default_factory=SceneNoise)
    frame_id: int = 0
    lookahead: tuple[float, float] = (0.5, 1.0)
    _rng: random.Random = field(init=False, repr=False)

    def __post_init__(self) -> None:
        self._rng = random.Random(self.noise.seed)
        self._ids: dict[int, int] = {}
        self._state: dict[int, str] = {}
        self._hits: dict[int, int] = {}
        self._next_id = 901          # his ids are a per-process counter, not 0-based

    def add(self, x: float, y: float, cls: str = "bolt", **kw) -> SimObject:
        obj = SimObject(x=x, y=y, cls=cls, **kw)
        self.objects.append(obj)
        return obj

    def render(self, true_pose: Pose2D, t_capture: float) -> dict:
        """One ``detail()`` dictionary, as his library would return it.

        The sim emits **his** shape, not a schema of nav's own, so the simulated
        path and the real one are byte-identical in structure and there is
        nothing for the two to drift apart on. His tracker is modelled only as
        far as nav actually depends on it: a stable id per object, and the
        CONFIRM/CAUTION split at 0.5.
        """
        tracks = []
        for obj in self.objects:
            bx, by = true_pose.inverse_transform_point(obj.x, obj.y)
            local = SimObject(
                x=bx, y=by, cls=obj.cls,
                width_m=obj.width_m, length_m=obj.length_m, height_m=obj.height_m,
            )
            bbox = self.camera.bbox_for(local)
            if bbox is None:
                continue  # out of frame: exactly what the blind leg is about
            if self._rng.random() < self.noise.miss_rate:
                continue
            x, y, w, h = bbox
            j = self.noise.jitter_px
            if j:
                x += self._rng.gauss(0.0, j)
                y += self._rng.gauss(0.0, j)
                w = max(1.0, w + self._rng.gauss(0.0, j))
                h = max(1.0, h + self._rng.gauss(0.0, j))
            raw = self.noise.conf + (
                self._rng.gauss(0.0, self.noise.conf_noise) if self.noise.conf_noise else 0.0
            )
            raw = min(0.999, max(0.01, raw))
            tid = self._id_for(obj)
            # His hysteresis: CONFIRM at >= 0.5, latched until it falls under 0.25.
            was = self._state.get(tid, IGNORE)
            if raw >= 0.5:
                state = CONFIRM
            elif was == CONFIRM and raw >= 0.25:
                state = CONFIRM
            elif raw >= 0.25:
                state = CAUTION
            else:
                state = IGNORE
            self._state[tid] = state
            box = (int(round(x)), int(round(y)), int(round(x + w)), int(round(y + h)))
            centroid = (0.5 * (box[0] + box[2]), 0.5 * (box[1] + box[3]))
            lo, hi = self.lookahead
            H = self.camera.height_px
            tracks.append({
                "id": tid, "state": state, "action": PICK, "cls": obj.cls,
                "conf": round(raw, 4), "raw": round(raw, 4),
                "hits": self._hits.get(tid, 0) + 1, "misses": 0,
                "box": list(box), "centroid": list(centroid),
                "in_zone": bool(lo * H <= centroid[1] <= hi * H),
            })
            self._hits[tid] = self._hits.get(tid, 0) + 1

        detail = {
            "frame_id": self.frame_id,
            "age": 0.0,
            "blocked": any(t["in_zone"] and t["state"] == CONFIRM for t in tracks),
            "fps": 30.0,
            "stage_ms": {"capture": 17.4, "preprocess": 1.1, "infer": 14.7,
                         "postprocess": 0.1, "total": 33.3},
            "top_scores": {},
            "camera": {
                "zoom": 1.0, "rotate": 0, "conf": 0.25,
                "frame_size": [self.camera.width_px, self.camera.height_px],
                "imgsz": 640, "focus_m": None,
            },
            "tracks": tracks,
            "error": None,
        }
        self.frame_id += 1
        return detail

    def _id_for(self, obj: SimObject) -> int:
        key = id(obj)
        if key not in self._ids:
            self._ids[key] = self._next_id
            self._next_id += 1
        return self._ids[key]
