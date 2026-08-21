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
from ..link.detections import SCHEMA_VERSION
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
    _rng: random.Random = field(init=False, repr=False)

    def __post_init__(self) -> None:
        self._rng = random.Random(self.noise.seed)

    def add(self, x: float, y: float, cls: str = "bolt", **kw) -> SimObject:
        obj = SimObject(x=x, y=y, cls=cls, **kw)
        self.objects.append(obj)
        return obj

    def render(self, true_pose: Pose2D, t_capture: float) -> dict:
        """One detection message, as the vision process would publish it."""
        dets = []
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
            conf = self.noise.conf + (
                self._rng.gauss(0.0, self.noise.conf_noise) if self.noise.conf_noise else 0.0
            )
            dets.append(
                {
                    "cls": obj.cls,
                    "conf": round(min(0.999, max(0.01, conf)), 3),
                    "bbox": [round(v, 1) for v in (x, y, w, h)],
                }
            )
        msg = {
            "schema": SCHEMA_VERSION,
            "t_capture": round(t_capture, 6),
            "t_publish": round(t_capture + 0.033, 6),  # the CV repo's measured 33.4 ms
            "frame_id": self.frame_id,
            "frame_size": [self.camera.width_px, self.camera.height_px],
            "dets": dets,
        }
        self.frame_id += 1
        return msg
