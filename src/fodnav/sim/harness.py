"""Assemble the whole stack against simulated hardware.

Everything real except the chassis, the camera and the serial port: the same
control loop, the same FSM, the same codec, the same projection maths, the same
config loader. The seams are exactly three -- a :class:`SimClock` instead of the
wall clock, a loopback transport instead of pyserial, and a rendered scene
instead of a broker -- and they are the three places where there is no laptop
equivalent of the hardware.

That matters more than it sounds. A simulator that reimplements the control
loop tests the simulator. This one runs
:class:`~fodnav.runner.ControlLoop` unmodified, so a watchdog trip, a malformed
line, a wrapped tick counter or a mistimed send happens here the way it would
happen on the robot.
"""

from __future__ import annotations

import json
from dataclasses import dataclass, field

from ..config import Config
from ..control import MotionLimits
from ..frames import Pose2D
from ..fsm import Mode, NavFsm
from ..ground import GroundProjector
from ..link.detections import QueueDetectionSource
from ..link.esp32 import Esp32Link
from ..odom import Odometry
from ..planner.boustrophedon import Rect
from ..planner.coverage import CoverageMap
from ..runlog import RunLog
from ..runner import ControlLoop, SimClock
from .camera import SimCamera
from .firmware import FakeFirmware, FirmwareConstants, build_sim_link
from .scene import SceneNoise, SimScene
from .unicycle import SimParams, UnicycleSim

__all__ = ["SimHarness"]


@dataclass
class SimHarness:
    robot: Config
    nav: Config
    targets: list[tuple[float, float]] = field(default_factory=list)
    params: SimParams | None = None
    noise: SceneNoise | None = None
    start_pose: Pose2D = field(default_factory=Pose2D)
    log: RunLog | None = None
    vision_rate_hz: float = 30.0
    firmware_constants: FirmwareConstants | None = None
    vision_dead_after_s: float | None = None  # inject a vision dropout

    def __post_init__(self) -> None:
        self.camera = SimCamera(self.robot)
        self.calibration = self.camera.calibration()
        self.projector = GroundProjector(
            self.calibration, max_valid_range_m=self.robot.get("camera.fov_far_limit_m")
        )
        self.scene = SimScene(self.camera, noise=self.noise or SceneNoise(seed=1))
        for x, y in self.targets:
            self.scene.add(x, y)

        self.link, self.firmware = build_sim_link(
            self.robot,
            self.params or SimParams.from_config(self.nav),
            pose=self.start_pose,
            constants=self.firmware_constants,
            on_log=self._on_firmware_log,
        )
        self.sim: UnicycleSim = self.firmware.sim
        self.source = QueueDetectionSource()
        self.odom = Odometry.from_config(self.robot, pose=self.start_pose)
        self.odom.update(*self.sim.ticks)

        self.fsm = NavFsm(self.robot, self.nav, projector=self.projector)
        self.clock = SimClock(pump=self._pump, step=0.002)
        self.loop = ControlLoop(
            robot=self.robot,
            nav=self.nav,
            link=self.link,
            detections=self.source,
            fsm=self.fsm,
            odom=self.odom,
            clock=self.clock,
            log=self.log,
        )

        self._vision_period = 1.0 / self.vision_rate_hz
        self._next_frame = 0.0
        self.frames_published = 0
        self.coverage: CoverageMap | None = None
        if self.fsm.mode is Mode.COVERAGE and self.fsm.arena is not None:
            self.coverage = CoverageMap(
                self.fsm.arena.inset(self.robot.get("chassis.planner_margin_m")), cell_m=0.05
            )
        self._drum_w = self.robot.get("drum.capture_width_m")
        self._drum_x = self.robot.get("drum.offset_x_m")

    # -- the pump -------------------------------------------------------

    def _pump(self, dt: float) -> None:
        """Advance everything the loop does not own: firmware, chassis, camera."""
        self.firmware.step(dt)
        t = self.clock.t + dt
        if self.coverage is not None:
            self.coverage.sweep(self.sim.true_pose, self._drum_w, self._drum_x)
        if t >= self._next_frame:
            self._next_frame += self._vision_period
            if self.vision_dead_after_s is not None and t >= self.vision_dead_after_s:
                return  # the camera process died; nav must notice and stop
            self.source.offer(json.dumps(self.scene.render(self.sim.true_pose, t)))
            self.frames_published += 1

    def _on_firmware_log(self, line) -> None:
        if self.log is not None:
            self.log.note(f"esp32: [{line.level}] {line.message}")

    # -- running --------------------------------------------------------

    def start(self) -> None:
        """Handshake, verify the constants, then enable. Protocol section 7."""
        self.link.open(boot_wait_s=0.0, pump=lambda: self.firmware.step(0.002))
        self.link.enable()

    def run(self, duration_s: float = 120.0):
        self.start()
        return self.loop.run(duration_s=duration_s)

    # -- results --------------------------------------------------------

    def drum_position(self) -> tuple[float, float]:
        return self.sim.true_pose.transform_point(self._drum_x, 0.0)

    def drum_miss_m(self, target: tuple[float, float]) -> float:
        import math

        dx, dy = self.drum_position()
        return math.hypot(dx - target[0], dy - target[1])

    def summary(self) -> dict:
        import math

        out = {
            "sim_time_s": round(self.clock.t, 3),
            "frames_published": self.frames_published,
            "true_pose": [
                round(self.sim.true_pose.x, 4),
                round(self.sim.true_pose.y, 4),
                round(self.sim.true_pose.theta, 4),
            ],
            "odom_pose": [
                round(self.odom.pose.x, 4),
                round(self.odom.pose.y, 4),
                round(self.odom.pose.theta, 4),
            ],
            "odometry_error_m": round(
                math.hypot(
                    self.sim.true_pose.x - self.odom.pose.x,
                    self.sim.true_pose.y - self.odom.pose.y,
                ),
                4,
            ),
            "distance_driven_m": round(self.sim.distance_m, 3),
            "watchdog_trips": self.firmware.n_watchdog_trips,
            "malformed_commands": self.firmware.n_malformed,
            "loop": self.loop.stats.as_dict(),
            "link": self.link.stats.as_dict(),
            "detections": self.source.stats.as_dict(),
            "fsm": self.fsm.status(),
        }
        if self.targets:
            out["target_misses_m"] = [round(self.drum_miss_m(t), 4) for t in self.targets]
        if self.coverage is not None:
            out["coverage"] = self.coverage.summary()
        return out
