"""Mode and state machine: what the robot is doing, and why it stopped.

**Both evaluation paradigms live here, behind a config switch, and neither is
resolved.** PRD v5 has vision setting sweep *speed* over a coverage path; the
advisor on 5 August described vision *steering* the robot to a thrown nail, and
approved coverage in the same meeting. There is no ruling. CLAUDE.md section 11
is explicit that this must not be silently decided in code, so
``mission.mode`` picks one at run time and the other branch stays built and
tested. Do not delete a branch to tidy up.

The safety rules from CLAUDE.md section 6 are checked before any mode logic
runs, in this order, and each of them commands a stop:

* the firmware reports a fault or refuses motion;
* the watchdog has fired, which means the link went quiet and recovery needs an
  explicit re-enable rather than a resumed velocity;
* the detection heartbeat is older than ``vision_timeout_ms`` -- vision is dead.
  Note that an *empty* detection message is not silence: it means the floor is
  clear. Only the absence of messages counts;
* the obstacle flag is set, if the config says to care.

The transitions and the reason for each are strings on the returned
:class:`~fodnav.control.Command`, and every one of them goes in the run log.
A robot that stopped and cannot say why is a robot that will do it again at the
exam.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum

from .config import Config
from .control import Command, Gains, MotionLimits, PathFollower
from .frames import Pose2D
from .ground import GroundProjector
from .link.detections import DetectionFrame, select_targets
from .link.esp32 import Telemetry
from .planner.boustrophedon import Rect, plan, row_spacing, swath_width_from_config
from .servo import ServoGains, ServoGeometry, ServoState, VisualServo
from .target import TargetTracker, TrackerParams

__all__ = ["Mode", "State", "NavInputs", "NavFsm"]


class Mode(Enum):
    """Which paradigm is running. CLAUDE.md section 11: unresolved."""

    COVERAGE = "coverage"
    TARGET = "target"
    IDLE = "idle"


class State(Enum):
    INIT = "init"
    SEARCH = "search"          # target mode, nothing in view
    SERVO = "servo"            # target mode, driving to something
    COVER = "cover"            # coverage mode, following the planned path
    STOPPED = "stopped"        # a safety rule is holding it still
    DONE = "done"              # the mission finished normally


@dataclass
class NavInputs:
    """Everything the FSM is allowed to look at, gathered once per tick."""

    now: float                                  # monotonic seconds
    odom_pose: Pose2D
    frames: list[DetectionFrame] = field(default_factory=list)
    vision_age_s: float = float("inf")
    telemetry: Telemetry | None = None


class NavFsm:
    def __init__(
        self,
        robot: Config,
        nav: Config,
        projector: GroundProjector | None = None,
        arena: Rect | None = None,
    ) -> None:
        self.robot = robot
        self.nav = nav
        self.projector = projector
        self.mode = Mode(nav.get("mission.mode"))
        self.state = State.INIT
        self.reason = ""

        self.limits = MotionLimits.from_config(robot)
        self.gains = Gains.from_config(nav)
        self.tracker = TargetTracker(TrackerParams.from_config(nav))
        self.vision_timeout_s = nav.get("loop.vision_timeout_s")
        self.max_valid_range_m = robot.get("camera.fov_far_limit_m")

        self._target_classes = nav.get("detections.target_classes")
        self._ignore_classes = nav.get("detections.ignore_classes")
        self._drop_conf = nav.get("detections.drop_conf")

        self.servo: VisualServo | None = None
        self.follower: PathFollower | None = None
        self.path: list[tuple[float, float]] = []
        self.swath_m = 0.0
        self.swath_source = ""
        self.spacing_m = 0.0
        self.arena = arena
        self.n_targets_collected = 0
        self.n_frames_seen = 0
        self.n_projection_rejects = 0

        if self.mode is Mode.TARGET:
            self.servo = VisualServo(
                ServoGains.from_config(nav), self.limits, ServoGeometry.from_config(robot, nav)
            )
        elif self.mode is Mode.COVERAGE:
            self._plan_coverage()

    # -- setup ----------------------------------------------------------

    def _plan_coverage(self) -> None:
        if self.arena is None:
            self.arena = Rect(
                self.nav.get("mission.arena.x_min"),
                self.nav.get("mission.arena.y_min"),
                self.nav.get("mission.arena.x_max"),
                self.nav.get("mission.arena.y_max"),
            )
        (margin,) = self.robot.require(
            "chassis.planner_margin_m", needed_by="the coverage planner"
        )
        inner = self.arena.inset(margin)
        self.swath_m, self.swath_source = swath_width_from_config(self.robot, self.nav)
        overlap = self.nav.get("planner.overlap")
        self.spacing_m = row_spacing(self.swath_m, overlap)
        self.path = plan(
            inner,
            self.swath_m,
            overlap,
            self.nav.get("planner.start_corner"),
            self.nav.get("planner.orientation"),
        )
        self.follower = PathFollower(self.path, self.gains, self.limits)

    @property
    def magnet_should_be_on(self) -> bool:
        """Whether the drum motor should be running right now."""
        if not self.nav.get("mission.magnet_on"):
            return False
        return self.state in (State.SEARCH, State.SERVO, State.COVER)

    # -- the tick -------------------------------------------------------

    def update(self, inputs: NavInputs) -> Command:
        self._ingest(inputs)
        cmd = self._safety(inputs)
        if cmd is not None:
            return cmd
        if self.mode is Mode.IDLE:
            return self._to(State.STOPPED, Command(reason="mode is idle"))
        if self.mode is Mode.TARGET:
            return self._target_mode(inputs)
        return self._coverage_mode(inputs)

    def _ingest(self, inputs: NavInputs) -> None:
        """Detections in, tracks out. The only place perception enters."""
        for frame in inputs.frames:
            self.n_frames_seen += 1
            observations = []
            for det in select_targets(
                frame, self._target_classes, self._ignore_classes, self._drop_conf
            ):
                if self.projector is None:
                    continue
                # Assert the resolution matches what the homography was
                # calibrated at. If it does not, every projection is silently
                # wrong, so this is allowed to raise into the control loop --
                # which stops the robot, which is the correct outcome.
                self.projector.check_frame_size(frame.frame_size)
                point = self.projector.project_detection(det)
                if point is None:
                    self.n_projection_rejects += 1
                    continue
                observations.append((point, det.conf, det.cls))
            self.tracker.update(observations, inputs.now, inputs.odom_pose)

    def _safety(self, inputs: NavInputs) -> Command | None:
        t = inputs.telemetry
        if t is not None:
            if t.flags.fault_state or t.flags.driver_fault:
                return self._to(
                    State.STOPPED,
                    Command(reason=f"firmware fault: {t.flags.describe()}"),
                )
            if t.flags.watchdog_fired:
                return self._to(
                    State.STOPPED,
                    Command(reason="watchdog fired: the link went quiet, re-enable explicitly"),
                )
            if t.flags.obstacle and self.nav.get("safety.stop_on_obstacle"):
                return self._to(State.STOPPED, Command(reason="obstacle sensor blocked"))
            if t.flags.battery_low and self.nav.get("safety.stop_on_battery_low"):
                return self._to(State.STOPPED, Command(reason="battery low"))

        if inputs.vision_age_s > self.vision_timeout_s:
            # No message at all for this long. An empty dets list would have
            # reset this: "the floor is clear" is a report, silence is not.
            age = "never" if inputs.vision_age_s == float("inf") else f"{inputs.vision_age_s:.2f} s"
            return self._to(
                State.STOPPED,
                Command(reason=f"vision heartbeat lost ({age} since the last message)"),
            )
        return None

    # -- target mode ----------------------------------------------------

    def _target_mode(self, inputs: NavInputs) -> Command:
        assert self.servo is not None
        best = self.tracker.best(max_range_m=self.max_valid_range_m)

        if self.servo.finished:
            # Arrived, or lost it. Either way go back to watching: "throw
            # again, robot follows" is the demo, so a completed run is not the
            # end of the mission.
            if self.servo.state is ServoState.ARRIVED:
                self.n_targets_collected += 1
            self.servo.reset()
            self.tracker.reset()
            return self._to(State.SEARCH, Command(reason="ready for the next target"))

        target = best.predict_base(inputs.odom_pose) if best is not None else None
        if target is None and self.servo.state in (ServoState.IDLE,):
            return self._to(State.SEARCH, self._search())

        cmd = self.servo.update(target, inputs.odom_pose, inputs.now)
        state = State.SERVO if not cmd.done else State.SEARCH
        return self._to(state, cmd)

    def _search(self) -> Command:
        """What to do with nothing in view.

        ``hold`` is the default and it is a deliberate non-decision: a search
        pattern is a behaviour nobody asked for, and the demo throws the nail
        into the field of view. ``scan`` exists for the day it does not.
        """
        if self.nav.get("mission.search") == "scan":
            omega = self.nav.get("mission.scan_omega_frac") * self.limits.omega_max
            return Command(0.0, omega, reason="scanning for a target")
        return Command(0.0, 0.0, reason="holding, no target in view")

    # -- coverage mode --------------------------------------------------

    def _coverage_mode(self, inputs: NavInputs) -> Command:
        assert self.follower is not None
        cmd = self.follower.update(inputs.odom_pose)
        if cmd.done:
            return self._to(State.DONE, Command(reason="coverage path complete"))
        # Detections are tracked and logged during a coverage sweep but are not
        # chased. Whether a sweep should divert to a detection is the
        # unresolved §11 question, and inventing a hybrid here would answer it
        # by accident.
        return self._to(State.COVER, cmd)

    # -- bookkeeping ----------------------------------------------------

    def _to(self, state: State, cmd: Command) -> Command:
        self.state = state
        self.reason = cmd.reason
        return cmd

    def status(self) -> dict:
        """A flat snapshot for the run log."""
        out = {
            "mode": self.mode.value,
            "state": self.state.value,
            "reason": self.reason,
            "frames_seen": self.n_frames_seen,
            "projection_rejects": self.n_projection_rejects,
            "tracks": len(self.tracker.tracks),
            "confirmed": len(self.tracker.confirmed()),
        }
        if self.mode is Mode.TARGET and self.servo is not None:
            out["servo_state"] = self.servo.state.value
            out["targets_collected"] = self.n_targets_collected
        if self.mode is Mode.COVERAGE and self.follower is not None:
            out["swath_m"] = self.swath_m
            out["swath_source"] = self.swath_source
            out["row_spacing_m"] = self.spacing_m
            out["waypoints"] = len(self.path)
            out["path_fraction_done"] = round(self.follower.fraction_done, 4)
        return out
