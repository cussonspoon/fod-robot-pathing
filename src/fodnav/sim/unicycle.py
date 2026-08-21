"""Kinematic chassis simulator with the odometry errors that actually matter.

Not a luxury. It is how the planner and both controllers get written and tested
before the chassis exists, and how they stay tested afterwards (CLAUDE.md
section 10).

The error sources, in order of how much damage they do:

**Per-wheel scale mismatch.** One wheel's effective rolling radius differs from
the other's by a fraction of a percent. The wheel turns exactly as commanded --
the encoder is a perfect counter of *rotations* -- but it covers more ground per
rotation than the Pi's ``wheel_radius_m`` says it does. The result is a robot
that curves while believing it is driving straight, and the belief is smooth,
confident and wrong. This is the dominant real-world odometry error on a
differential drive and it is the reason a 4 m dead-reckoned leg arrives beside
the nail rather than on it. Without it the sim will cheerfully report that
``move_to`` over 4 m is fine, and the hardware will disagree.

**Tick quantisation.** The encoder reports whole ticks. Sub-tick motion is not
lost, it is deferred -- the accumulator carries it -- which is what a real
quadrature counter does.

**Gaussian tick noise.** Modelled as noise on each step's *delta*, so it
accumulates as a random walk the way miscounts do rather than as jitter that
averages out. Deliberately small: at the shipped defaults it contributes
millimetres over a run where the scale mismatch contributes centimetres, and
seeing that difference in the sim is the point.

**A constant heading bias.** Unmodelled yaw with no encoder evidence at all --
a dragging caster, a floor camber. Odometry cannot see it by construction.

**Slip events.** The wheel turns, the ground does not move. The encoder counts
anyway, so odometry over-reports distance.

**Stiction.** A command below the measured deadband does not start the robot
from rest, though it will keep it moving once moving. This is what makes a
position controller creep toward a waypoint at a velocity too small to
overcome static friction and sit there, believing it is still approaching.
"""

from __future__ import annotations

import math
import random
from dataclasses import dataclass

from ..config import Config
from ..frames import Pose2D
from ..odom import TICK_MODULUS, _integrate

__all__ = ["SimParams", "UnicycleSim"]


@dataclass(frozen=True)
class SimParams:
    """Error sources. All sim-only; none of this describes real hardware."""

    wheel_scale_left: float = 1.0
    wheel_scale_right: float = 1.0
    tick_noise_std: float = 0.0
    heading_bias_radps: float = 0.0
    slip_prob: float = 0.0
    slip_fraction: float = 0.3
    seed: int = 0
    apply_deadband: bool = True

    @classmethod
    def from_config(cls, nav: Config, **over) -> "SimParams":
        kw = dict(
            wheel_scale_left=nav.get("sim.wheel_scale_left"),
            wheel_scale_right=nav.get("sim.wheel_scale_right"),
            tick_noise_std=nav.get("sim.tick_noise_std"),
            heading_bias_radps=nav.get("sim.heading_bias_radps"),
            slip_prob=nav.get("sim.slip_prob"),
            slip_fraction=nav.get("sim.slip_fraction"),
            seed=nav.get("sim.seed"),
        )
        kw.update(over)
        return cls(**kw)

    @classmethod
    def perfect(cls) -> "SimParams":
        """A robot with no errors at all. For testing the *controller* in
        isolation -- never for concluding that a controller works."""
        return cls(apply_deadband=False)


class UnicycleSim:
    """The chassis, the wheels and the encoders. Not the firmware.

    Clamping, the watchdog and the wire protocol live in
    :mod:`fodnav.sim.firmware`, which drives this. Keeping them apart means the
    physics here stays honest about what it is: a kinematic model with error
    sources, and no opinions.
    """

    def __init__(
        self,
        robot: Config,
        params: SimParams | None = None,
        pose: Pose2D = Pose2D(),
    ) -> None:
        self.wheel_radius_m, self.track_width_m, self.ticks_per_rev = robot.require(
            "drive.wheel_radius_m",
            "drive.track_width_m",
            "drive.ticks_per_rev",
            needed_by="the chassis simulator",
        )
        self.v_min, self.omega_min = robot.require(
            "drive.v_min_mps", "drive.omega_min_radps", needed_by="the simulated friction deadband"
        )
        self.params = params or SimParams()
        self._rng = random.Random(self.params.seed)

        self.true_pose = pose
        self.t = 0.0
        self.v_true = 0.0
        self.omega_true = 0.0
        # Float accumulators; the reported counters are these, truncated. Real
        # encoders do not lose sub-tick motion either.
        self._acc_l = 0.0
        self._acc_r = 0.0
        self.ticks_l = 0
        self.ticks_r = 0
        # Whether the wheels turned last step. Tracked from wheel motion rather
        # than from the chassis velocity, so that a disturbance term can never
        # be what releases the stiction.
        self._wheels_turning = False
        self.n_slips = 0
        self.distance_m = 0.0

    # -- setup ----------------------------------------------------------

    def seed_ticks(self, ticks_l: int, ticks_r: int) -> None:
        """Start the counters somewhere specific -- e.g. just short of the
        int32 wrap, so a test can drive across it."""
        self._acc_l = float(ticks_l)
        self._acc_r = float(ticks_r)
        self.ticks_l = _wrap_int32(int(ticks_l))
        self.ticks_r = _wrap_int32(int(ticks_r))

    def reset(self, pose: Pose2D = Pose2D()) -> None:
        self.true_pose = pose
        self.v_true = self.omega_true = 0.0
        self._wheels_turning = False

    # -- physics --------------------------------------------------------

    def step(self, v_cmd: float, omega_cmd: float, dt: float) -> tuple[int, int]:
        """Advance by ``dt`` under a velocity command. Returns the tick counters.

        The command is taken as achieved instantly: there is no motor lag here.
        That is a real omission, and it is a deliberate one -- what breaks
        dead reckoning is systematic geometry error, not first-order lag, and a
        motor time constant nobody has measured would be a number invented to
        look thorough.
        """
        if dt <= 0:
            raise ValueError(f"dt must be positive, got {dt}")

        v, omega = float(v_cmd), float(omega_cmd)

        # Stiction. From rest, a command below both deadbands does nothing at
        # all; once moving, the deadband is gone. Model it this way round or
        # the sim will never reproduce "the robot sat there thinking it was
        # still approaching the waypoint".
        if self.params.apply_deadband and not self._wheels_turning:
            if abs(v) < self.v_min and abs(omega) < self.omega_min:
                v = omega = 0.0

        # Command to wheels, through the *nominal* geometry -- which is what
        # the firmware would use.
        half = 0.5 * self.track_width_m
        vl_cmd = v - omega * half
        vr_cmd = v + omega * half

        # The wheels turn as commanded. What differs is how much ground each
        # covers per turn.
        vl_true = vl_cmd * self.params.wheel_scale_left
        vr_true = vr_cmd * self.params.wheel_scale_right

        # Slip: the wheel keeps turning (so the encoder keeps counting) while
        # the ground motion is lost.
        if self.params.slip_prob > 0.0 and self._rng.random() < self.params.slip_prob:
            self.n_slips += 1
            if self._rng.random() < 0.5:
                vl_true *= 1.0 - self.params.slip_fraction
            else:
                vr_true *= 1.0 - self.params.slip_fraction

        self._wheels_turning = abs(vl_cmd) > 1e-9 or abs(vr_cmd) > 1e-9

        ds = 0.5 * (vl_true + vr_true) * dt
        dtheta = (vr_true - vl_true) / self.track_width_m * dt
        # Invisible to the encoders, and only while the robot is actually
        # driving: a parked robot does not slowly rotate, and a disturbance
        # term must never be what breaks the chassis free of stiction.
        if self._wheels_turning:
            dtheta += self.params.heading_bias_radps * dt

        self.true_pose = _integrate(self.true_pose, ds, dtheta)
        self.v_true = ds / dt
        self.omega_true = dtheta / dt
        self.distance_m += abs(ds)
        self.t += dt

        # Encoders count wheel rotation, so they follow the commanded wheel
        # speed and know nothing about scale error or slip. That asymmetry is
        # the entire reason odometry drifts.
        ticks_per_metre = self.ticks_per_rev / (2.0 * math.pi * self.wheel_radius_m)
        dl = vl_cmd * dt * ticks_per_metre
        dr = vr_cmd * dt * ticks_per_metre
        if self.params.tick_noise_std > 0.0:
            dl += self._rng.gauss(0.0, self.params.tick_noise_std)
            dr += self._rng.gauss(0.0, self.params.tick_noise_std)
        self._acc_l += dl
        self._acc_r += dr
        self.ticks_l = _wrap_int32(math.floor(self._acc_l))
        self.ticks_r = _wrap_int32(math.floor(self._acc_r))
        return (self.ticks_l, self.ticks_r)

    # -- what the firmware would report ---------------------------------

    @property
    def ticks(self) -> tuple[int, int]:
        return (self.ticks_l, self.ticks_r)


def _wrap_int32(v: int) -> int:
    """Fold an integer into the signed 32-bit range, the way a counter does."""
    v &= TICK_MODULUS - 1
    return v - TICK_MODULUS if v >= (1 << 31) else v
