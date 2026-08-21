"""Config loading and validation. Nav owns the schema; Teemy owns the values.

Two files, two owners, one loader.

``config/robot.yaml``
    Physical constants. Measured on the hardware by Teemy, who edits and
    commits it directly -- its git history is the calibration record. Nav
    never writes it and never re-types a number out of it into a second
    place. Its shape is fixed by ``docs/HARDWARE.md`` section 8, verbatim,
    nulls included, because that document is what Teemy fills in.

``config/nav.yaml``
    Gains, tolerances, rates and mode switches. Tuned, not measured. Nav owns
    these and they deliberately do *not* live in ``robot.yaml``: mixing them in
    would make Teemy's file something other than a calibration record, and
    would make the file stop matching the block printed in HARDWARE.md section 8.

A missing measurement is a ``null``, and a ``null`` that a code path actually
needs raises here, at the point of use, naming the field and the measurement
procedure. It is never silently replaced with something plausible. CLAUDE.md
section 12: a plausible placeholder that becomes load-bearing is worse than a
crash.

There is a third file, ``config/sim_robot.yaml``. It conforms to the same
schema as ``robot.yaml`` and is full of invented numbers, which is legitimate
only because it describes a robot that does not exist. Nothing in it is a
measurement of anything. It exists so the whole stack runs on a laptop.
"""

from __future__ import annotations

import math
import os
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Callable, Iterable, Mapping

import yaml

__all__ = [
    "Config",
    "ConfigError",
    "ConfigSchemaError",
    "MissingValueError",
    "load_robot_config",
    "load_nav_config",
    "find_config_dir",
    "ROBOT_SCHEMA",
    "NAV_SCHEMA",
]


class ConfigError(Exception):
    """Base for every config failure."""


class ConfigSchemaError(ConfigError):
    """The file, or a lookup against it, disagrees with the schema.

    Raised at load for an unknown key, a wrong type or an out-of-range value,
    and at lookup for a dotted path nav asked for that the schema does not
    define. The latter is a bug in nav, not a missing measurement, and the two
    are deliberately different exceptions.
    """


class MissingValueError(ConfigError):
    """A value a code path needs is ``null``, or absent from the file.

    This is the intended failure mode of an uncalibrated robot, not an
    accident. Read the message: it names the measurement and the procedure.
    """


# ---------------------------------------------------------------------------
# schema
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class Field:
    """One leaf in a config file, and everything needed to complain about it."""

    kind: type
    what: str
    unit: str = ""
    where: str = ""  # where the number comes from: a HARDWARE.md section, or "tuned"
    minimum: float | None = None
    maximum: float | None = None
    choices: tuple | None = None

    def describe(self) -> str:
        unit = f" [{self.unit}]" if self.unit else ""
        return f"{self.what}{unit}"

    def coerce(self, path: str, value: Any, source: str) -> Any:
        if self.kind is float:
            if isinstance(value, bool) or not isinstance(value, (int, float)):
                raise ConfigSchemaError(
                    f"{source}: {path} must be a number, got {value!r}"
                )
            value = float(value)
            if not math.isfinite(value):
                raise ConfigSchemaError(f"{source}: {path} must be finite, got {value!r}")
        elif self.kind is int:
            if isinstance(value, bool) or not isinstance(value, int):
                raise ConfigSchemaError(
                    f"{source}: {path} must be a whole number, got {value!r}"
                )
        elif self.kind is bool:
            if not isinstance(value, bool):
                raise ConfigSchemaError(f"{source}: {path} must be true or false, got {value!r}")
        elif self.kind is str:
            if not isinstance(value, str):
                raise ConfigSchemaError(f"{source}: {path} must be a string, got {value!r}")
        elif self.kind is list:
            if not isinstance(value, list):
                raise ConfigSchemaError(f"{source}: {path} must be a list, got {value!r}")

        if self.choices is not None and value not in self.choices:
            raise ConfigSchemaError(
                f"{source}: {path} must be one of {list(self.choices)}, got {value!r}"
            )
        if self.minimum is not None and value < self.minimum:
            raise ConfigSchemaError(
                f"{source}: {path} = {value!r} is below the plausible minimum "
                f"{self.minimum} ({self.describe()}). If that is genuinely the "
                f"measurement, the schema in config.py is wrong -- fix it there, "
                f"not by rounding the measurement."
            )
        if self.maximum is not None and value > self.maximum:
            raise ConfigSchemaError(
                f"{source}: {path} = {value!r} is above the plausible maximum "
                f"{self.maximum} ({self.describe()}). Check the units: metres, "
                f"radians, seconds. If the measurement is right, fix the schema."
            )
        return value


@dataclass(frozen=True)
class Derived:
    """A value nav computes from a stored one, so nobody stores it twice.

    Unit conversion lives here: ``robot.yaml`` records tilt in degrees because
    that is what an inclinometer reads, and this is the boundary where degrees
    stop (CLAUDE.md section 3).
    """

    source: str
    fn: Callable[[Any], Any]
    what: str
    unit: str = ""

    def describe(self) -> str:
        unit = f" [{self.unit}]" if self.unit else ""
        return f"{self.what}{unit}"


HW = "docs/HARDWARE.md"

#: Schema for ``config/robot.yaml`` (and ``config/sim_robot.yaml``, which must
#: stay schema-identical to it so that swapping one for the other is a
#: one-flag change and exercises exactly the same code).
ROBOT_SCHEMA: dict[str, Field] = {
    # -- drive geometry: the entire basis of odometry ----------------------
    "drive.wheel_radius_m": Field(
        float, "effective rolling wheel radius, loaded", "m", f"{HW} §2.1 (10-revolution roll test)",
        minimum=0.005, maximum=0.5,
    ),
    "drive.track_width_m": Field(
        float, "distance between the wheel contact patches", "m",
        f"{HW} §2.2 (tape, then corrected by the 10-rotation spin test)",
        minimum=0.02, maximum=2.0,
    ),
    "drive.ticks_per_rev": Field(
        float, "encoder ticks per wheel revolution, after gearbox, including quadrature", "ticks",
        f"{HW} §2.3 (10 hand revolutions, verified -- not CPR x gear ratio)",
        minimum=1.0, maximum=1e6,
    ),
    "drive.quadrature": Field(
        int, "quadrature decoding mode the firmware uses", "x", f"{HW} §2.3",
        choices=(1, 2, 4),
    ),
    "drive.v_max_mps": Field(
        float, "highest forward speed the wheels actually achieve", "m/s", f"{HW} §2.5 (measured, not spec)",
        minimum=0.01, maximum=10.0,
    ),
    "drive.v_min_mps": Field(
        float, "lowest forward speed that produces reliable motion (friction deadband)", "m/s",
        f"{HW} §2.5", minimum=0.0, maximum=10.0,
    ),
    "drive.omega_max_radps": Field(
        float, "highest in-place spin rate", "rad/s", f"{HW} §2.5", minimum=0.01, maximum=50.0,
    ),
    "drive.omega_min_radps": Field(
        float, "lowest reliable spin rate", "rad/s", f"{HW} §2.5", minimum=0.0, maximum=50.0,
    ),
    "drive.stopping_distance_m": Field(
        float, "distance travelled after an S command at v_max", "m", f"{HW} §2.6",
        minimum=0.0, maximum=5.0,
    ),
    # -- camera mount: void the moment the mount moves ---------------------
    "camera.height_m": Field(
        float, "optical centre above the floor", "m", f"{HW} §3", minimum=0.01, maximum=2.0,
    ),
    "camera.tilt_deg": Field(
        float, "down-tilt from horizontal, positive downward", "deg",
        f"{HW} §3 (inclinometer on a flat face of the mount). Target band 10-25.",
        minimum=0.0, maximum=89.0,
    ),
    "camera.offset_x_m": Field(
        float, "forward of the axle midpoint, positive forward", "m", f"{HW} §3",
        minimum=-1.0, maximum=1.0,
    ),
    "camera.offset_y_m": Field(
        float, "left of the centreline; should be 0, measured anyway", "m", f"{HW} §3",
        minimum=-1.0, maximum=1.0,
    ),
    "camera.roll_deg": Field(
        float, "rotation about the optical axis; should be 0, a twist shears every projection", "deg",
        f"{HW} §3", minimum=-45.0, maximum=45.0,
    ),
    "camera.capture_width_px": Field(
        int, "captured frame width -- not the 480 network input", "px", f"{HW} §3",
        minimum=16, maximum=20000,
    ),
    "camera.capture_height_px": Field(
        int, "captured frame height -- not the 480 network input", "px", f"{HW} §3",
        minimum=16, maximum=20000,
    ),
    "camera.fov_near_limit_m": Field(
        float, "closest floor distance still in frame -- sizes the terminal blind leg", "m",
        f"{HW} §3.1", minimum=0.0, maximum=3.0,
    ),
    "camera.fov_far_limit_m": Field(
        float, "furthest distance the detector still fires reliably", "m", f"{HW} §3.1",
        minimum=0.05, maximum=20.0,
    ),
    "camera.fov_width_at_lookahead_m": Field(
        float, "ground width of the frame at 0.3 m -- candidate swath width", "m", f"{HW} §3.1",
        minimum=0.01, maximum=10.0,
    ),
    # -- collection drum ---------------------------------------------------
    "drum.width_m": Field(
        float, "mechanical swept width of the drum", "m", f"{HW} §4", minimum=0.01, maximum=2.0,
    ),
    "drum.capture_width_m": Field(
        float, "width over which a screw is actually lifted -- measured, not assumed", "m",
        f"{HW} §4 (screws at 2 cm spacing, push over once, see which came up)",
        minimum=0.01, maximum=2.0,
    ),
    "drum.offset_x_m": Field(
        float, "drum position relative to the axle midpoint, signed, positive forward", "m",
        f"{HW} §4 -- sets the terminal blind-leg overshoot", minimum=-1.0, maximum=1.0,
    ),
    "drum.clearance_mm": Field(
        float, "magnet face to floor", "mm", f"{HW} §4", minimum=0.0, maximum=200.0,
    ),
    # -- obstacle sensing --------------------------------------------------
    "obstacle.tof_height_m": Field(
        float, "ToF sensor height above the floor", "m", f"{HW} §5", minimum=0.0, maximum=2.0,
    ),
    "obstacle.tof_offset_x_m": Field(
        float, "ToF forward offset from the axle midpoint", "m", f"{HW} §5", minimum=-1.0, maximum=2.0,
    ),
    "obstacle.tof_threshold_m": Field(
        float, "distance below which the firmware sets flag bit 3", "m", f"{HW} §5",
        minimum=0.0, maximum=10.0,
    ),
    # -- chassis -----------------------------------------------------------
    "chassis.mass_kg": Field(float, "total loaded mass", "kg", f"{HW} §5", minimum=0.1, maximum=500.0),
    "chassis.footprint_length_m": Field(
        float, "footprint length", "m", f"{HW} §5", minimum=0.05, maximum=5.0,
    ),
    "chassis.footprint_width_m": Field(
        float, "footprint width", "m", f"{HW} §5", minimum=0.05, maximum=5.0,
    ),
    "chassis.planner_margin_m": Field(
        float, "half footprint plus clearance; the planner insets the arena by this", "m",
        f"{HW} §5", minimum=0.0, maximum=2.0,
    ),
    # -- power -------------------------------------------------------------
    "power.nominal_v": Field(float, "battery nominal voltage", "V", f"{HW} §5", minimum=1.0, maximum=100.0),
    "power.warn_v": Field(
        float, "low-battery warning threshold, firmware flag bit 6", "V", f"{HW} §5",
        minimum=1.0, maximum=100.0,
    ),
    "power.cutoff_v": Field(float, "cutoff voltage", "V", f"{HW} §5", minimum=1.0, maximum=100.0),
    # -- link --------------------------------------------------------------
    "link.port": Field(
        str, "serial device path", "", f"{HW} §6 -- /dev/serial/by-id/..., never /dev/ttyACM0",
    ),
    "link.baud": Field(
        int, "serial baud rate", "baud", "docs/protocol.md §1 fixes this at 115200",
        choices=(9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600),
    ),
    "link.proto_version": Field(
        int, "protocol version the firmware implements", "", f"{HW} §6", minimum=0, maximum=255,
    ),
}

ROBOT_DERIVED: dict[str, Derived] = {
    "camera.tilt_rad": Derived("camera.tilt_deg", math.radians, "camera down-tilt", "rad"),
    "camera.roll_rad": Derived("camera.roll_deg", math.radians, "camera roll", "rad"),
    "drum.clearance_m": Derived("drum.clearance_mm", lambda v: v / 1000.0, "magnet face to floor", "m"),
}

#: Schema for ``config/nav.yaml``. Everything here is a tuning choice, so
#: everything here has a real default -- unlike ``robot.yaml``, where a
#: default would be a lie about the hardware.
NAV_SCHEMA: dict[str, Field] = {
    # -- control loop ------------------------------------------------------
    "loop.rate_hz": Field(
        float, "control loop rate; protocol.md §6 requires V at a fixed 50 Hz", "Hz",
        "docs/protocol.md §6", minimum=1.0, maximum=1000.0,
    ),
    "loop.vision_timeout_ms": Field(
        float, "no detection message for this long means vision is dead -> stop", "ms",
        "tuned; must exceed several publish periods at ~30 Hz", minimum=1.0, maximum=60000.0,
    ),
    "loop.max_stall_ms": Field(
        float, "a loop iteration longer than this is logged loudly", "ms", "tuned",
        minimum=1.0, maximum=10000.0,
    ),
    # -- mission -----------------------------------------------------------
    "mission.mode": Field(
        str, "which paradigm runs; CLAUDE.md §11 is unresolved, so this is a switch", "",
        "unresolved between the team and the advisor -- do not hardcode either",
        choices=("coverage", "target", "idle"),
    ),
    "mission.search": Field(
        str, "what target mode does with no target in view", "",
        "hold is the default because scanning is an invented behaviour",
        choices=("hold", "scan"),
    ),
    "mission.scan_omega_frac": Field(
        float, "scan spin rate as a fraction of the measured omega_max", "", "tuned",
        minimum=0.0, maximum=1.0,
    ),
    "mission.arena.x_min": Field(float, "arena rectangle, world metres", "m", "mission choice",
                                 minimum=-1000.0, maximum=1000.0),
    "mission.arena.y_min": Field(float, "arena rectangle, world metres", "m", "mission choice",
                                 minimum=-1000.0, maximum=1000.0),
    "mission.arena.x_max": Field(float, "arena rectangle, world metres", "m", "mission choice",
                                 minimum=-1000.0, maximum=1000.0),
    "mission.arena.y_max": Field(float, "arena rectangle, world metres", "m", "mission choice",
                                 minimum=-1000.0, maximum=1000.0),
    "mission.magnet_on": Field(
        bool, "run the magnet drum while the mission is active", "", "mission choice",
    ),
    # -- move_to / pure pursuit -------------------------------------------
    "control.goal_radius_m": Field(
        float, "a waypoint counts as reached inside this radius", "m", "tuned",
        minimum=0.005, maximum=1.0,
    ),
    "control.heading_tolerance_rad": Field(
        float, "turn-in-place finishes inside this heading error", "rad", "tuned",
        minimum=0.001, maximum=1.5,
    ),
    "control.turn_in_place_rad": Field(
        float, "beyond this heading error, stop translating and turn on the spot", "rad", "tuned",
        minimum=0.01, maximum=math.pi,
    ),
    "control.k_omega": Field(
        float, "heading P gain, rad/s per rad", "1/s", "tuned in sim, re-tune on hardware",
        minimum=0.0, maximum=50.0,
    ),
    "control.k_v": Field(
        float, "approach P gain, m/s per m", "1/s", "tuned in sim, re-tune on hardware",
        minimum=0.0, maximum=50.0,
    ),
    "control.cruise_fraction": Field(
        float, "cruise speed as a fraction of the measured v_max -- dimensionless on purpose, "
        "so nav never states a speed in m/s that only the hardware knows", "",
        "tuned", minimum=0.0, maximum=1.0,
    ),
    "control.lookahead_m": Field(
        float, "pure-pursuit lookahead along the path", "m", "tuned in sim",
        minimum=0.02, maximum=5.0,
    ),
    "control.slowdown_bearing_rad": Field(
        float, "forward speed is scaled by cos(bearing) and zeroed past this", "rad", "tuned",
        minimum=0.05, maximum=math.pi,
    ),
    # -- visual servo ------------------------------------------------------
    "servo.k_omega": Field(float, "bearing P gain", "1/s", "tuned in sim", minimum=0.0, maximum=50.0),
    "servo.k_v": Field(float, "range P gain", "1/s", "tuned in sim", minimum=0.0, maximum=50.0),
    "servo.turn_first_rad": Field(
        float, "beyond this bearing, turn before driving", "rad", "tuned",
        minimum=0.05, maximum=math.pi,
    ),
    "servo.blind_leg_extra_m": Field(
        float, "safety margin added to the open-loop terminal leg", "m", "tuned",
        minimum=0.0, maximum=0.5,
    ),
    "servo.blind_leg_min_v_frac": Field(
        float, "speed floor for the terminal leg, as a fraction of the cruise speed", "",
        "tuned; stops the proportional tail creeping for seconds over the last few cm",
        minimum=0.0, maximum=1.0,
    ),
    "servo.blind_leg_timeout_s": Field(
        float, "abort the open-loop terminal leg after this long", "s", "tuned",
        minimum=0.1, maximum=60.0,
    ),
    "servo.lost_timeout_s": Field(
        float, "no target for this long while servoing means the target is lost", "s", "tuned",
        minimum=0.05, maximum=60.0,
    ),
    # -- detections --------------------------------------------------------
    "detections.topic": Field(
        str, "MQTT topic the vision process publishes on", "", "CLAUDE.md §8",
    ),
    "detections.broker_host": Field(str, "MQTT broker host", "", "loopback on the Pi"),
    "detections.broker_port": Field(int, "MQTT broker port", "", "", minimum=1, maximum=65535),
    "detections.acquire_conf": Field(
        float, "confidence needed to start trusting a detection", "", "tuned; hysteresis pair",
        minimum=0.0, maximum=1.0,
    ),
    "detections.drop_conf": Field(
        float, "confidence below which an already-tracked target is dropped", "",
        "tuned; hysteresis pair", minimum=0.0, maximum=1.0,
    ),
    "detections.target_classes": Field(
        list, "class names treated as one target class (CLAUDE.md §8: nail/screw/bolt)", "",
        "CLAUDE.md §8 -- localisation is solved, naming is not",
    ),
    "detections.ignore_classes": Field(
        list, "class names discarded on receipt", "", "CLAUDE.md §8 -- 'unknown' fires on furniture",
    ),
    "detections.assoc_max_jump_m": Field(
        float, "nearest-neighbour association gate in the base frame", "m", "tuned",
        minimum=0.01, maximum=5.0,
    ),
    "detections.min_hits": Field(
        int, "associated frames before a track is acted on", "frames", "tuned", minimum=1, maximum=100,
    ),
    "detections.max_misses": Field(
        int, "unassociated frames before a track is dropped", "frames", "tuned", minimum=1, maximum=1000,
    ),
    # -- planner -----------------------------------------------------------
    "planner.swath_source": Field(
        str, "which width a coverage row sweeps: the drum, or the camera footprint", "",
        "CLAUDE.md §9 -- depends on the unresolved evaluation question; logged per run",
        choices=("drum", "drum_capture", "camera"),
    ),
    "planner.overlap": Field(
        float, "fraction of the swath that adjacent rows share", "", "tuned",
        minimum=0.0, maximum=0.9,
    ),
    "planner.start_corner": Field(
        str, "corner of the arena rectangle the sweep starts at", "", "mission choice",
        choices=("sw", "se", "nw", "ne"),
    ),
    "planner.orientation": Field(
        str, "row direction; 'long' runs rows along the rectangle's long axis to minimise turns", "",
        "CLAUDE.md §9", choices=("long", "short", "x", "y"),
    ),
    # -- link timing (the port itself is Teemy's, in robot.yaml) -----------
    "link.boot_wait_s": Field(
        float, "wait after opening the port for the ESP32 to finish booting", "s",
        "docs/protocol.md §7 step 1", minimum=0.0, maximum=30.0,
    ),
    "link.handshake_timeout_s": Field(
        float, "how long to wait for the I reply to H", "s", "docs/protocol.md §7 step 2",
        minimum=0.1, maximum=30.0,
    ),
    # -- safety ------------------------------------------------------------
    "safety.stop_on_obstacle": Field(
        bool, "stop when the firmware reports flag bit 3 (ToF blocked)", "", "safety policy",
    ),
    "safety.stop_on_battery_low": Field(
        bool, "stop when the firmware reports flag bit 6", "", "safety policy",
    ),
    # -- simulator error sources (CLAUDE.md §10) ---------------------------
    "sim.dt_s": Field(float, "simulator integration step", "s", "sim only", minimum=1e-4, maximum=1.0),
    "sim.wheel_scale_left": Field(
        float, "left wheel effective-radius scale error -- the dominant real odometry error", "",
        "sim only; CLAUDE.md §10 insists this is modelled", minimum=0.5, maximum=1.5,
    ),
    "sim.wheel_scale_right": Field(
        float, "right wheel effective-radius scale error", "", "sim only", minimum=0.5, maximum=1.5,
    ),
    "sim.tick_noise_std": Field(
        float, "gaussian noise added per wheel per step, in ticks", "ticks", "sim only",
        minimum=0.0, maximum=100.0,
    ),
    "sim.heading_bias_radps": Field(
        float, "constant unmodelled yaw rate", "rad/s", "sim only", minimum=-1.0, maximum=1.0,
    ),
    "sim.slip_prob": Field(
        float, "per-step probability of a wheel slip event", "", "sim only", minimum=0.0, maximum=1.0,
    ),
    "sim.slip_fraction": Field(
        float, "fraction of a step's motion lost when a slip fires", "", "sim only",
        minimum=0.0, maximum=1.0,
    ),
    "sim.seed": Field(int, "RNG seed; a run is reproducible or it is anecdote", "", "sim only",
                      minimum=0, maximum=2**31 - 1),
}

NAV_DERIVED: dict[str, Derived] = {
    "loop.dt_s": Derived("loop.rate_hz", lambda v: 1.0 / v, "control loop period", "s"),
    "loop.vision_timeout_s": Derived(
        "loop.vision_timeout_ms", lambda v: v / 1000.0, "vision heartbeat timeout", "s"
    ),
}


# ---------------------------------------------------------------------------
# cross-field checks, run at load over whatever is present
# ---------------------------------------------------------------------------


def _pairwise_checks(schema_name: str) -> list[tuple[tuple[str, ...], Callable[..., str | None]]]:
    if schema_name == "robot":
        return [
            (("drive.v_min_mps", "drive.v_max_mps"),
             lambda lo, hi: None if lo < hi else
             "drive.v_min_mps must be below drive.v_max_mps"),
            (("drive.omega_min_radps", "drive.omega_max_radps"),
             lambda lo, hi: None if lo < hi else
             "drive.omega_min_radps must be below drive.omega_max_radps"),
            (("camera.fov_near_limit_m", "camera.fov_far_limit_m"),
             lambda near, far: None if near < far else
             "camera.fov_near_limit_m must be below camera.fov_far_limit_m"),
            (("power.cutoff_v", "power.warn_v"),
             lambda cut, warn: None if cut < warn else
             "power.cutoff_v must be below power.warn_v"),
            (("power.warn_v", "power.nominal_v"),
             lambda warn, nom: None if warn <= nom else
             "power.warn_v must not exceed power.nominal_v"),
            (("drive.wheel_radius_m", "drive.track_width_m"),
             lambda r, w: None if 2.0 * r < w * 3.0 else
             "drive.wheel_radius_m looks large against drive.track_width_m -- check units"),
        ]
    return [
        (("detections.drop_conf", "detections.acquire_conf"),
         lambda drop, acq: None if drop <= acq else
         "detections.drop_conf must not exceed detections.acquire_conf "
         "(they are a hysteresis pair: acquire high, drop low)"),
        (("control.heading_tolerance_rad", "control.turn_in_place_rad"),
         lambda tol, turn: None if tol < turn else
         "control.heading_tolerance_rad must be below control.turn_in_place_rad"),
    ]


# ---------------------------------------------------------------------------
# the loaded object
# ---------------------------------------------------------------------------


def _flatten(d: Mapping, prefix: str = "") -> dict[str, Any]:
    out: dict[str, Any] = {}
    for k, v in d.items():
        path = f"{prefix}{k}"
        if isinstance(v, dict):
            out.update(_flatten(v, f"{path}."))
        else:
            out[path] = v
    return out


class Config:
    """A validated config file. Nulls survive load and raise at first use."""

    def __init__(
        self,
        values: Mapping[str, Any],
        schema: Mapping[str, Field],
        derived: Mapping[str, Derived],
        source: str,
        name: str,
    ) -> None:
        self._values = dict(values)
        self._schema = schema
        self._derived = derived
        self.source = source
        self.name = name

    # -- lookup ---------------------------------------------------------

    def get(self, path: str) -> Any:
        """Return the value at a dotted path.

        Raises :class:`ConfigSchemaError` if nav asked for something the schema
        does not define (a bug here), and :class:`MissingValueError` if the
        value is null or absent from the file (a measurement that has not been
        taken).
        """
        if path in self._derived:
            d = self._derived[path]
            return d.fn(self.get(d.source))
        if path not in self._schema:
            raise ConfigSchemaError(self._unknown_path_message(path))
        value = self._values.get(path, None)
        if value is None:
            raise MissingValueError(self._missing_message([path]))
        return value

    def require(self, *paths: str, needed_by: str = "this code path") -> tuple:
        """Fetch several values, reporting *every* missing one in one error.

        Call this at construction time of anything that needs measurements, so
        that an uncalibrated robot fails at startup with a complete shopping
        list rather than one field per run.
        """
        missing: list[str] = []
        out: list[Any] = []
        for p in paths:
            root = self._derived[p].source if p in self._derived else p
            if root not in self._schema:
                raise ConfigSchemaError(self._unknown_path_message(p))
            if self._values.get(root, None) is None:
                missing.append(p)
                out.append(None)
            else:
                out.append(self.get(p))
        if missing:
            raise MissingValueError(self._missing_message(missing, needed_by))
        return tuple(out)

    def get_or(self, path: str, default: Any) -> Any:
        """Value, or ``default`` if it is null. Use sparingly and never for a
        physical constant -- a defaulted measurement is exactly the failure
        mode this module exists to prevent."""
        try:
            return self.get(path)
        except MissingValueError:
            return default

    def has(self, path: str) -> bool:
        try:
            self.get(path)
            return True
        except MissingValueError:
            return False

    def absent_paths(self) -> list[str]:
        """Schema keys the file does not mention at all (as opposed to null).

        Usually means the file predates a schema addition. Worth reporting at
        startup so it is not mistaken for a missing measurement.
        """
        return [p for p in self._schema if p not in self._values]

    def null_paths(self) -> list[str]:
        return [p for p in self._schema if self._values.get(p, None) is None]

    def resolved(self) -> dict[str, Any]:
        """Flat dict of everything, nulls included, for the run log."""
        return dict(sorted(self._values.items()))

    # -- messages -------------------------------------------------------

    def _unknown_path_message(self, path: str) -> str:
        other = "nav" if self.name == "robot" else "robot"
        other_schema = NAV_SCHEMA if self.name == "robot" else ROBOT_SCHEMA
        hint = ""
        if path in other_schema:
            hint = (
                f"\n  It is in the {other} schema. You are holding the "
                f"{self.name} config -- wrong object."
            )
        near = [p for p in self._schema if p.split(".")[-1] == path.split(".")[-1]]
        if near and not hint:
            hint = f"\n  Did you mean: {', '.join(near)}"
        return f"{self.source}: no schema entry for {path!r}.{hint}"

    def _missing_message(self, paths: Iterable[str], needed_by: str = "this code path") -> str:
        paths = list(paths)
        lines = [
            f"{self.source} is missing {len(paths)} value(s) that {needed_by} needs:",
            "",
        ]
        for p in paths:
            spec: Field | Derived
            root = p
            if p in self._derived:
                root = self._derived[p].source
                spec = self._schema[root]
                lines.append(f"  {p}  (from {root})")
            else:
                spec = self._schema[p]
                lines.append(f"  {p}")
            lines.append(f"      {spec.describe()}")
            if isinstance(spec, Field) and spec.where:
                lines.append(f"      -> {spec.where}")
            if root not in self._values:
                lines.append("      (the key is absent from the file, not just null)")
        lines += ["", self._how_to_fix()]
        return "\n".join(lines)

    def _how_to_fix(self) -> str:
        if self.name == "robot":
            return (
                "Teemy measures these on the hardware and commits config/robot.yaml;\n"
                "docs/HARDWARE.md has the procedure for each. Do not substitute a\n"
                "plausible default -- a wrong constant does not crash, it makes the\n"
                "robot subtly and consistently wrong in a way that looks like a\n"
                "tuning problem for two days.\n"
                "\n"
                "To run with no hardware, use the fictional robot instead:\n"
                "    --robot-config config/sim_robot.yaml\n"
                "Nothing in that file is a measurement of anything."
            )
        return (
            "These are nav's own tuning values, not measurements. config/nav.yaml\n"
            "ships with a working set; if a key is missing, the file predates it."
        )

    def __repr__(self) -> str:
        n_null = len(self.null_paths())
        return f"<Config {self.name} from {self.source}: {len(self._schema)} keys, {n_null} null>"


# ---------------------------------------------------------------------------
# loading
# ---------------------------------------------------------------------------


def _load(path: Path, schema: Mapping[str, Field], derived: Mapping[str, Derived], name: str) -> Config:
    source = str(path)
    if not path.is_file():
        raise ConfigError(f"{source}: no such config file")
    with path.open("r", encoding="utf-8") as fh:
        raw = yaml.safe_load(fh)
    if raw is None:
        raw = {}
    if not isinstance(raw, dict):
        raise ConfigSchemaError(f"{source}: top level must be a mapping, got {type(raw).__name__}")

    flat = _flatten(raw)

    unknown = sorted(set(flat) - set(schema))
    if unknown:
        raise ConfigSchemaError(
            f"{source}: {len(unknown)} key(s) the schema does not define: "
            + ", ".join(unknown)
            + "\n  Either it is a typo, or nav needs to add it to the schema in "
            "src/fodnav/config.py.\n  Nav owns the schema; ask before inventing a key."
        )

    values: dict[str, Any] = {}
    for p, v in flat.items():
        values[p] = None if v is None else schema[p].coerce(p, v, source)

    for paths, check in _pairwise_checks(name):
        vals = [values.get(p) for p in paths]
        if any(v is None for v in vals):
            continue
        problem = check(*vals)
        if problem:
            raise ConfigSchemaError(f"{source}: {problem}")

    return Config(values, schema, derived, source, name)


def find_config_dir(explicit: str | os.PathLike | None = None) -> Path:
    """Locate the ``config/`` directory.

    Order: an explicit argument, then ``$FODNAV_CONFIG_DIR``, then ``./config``,
    then the repo root above this file. Nothing here searches the whole system:
    a config found by accident is worse than one not found at all.
    """
    if explicit is not None:
        p = Path(explicit).expanduser()
        if not p.is_dir():
            raise ConfigError(f"{p}: not a directory")
        return p
    env = os.environ.get("FODNAV_CONFIG_DIR")
    if env:
        p = Path(env).expanduser()
        if not p.is_dir():
            raise ConfigError(f"$FODNAV_CONFIG_DIR={env}: not a directory")
        return p
    cwd = Path.cwd() / "config"
    if cwd.is_dir():
        return cwd
    repo = Path(__file__).resolve().parents[2] / "config"
    if repo.is_dir():
        return repo
    raise ConfigError(
        "cannot find a config/ directory. Run from the repo root, pass an "
        "explicit path, or set $FODNAV_CONFIG_DIR."
    )


def load_robot_config(path: str | os.PathLike | None = None) -> Config:
    """Load ``config/robot.yaml`` (or a schema-identical stand-in)."""
    p = Path(path) if path is not None else find_config_dir() / "robot.yaml"
    return _load(Path(p), ROBOT_SCHEMA, ROBOT_DERIVED, "robot")


def load_nav_config(path: str | os.PathLike | None = None) -> Config:
    """Load ``config/nav.yaml``."""
    p = Path(path) if path is not None else find_config_dir() / "nav.yaml"
    return _load(Path(p), NAV_SCHEMA, NAV_DERIVED, "nav")
