"""The config loader. Its job is to fail loudly, so most of this tests failure."""

from __future__ import annotations

import math
import textwrap
from pathlib import Path

import pytest

from fodnav.config import (
    ConfigSchemaError,
    MissingValueError,
    load_nav_config,
    load_robot_config,
)

REPO = Path(__file__).resolve().parents[1]
ROBOT_YAML = REPO / "config" / "robot.yaml"
SIM_YAML = REPO / "config" / "sim_robot.yaml"
NAV_YAML = REPO / "config" / "nav.yaml"


def write(tmp_path: Path, body: str) -> Path:
    p = tmp_path / "robot.yaml"
    p.write_text(textwrap.dedent(body), encoding="utf-8")
    return p


# -- the files that ship --------------------------------------------------


def test_the_real_robot_yaml_parses_and_is_still_unmeasured():
    cfg = load_robot_config(ROBOT_YAML)
    # It ships as nulls. If this ever fails because it is full of numbers,
    # good -- delete the test. Until then, it documents the state of play.
    assert cfg.null_paths(), "robot.yaml has values now; update this test"
    assert cfg.absent_paths() == [], f"schema keys missing from the file: {cfg.absent_paths()}"


def test_robot_yaml_matches_the_block_printed_in_hardware_md():
    # HARDWARE.md section 8 is what Teemy fills in. If the file drifts from the
    # document, he edits one thing and nav reads another.
    doc = (REPO / "docs" / "HARDWARE.md").read_text(encoding="utf-8")
    block = doc.split("## 8. The file to edit", 1)[1].split("```yaml", 1)[1].split("```", 1)[0]
    assert ROBOT_YAML.read_text(encoding="utf-8").strip() == block.strip()


def test_the_fictional_robot_is_complete_and_schema_identical():
    sim = load_robot_config(SIM_YAML)
    assert sim.null_paths() == [], "sim_robot.yaml must be fully populated"
    assert sim.absent_paths() == []
    real = load_robot_config(ROBOT_YAML)
    assert set(sim.resolved()) == set(real.resolved()), (
        "sim_robot.yaml and robot.yaml must stay key-identical, so that swapping "
        "one for the other exercises the same code"
    )


def test_nav_yaml_is_complete():
    nav = load_nav_config(NAV_YAML)
    assert nav.null_paths() == [], "nav.yaml holds tuning values; none may be null"
    assert nav.absent_paths() == []


def test_shipped_nav_values_are_the_ones_the_contract_requires():
    nav = load_nav_config(NAV_YAML)
    # protocol.md section 6. Not a preference.
    assert nav.get("loop.rate_hz") == 50
    # CLAUDE.md section 8: one target class, and unknown stays suppressed.
    assert set(nav.get("detections.target_classes")) == {"nail", "screw", "bolt"}
    assert "unknown" in nav.get("detections.ignore_classes")


# -- null handling --------------------------------------------------------


def test_a_null_raises_at_use_naming_the_procedure():
    cfg = load_robot_config(ROBOT_YAML)
    with pytest.raises(MissingValueError) as e:
        cfg.get("drive.wheel_radius_m")
    msg = str(e.value)
    assert "drive.wheel_radius_m" in msg
    assert "HARDWARE.md" in msg  # the message must say how to get the number


def test_require_reports_every_missing_value_at_once():
    cfg = load_robot_config(ROBOT_YAML)
    with pytest.raises(MissingValueError) as e:
        cfg.require(
            "drive.wheel_radius_m", "drive.track_width_m", "drive.ticks_per_rev",
            needed_by="odometry",
        )
    msg = str(e.value)
    assert msg.count("drive.") >= 3
    assert "odometry" in msg


def test_an_unknown_path_is_a_different_error_from_a_missing_measurement():
    # A typo in nav's own code is a bug; a null is a measurement not yet taken.
    # Conflating them would let a typo look like an uncalibrated robot.
    cfg = load_robot_config(ROBOT_YAML)
    with pytest.raises(ConfigSchemaError):
        cfg.get("drive.wheel_radius")


def test_asking_the_wrong_config_object_says_so():
    nav = load_nav_config(NAV_YAML)
    with pytest.raises(ConfigSchemaError, match="robot schema"):
        nav.get("link.port")


def test_get_or_falls_back_but_has_reports_honestly():
    cfg = load_robot_config(ROBOT_YAML)
    assert cfg.get_or("drive.wheel_radius_m", 0.0325) == 0.0325
    assert cfg.has("drive.wheel_radius_m") is False


# -- load-time validation -------------------------------------------------


def test_an_unknown_key_is_rejected_with_the_key_named(tmp_path):
    p = write(tmp_path, """
        drive:
          wheel_radius_m: 0.0325
          whee1_radius_m: 0.0325
    """)
    with pytest.raises(ConfigSchemaError, match="whee1_radius_m"):
        load_robot_config(p)


def test_a_string_where_a_number_belongs_is_rejected(tmp_path):
    p = write(tmp_path, """
        drive:
          wheel_radius_m: "0.0325"
    """)
    with pytest.raises(ConfigSchemaError, match="must be a number"):
        load_robot_config(p)


def test_a_wheel_radius_in_millimetres_is_rejected(tmp_path):
    # 32.5 is not a wheel radius in metres, it is a wheel radius in the wrong
    # unit, and it is exactly the mistake the range check exists to catch.
    p = write(tmp_path, """
        drive:
          wheel_radius_m: 32.5
    """)
    with pytest.raises(ConfigSchemaError, match="above the plausible maximum"):
        load_robot_config(p)


def test_a_bad_quadrature_mode_is_rejected(tmp_path):
    p = write(tmp_path, "drive:\n  quadrature: 3\n")
    with pytest.raises(ConfigSchemaError, match="must be one of"):
        load_robot_config(p)


def test_cross_field_checks_fire_when_both_values_are_present(tmp_path):
    p = write(tmp_path, """
        drive:
          v_min_mps: 0.5
          v_max_mps: 0.4
    """)
    with pytest.raises(ConfigSchemaError, match="v_min_mps must be below"):
        load_robot_config(p)


def test_cross_field_checks_stay_quiet_while_a_value_is_still_null(tmp_path):
    p = write(tmp_path, """
        drive:
          v_min_mps: 0.5
          v_max_mps: null
    """)
    load_robot_config(p)  # must not raise; the measurement simply is not in yet


def test_the_near_and_far_fov_limits_must_be_the_right_way_round(tmp_path):
    p = write(tmp_path, """
        camera:
          fov_near_limit_m: 1.6
          fov_far_limit_m: 0.18
    """)
    with pytest.raises(ConfigSchemaError, match="fov_near_limit_m"):
        load_robot_config(p)


def test_a_missing_file_is_a_clear_error(tmp_path):
    with pytest.raises(Exception, match="no such config file"):
        load_robot_config(tmp_path / "nope.yaml")


# -- derived values -------------------------------------------------------


def test_degrees_are_converted_at_the_loader_and_nowhere_else():
    sim = load_robot_config(SIM_YAML)
    assert sim.get("camera.tilt_rad") == pytest.approx(math.radians(sim.get("camera.tilt_deg")))
    assert sim.get("drum.clearance_m") == pytest.approx(sim.get("drum.clearance_mm") / 1000.0)


def test_derived_values_report_their_source_when_missing():
    cfg = load_robot_config(ROBOT_YAML)
    with pytest.raises(MissingValueError, match="camera.tilt_deg"):
        cfg.require("camera.tilt_rad", needed_by="the ground projection")


def test_loop_period_comes_from_the_rate():
    nav = load_nav_config(NAV_YAML)
    assert nav.get("loop.dt_s") == pytest.approx(1.0 / nav.get("loop.rate_hz"))
    assert nav.get("loop.vision_timeout_s") == pytest.approx(
        nav.get("loop.vision_timeout_ms") / 1000.0
    )
