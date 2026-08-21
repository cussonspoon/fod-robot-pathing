"""The control loop, and specifically its first obligation: the heartbeat.

These run the real :class:`~fodnav.runner.ControlLoop` against the simulated
firmware over the real codec, so a mistimed send here is a mistimed send on the
robot.
"""

from __future__ import annotations

import pytest

from fodnav.config import load_nav_config, load_robot_config
from fodnav.frames import Pose2D
from fodnav.fsm import State
from fodnav.link.esp32 import WATCHDOG_TIMEOUT_MS
from fodnav.sim.firmware import FirmwareConstants
from fodnav.sim.harness import SimHarness
from fodnav.sim.scene import SceneNoise
from fodnav.sim.unicycle import SimParams


@pytest.fixture(scope="module")
def robot():
    return load_robot_config("config/sim_robot.yaml")


def nav_cfg(**overrides):
    cfg = load_nav_config("config/nav.yaml")
    for k, v in overrides.items():
        cfg._values[k.replace("__", ".")] = v
    return cfg


def harness(robot, mode="target", targets=(), **kw):
    return SimHarness(
        robot=robot,
        nav=kw.pop("nav", nav_cfg(mission__mode=mode)),
        targets=list(targets),
        params=kw.pop("params", SimParams.from_config(load_nav_config("config/nav.yaml"))),
        **kw,
    )


# -- the heartbeat --------------------------------------------------------


def test_a_velocity_command_goes_out_on_every_single_tick(robot):
    # docs/protocol.md §6: at a fixed 50 Hz whenever the link is open,
    # including V 0.000 0.000, including when nothing has changed. Never
    # skipped as an optimisation.
    h = harness(robot, mode="idle")
    h.run(duration_s=5.0)
    assert h.firmware.n_velocity_commands == h.loop.stats.ticks
    assert h.loop.stats.ticks == pytest.approx(5.0 * 50, rel=0.02)


def test_a_stationary_robot_still_feeds_the_watchdog(robot):
    h = harness(robot, mode="idle")
    h.run(duration_s=10.0)
    assert h.firmware.n_watchdog_trips == 0
    assert h.sim.distance_m == 0.0


def test_the_watchdog_never_fires_during_a_normal_run(robot):
    h = harness(robot, mode="target", targets=[(1.3, 0.2)])
    h.run(duration_s=30.0)
    assert h.firmware.n_watchdog_trips == 0
    assert h.firmware.ms_since_command < WATCHDOG_TIMEOUT_MS


def test_when_the_loop_stops_ticking_the_firmware_stops_the_robot(robot):
    # The kill -9 case, in simulation. The Pi process goes away mid-drive and
    # the robot must not keep going.
    h = harness(robot, mode="target", targets=[(1.4, 0.2)])
    h.start()
    for _ in range(400):  # get it moving
        h.loop.tick()
        h.clock.sleep_until(h.clock.t + h.loop.dt)
    assert h.sim.v_true > 0.05, "it should be driving before we kill it"

    for _ in range(200):  # the loop is gone; only the firmware still runs
        h.firmware.step(0.005)
    assert h.firmware.n_watchdog_trips == 1
    assert h.sim.v_true == 0.0


def test_an_exception_in_the_loop_stops_the_wheels_before_propagating(robot):
    h = harness(robot, mode="target", targets=[(1.4, 0.2)])
    h.start()
    for _ in range(200):
        h.loop.tick()
        h.clock.sleep_until(h.clock.t + h.loop.dt)
    assert h.sim.v_true > 0.05

    boom = RuntimeError("something in perception exploded")

    def explode(_inputs):
        raise boom

    h.fsm.update = explode
    with pytest.raises(RuntimeError):
        h.loop.run(duration_s=1.0)
    h.firmware.step(0.01)
    assert h.firmware.v_cmd == 0.0, "S must go out on the way through the exception"


def test_safe_stop_is_idempotent_and_never_raises(robot):
    h = harness(robot, mode="idle")
    h.start()
    h.loop.safe_stop()
    h.loop.safe_stop()
    h.firmware.step(0.01)
    assert h.firmware.v_cmd == 0.0
    assert not h.firmware.magnet_on


# -- startup --------------------------------------------------------------


def test_the_handshake_runs_before_anything_moves(robot):
    h = harness(robot, mode="target", targets=[(1.2, 0.0)])
    assert h.link.info is None
    h.start()
    assert h.link.info is not None and h.link.info.proto_version == 1


def test_a_firmware_built_with_different_constants_refuses_the_run(robot):
    from fodnav.link.esp32 import HandshakeError

    h = harness(
        robot,
        mode="idle",
        firmware_constants=FirmwareConstants.from_config(robot, track_width_m=0.21),
    )
    with pytest.raises(HandshakeError, match="track_width_m"):
        h.start()


# -- the missions, end to end --------------------------------------------


def test_the_robot_collects_a_thrown_fastener(robot):
    target = (1.35, 0.30)
    h = harness(robot, mode="target", targets=[target])
    h.run(duration_s=40.0)
    summary = h.summary()
    assert summary["fsm"]["targets_collected"] >= 1
    assert h.drum_miss_m(target) < robot.get("drum.capture_width_m") / 2
    assert summary["loop"]["overruns"] == 0


def test_it_still_collects_when_the_detector_is_unreliable(robot):
    target = (1.2, -0.25)
    h = harness(
        robot, mode="target", targets=[target],
        noise=SceneNoise(conf=0.72, conf_noise=0.1, jitter_px=5.0, miss_rate=0.3, seed=11),
    )
    h.run(duration_s=40.0)
    assert h.drum_miss_m(target) < robot.get("drum.width_m") / 2


def test_vision_dying_mid_approach_stops_the_robot_short(robot):
    # The correct outcome is a miss. Driving on to where the nail was last seen
    # would be guessing with a moving robot and a dead camera.
    target = (1.6, 0.0)
    h = harness(robot, mode="target", targets=[target], vision_dead_after_s=2.0)
    h.run(duration_s=15.0)
    assert h.fsm.state is State.STOPPED
    assert "vision" in h.fsm.reason
    assert h.sim.v_true == 0.0


def test_a_coverage_sweep_completes_and_reports_what_it_swept(robot):
    h = harness(robot, mode="coverage", params=SimParams.perfect())
    stats = h.run(duration_s=600.0)
    assert h.fsm.state is State.DONE
    assert "complete" in stats.stop_reason
    coverage = h.summary()["coverage"]
    assert coverage["fraction"] > 0.95


def test_odometry_drift_is_what_ruins_a_coverage_sweep(robot):
    # Same path, same controller, same everything except a 1.5% wheel-scale
    # mismatch. This is the number that says whether open-loop coverage of the
    # arena is viable, and on this geometry it is not.
    clean = harness(robot, mode="coverage", params=SimParams.perfect())
    clean.run(duration_s=600.0)
    drifting = harness(robot, mode="coverage", params=SimParams(wheel_scale_left=1.015))
    drifting.run(duration_s=600.0)
    assert clean.summary()["coverage"]["fraction"] > 0.95
    assert drifting.summary()["coverage"]["fraction"] < 0.75
    assert drifting.summary()["odometry_error_m"] > 1.0


# -- logging --------------------------------------------------------------


def test_a_run_writes_a_reproducible_record(robot, tmp_path):
    import json

    from fodnav.runlog import RunLog

    log = RunLog(root=tmp_path, name="test")
    log.write_meta(argv=["fodnav-sim", "--target", "1.2", "0"])
    nav = nav_cfg(mission__mode="target")
    log.write_config(robot, nav)
    h = SimHarness(robot=robot, nav=nav, targets=[(1.2, 0.0)], log=log)
    h.run(duration_s=10.0)
    log.close(h.summary())

    meta = json.loads((log.dir / "meta.json").read_text())
    assert "git" in meta and "sha" in meta["git"]
    assert meta["git"]["dirty"] in (True, False)
    config = json.loads((log.dir / "config.json").read_text())
    assert config["robot"]["values"]["drive.wheel_radius_m"] == robot.get("drive.wheel_radius_m")
    summary = json.loads((log.dir / "summary.json").read_text())
    assert summary["ticks"] > 0
    lines = (log.dir / "stream.jsonl").read_text().strip().splitlines()
    assert len(lines) == summary["ticks"]
    first = json.loads(lines[0])
    assert {"t", "state", "v", "omega", "reason"} <= set(first)


def test_a_broken_log_directory_does_not_stop_the_robot(robot, tmp_path):
    from fodnav.runlog import RunLog

    log = RunLog(root=tmp_path, name="broken")
    log._stream = None  # as if the card filled up
    h = SimHarness(robot=robot, nav=nav_cfg(mission__mode="idle"), log=log)
    h.run(duration_s=2.0)
    assert h.loop.stats.ticks > 50
