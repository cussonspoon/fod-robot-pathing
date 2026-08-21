"""The wire protocol. Codec round-trips, malformed lines, and watchdog timing.

docs/protocol.md is a contract between two repos and it has not been reviewed
by Teemy yet. These tests are how nav states, precisely and checkably, what it
believes the document says.
"""

from __future__ import annotations

import math

import pytest

from fodnav.config import load_robot_config
from fodnav.link.esp32 import (
    MAX_LINE_BYTES,
    PROTO_VERSION,
    WATCHDOG_TIMEOUT_MS,
    CmdMagnet,
    CmdSimple,
    CmdVelocity,
    Esp32Link,
    Flags,
    HandshakeError,
    Info,
    LineReader,
    LogLine,
    LoopbackTransport,
    ProtocolError,
    SerialTransport,
    Telemetry,
    encode_disable,
    encode_enable,
    encode_info,
    encode_magnet,
    encode_stop,
    encode_telemetry,
    encode_velocity,
    parse_command,
    parse_line,
    seq_delta,
)
from fodnav.sim.firmware import FakeFirmware, FirmwareConstants, build_sim_link
from fodnav.sim.unicycle import SimParams

DT = 0.005


@pytest.fixture(scope="module")
def robot():
    return load_robot_config("config/sim_robot.yaml")


# -- encoding -------------------------------------------------------------


@pytest.mark.parametrize(
    "v, omega, expect",
    [
        (0.25, -0.4, b"V 0.250 -0.400\n"),
        (0.0, 0.0, b"V 0.000 0.000\n"),
        (-1.5, 2.0, b"V -1.500 2.000\n"),
        (0.0001, 0.0, b"V 0.000 0.000\n"),
    ],
)
def test_velocity_encoding_matches_the_document(v, omega, expect):
    assert encode_velocity(v, omega) == expect


def test_the_documented_command_examples_encode_exactly():
    # The four examples printed in protocol.md section 3.
    assert encode_velocity(0.250, -0.400) == b"V 0.250 -0.400\n"
    assert encode_velocity(0.0, 0.0) == b"V 0.000 0.000\n"
    assert encode_stop() == b"S\n"
    assert encode_magnet(True) == b"M 1\n"


def test_negative_zero_is_normalised_away():
    # Legal by the letter of the spec, confusing in a log at 3 a.m.
    assert encode_velocity(-1e-9, -0.0) == b"V 0.000 0.000\n"


def test_no_exponent_notation_ever_leaves_this_process():
    line = encode_velocity(1e-7, 12345.6789)
    assert b"e" not in line and b"E" not in line


@pytest.mark.parametrize("bad", [float("nan"), float("inf"), float("-inf")])
def test_non_finite_values_are_refused_before_they_reach_the_wire(bad):
    with pytest.raises(ProtocolError):
        encode_velocity(bad, 0.0)


def test_lines_stay_inside_the_length_limit():
    assert len(encode_velocity(-999.999, -999.999)) <= MAX_LINE_BYTES
    t = Telemetry(65535, 4294967295, -2147483648, 2147483647, -9.999, -9.999, Flags(0xFF))
    assert len(encode_telemetry(t)) <= MAX_LINE_BYTES


def test_an_over_long_line_is_refused_rather_than_truncated():
    with pytest.raises(ProtocolError, match="byte limit"):
        encode_info(Info(1, "x" * 200, 1440.0, 0.0325, 0.2))


# -- decoding -------------------------------------------------------------


def test_the_documented_telemetry_example_parses():
    # protocol.md section 4, verbatim.
    t = parse_line("T 4821 903442 -128374 -128991 0.248 -0.402 03")
    assert isinstance(t, Telemetry)
    assert (t.seq, t.t_ms, t.ticks_l, t.ticks_r) == (4821, 903442, -128374, -128991)
    assert (t.v_meas, t.omega_meas) == (0.248, -0.402)
    assert t.flags.motors_enabled and t.flags.watchdog_fired


def test_telemetry_round_trips():
    t = Telemetry(4821, 903442, -128374, -128991, 0.248, -0.402, Flags(0x03))
    back = parse_line(encode_telemetry(t))
    assert (back.seq, back.ticks_l, back.ticks_r, int(back.flags)) == (
        t.seq, t.ticks_l, t.ticks_r, int(t.flags)
    )


def test_the_handshake_line_round_trips_at_micron_precision():
    # The whole point of millimetres on the wire: 1e-6 m must survive the trip,
    # because that is the tolerance the handshake asserts to.
    i = Info(PROTO_VERSION, "fw-0.3.1", 1440.0, 0.032500, 0.200000)
    back = parse_line(encode_info(i))
    assert back.wheel_radius_m == pytest.approx(i.wheel_radius_m, abs=1e-9)
    assert back.track_width_m == pytest.approx(i.track_width_m, abs=1e-9)
    assert back.fw_version == "fw-0.3.1"


def test_a_metre_encoded_handshake_would_have_failed():
    # Guards the reason the units are what they are. 0.0325 at three decimal
    # places is 0.032, which is 1.5% out -- the exact error the handshake
    # exists to catch.
    assert abs(round(0.0325, 3) - 0.0325) > 1e-6


def test_log_lines_keep_their_spaces():
    m = parse_line("L W drum current high, backing off")
    assert isinstance(m, LogLine)
    assert m.level == "W" and m.message == "drum current high, backing off"


def test_carriage_returns_are_accepted_and_ignored():
    assert parse_line("S\r\n") is None  # S is a command, not a device->host line
    t = parse_line("T 1 2 3 4 0.1 0.2 00\r")
    assert isinstance(t, Telemetry)


@pytest.mark.parametrize(
    "line",
    ["garbage from the bootloader", "rst:0x1 (POWERON_RESET)", "", "X 1 2 3", "  "],
)
def test_an_unrecognised_first_token_is_ignored_not_an_error(line):
    # Section 2: ignore, do not error, do not reset. Boot spew is expected.
    assert parse_line(line) is None


@pytest.mark.parametrize(
    "line",
    [
        "T 1 2 3",                          # too few fields
        "T 1 2 3 4 0.1 0.2 00 extra",       # too many
        "T 1 2 3 4 x 0.2 00",               # non-numeric
        "T 1 2 3 4 0.1 0.2 zz",             # flags not hex
        "T 1 2 3.5 4 0.1 0.2 00",           # non-integer ticks
        "T 1 2 3 4 1e5 0.2 00",             # exponent notation
        "I 1 fw",                           # short handshake
        "L X message",                      # bad level
        "L I",                              # no message
    ],
)
def test_a_recognised_line_with_a_bad_field_raises(line):
    with pytest.raises(ProtocolError):
        parse_line(line)


def test_command_parsing_round_trips_both_directions():
    assert parse_command(encode_velocity(0.25, -0.4)) == CmdVelocity(0.25, -0.4)
    assert parse_command(encode_stop()) == CmdSimple("S")
    assert parse_command(encode_enable()) == CmdSimple("E")
    assert parse_command(encode_disable()) == CmdSimple("D")
    assert parse_command(encode_magnet(False)) == CmdMagnet(False)


@pytest.mark.parametrize("line", ["V 0.1", "V 0.1 0.2 0.3", "V a b", "M 2", "M", "S now", "E 1"])
def test_malformed_commands_raise_on_the_firmware_side(line):
    with pytest.raises(ProtocolError):
        parse_command(line)


def test_seq_delta_wraps_at_uint16():
    assert seq_delta(3, 65535) == 4
    assert seq_delta(1, 0) == 1
    assert seq_delta(0, 65535) == 1


# -- flags ----------------------------------------------------------------


def test_every_documented_flag_bit_has_a_name():
    assert Flags(0xFF).names() == [
        "motors_enabled", "watchdog_fired", "driver_fault", "obstacle",
        "encoder_fault", "clamped", "battery_low", "fault_state",
    ]
    assert Flags(0x00).describe() == "none"


def test_refuses_motion_covers_the_states_where_commanding_is_pointless():
    assert Flags(0x00).refuses_motion          # not enabled
    assert Flags(0x80).refuses_motion          # fault state
    assert Flags(0x05).refuses_motion          # enabled but driver fault
    assert not Flags(0x01).refuses_motion


# -- the line reader ------------------------------------------------------


def test_lines_are_split_on_newlines_across_chunk_boundaries():
    r = LineReader()
    out = list(r.feed(b"T 1 2 3 4 0.1 0.2 00\nT 2 3 4 5 0.1")) + list(r.feed(b" 0.2 00\n"))
    assert len(out) == 2 and out[1].startswith(b"T 2")


def test_garbage_before_the_first_newline_is_discarded():
    # Expected on connect: boot messages, line noise.
    r = LineReader()
    lines = list(r.feed(b"\x00\xff rst:0x1 junk\nT 1 2 3 4 0.1 0.2 00\n"))
    assert len(lines) == 2
    assert parse_line(lines[0]) is None
    assert isinstance(parse_line(lines[1]), Telemetry)


def test_an_over_long_line_is_dropped_and_the_stream_resynchronises():
    r = LineReader()
    flood = b"T " + b"9" * (MAX_LINE_BYTES + 50) + b"\nT 1 2 3 4 0.1 0.2 00\n"
    lines = list(r.feed(flood))
    assert r.n_overlong == 1
    assert len(lines) == 1, "the over-long line is dropped whole, not truncated into a bad one"
    assert isinstance(parse_line(lines[0]), Telemetry)


def test_a_truncated_final_line_is_held_not_emitted():
    r = LineReader()
    assert list(r.feed(b"T 1 2 3 4 0.1 0.2")) == []


# -- the link -------------------------------------------------------------


def sim_link(robot, **kw):
    link, fw = build_sim_link(robot, SimParams.perfect(), **kw)
    return link, fw


def pump(fw, link, seconds, dt=DT, command=None):
    """Advance both sides, optionally sending a command each tick."""
    for _ in range(int(round(seconds / dt))):
        if command is not None:
            command()
        fw.step(dt)
        link.poll()


def test_the_handshake_verifies_and_returns(robot):
    link, fw = sim_link(robot)
    info = link.open(pump=lambda: fw.step(DT))
    assert info.proto_version == PROTO_VERSION
    assert link.info is not None


def test_a_protocol_version_mismatch_refuses_to_run(robot):
    link, fw = sim_link(robot)
    fw.proto_version = 2
    with pytest.raises(HandshakeError, match="protocol version"):
        link.open(pump=lambda: fw.step(DT))


@pytest.mark.parametrize(
    "field, value",
    [("wheel_radius_m", 0.0330), ("track_width_m", 0.2100), ("ticks_per_rev", 1436.0)],
)
def test_a_constants_divergence_refuses_to_run(robot, field, value):
    # This is the failure the handshake exists for: the firmware and
    # robot.yaml disagreeing produces odometry that is wrong by a fixed
    # percentage and looks like a controller tuning problem for days.
    link, fw = sim_link(robot, constants=FirmwareConstants.from_config(robot, **{field: value}))
    with pytest.raises(HandshakeError, match=field):
        link.open(pump=lambda: fw.step(DT))


def test_a_silent_port_times_out_with_a_useful_message(robot):
    link = Esp32Link(LoopbackTransport(), robot=robot)
    with pytest.raises(HandshakeError, match="no I reply"):
        link.handshake(timeout_s=0.05)


def test_the_link_counts_dropped_telemetry_lines(robot):
    link, fw = sim_link(robot)
    link.open(pump=lambda: fw.step(DT))
    pump(fw, link, 0.2, command=lambda: link.send_velocity(0.0, 0.0))
    fw.seq = (fw.seq + 5) & 0xFFFF  # as if five lines were lost
    pump(fw, link, 0.1, command=lambda: link.send_velocity(0.0, 0.0))
    assert link.stats.seq_gaps == 1
    assert link.stats.lines_dropped_by_gaps == 5


def test_log_lines_are_handed_to_the_callback(robot):
    seen = []
    link, fw = build_sim_link(robot, SimParams.perfect(), on_log=seen.append)
    link.open(pump=lambda: fw.step(DT))
    fw._log("W", "hello from the firmware")
    link.poll()
    assert seen and seen[-1].message == "hello from the firmware"


def test_shutdown_sends_stop_then_disable(robot):
    link, fw = sim_link(robot)
    link.open(pump=lambda: fw.step(DT))
    link.enable()
    link.send_velocity(0.3, 0.0)
    fw.step(DT)
    link.shutdown()
    fw.step(DT)
    assert fw.v_cmd == 0.0 and fw.enabled is False


def test_the_serial_transport_refuses_the_renumbering_path():
    # /dev/ttyACM0 renumbers on replug and the run then fails at a different
    # time than the mistake.
    with pytest.raises(ValueError, match="by-id"):
        SerialTransport("/dev/ttyACM0")


# -- the firmware's behaviour --------------------------------------------


def test_velocity_is_clamped_not_rejected(robot):
    # A rejected command in a 50 Hz loop becomes a watchdog trip and a sudden
    # stop, so the firmware saturates instead.
    link, fw = sim_link(robot)
    link.open(pump=lambda: fw.step(DT))
    link.enable()
    link.send_velocity(99.0, -99.0)
    fw.step(DT)
    assert fw.v_cmd == robot.get("drive.v_max_mps")
    assert fw.omega_cmd == -robot.get("drive.omega_max_radps")


def test_clamping_raises_bit_five_for_that_cycle_only(robot):
    link, fw = sim_link(robot)
    link.open(pump=lambda: fw.step(DT))
    link.enable()
    link.send_velocity(99.0, 0.0)
    pump(fw, link, 0.03)
    assert any(t.flags.clamped for t in [link.last_telemetry])
    pump(fw, link, 0.05, command=lambda: link.send_velocity(0.1, 0.0))
    assert not link.last_telemetry.flags.clamped


def test_a_partially_parsed_command_is_never_acted_on(robot):
    link, fw = sim_link(robot)
    link.open(pump=lambda: fw.step(DT))
    link.enable()
    link.send_velocity(0.2, 0.0)
    fw.step(DT)
    link.transport.write(b"V 0.9 notanumber\n")  # the 0.9 must not survive
    fw.step(DT)
    assert fw.v_cmd == 0.2
    assert fw.n_malformed == 1


def test_velocity_while_disabled_is_stored_but_does_not_move_the_robot(robot):
    # This is what lets the control loop keep the watchdog fed during a pause.
    link, fw = sim_link(robot)
    link.open(pump=lambda: fw.step(DT))
    pump(fw, link, 0.2, command=lambda: link.send_velocity(0.3, 0.0))
    assert fw.v_cmd == 0.3
    assert fw.sim.distance_m == 0.0
    assert fw.n_watchdog_trips == 0, "a paused loop that keeps sending V must not trip it"
    link.enable()
    pump(fw, link, 0.2, command=lambda: link.send_velocity(0.3, 0.0))
    assert fw.sim.distance_m > 0.0


def test_a_telemetry_request_answers_immediately(robot):
    link, fw = sim_link(robot)
    link.open(pump=lambda: fw.step(DT))
    before = link.stats.telemetry_rx
    link.request_telemetry()
    fw._consume_input()
    got = link.poll()
    assert len(got) == 1 and link.stats.telemetry_rx == before + 1


# -- the watchdog ---------------------------------------------------------


def test_the_watchdog_stops_a_moving_robot_when_the_pi_goes_silent(robot):
    # The single most important requirement in the protocol. A crashed Pi
    # process must not leave the last velocity executing forever.
    link, fw = sim_link(robot)
    link.open(pump=lambda: fw.step(DT))
    link.enable()
    pump(fw, link, 0.5, command=lambda: link.send_velocity(0.3, 0.0))
    assert fw.sim.v_true > 0.2

    last_command_at = fw.t_last_command
    pump(fw, link, 1.0)  # the process died: no commands at all
    assert fw.n_watchdog_trips == 1
    assert fw.sim.v_true == 0.0
    assert fw.v_cmd == 0.0
    assert link.last_telemetry.flags.watchdog_fired
    # It must fire within one timeout of the silence starting, plus a step or
    # two of granularity -- not "eventually".
    fired_after_ms = (fw.t_watchdog_fired - last_command_at) * 1000.0
    assert WATCHDOG_TIMEOUT_MS <= fired_after_ms < WATCHDOG_TIMEOUT_MS + 2 * DT * 1000


def test_the_watchdog_does_not_fire_while_the_loop_is_feeding_it(robot):
    link, fw = sim_link(robot)
    link.open(pump=lambda: fw.step(DT))
    link.enable()
    pump(fw, link, 3.0, command=lambda: link.send_velocity(0.0, 0.0))
    assert fw.n_watchdog_trips == 0, "V 0.000 0.000 is a command and must count as one"


def test_a_zero_velocity_command_still_feeds_the_watchdog(robot):
    link, fw = sim_link(robot)
    link.open(pump=lambda: fw.step(DT))
    link.enable()
    pump(fw, link, 1.0, command=lambda: link.send_velocity(0.0, 0.0))
    assert fw.n_watchdog_trips == 0


def test_the_magnet_command_does_not_feed_the_watchdog(robot):
    # Section 6 lists V/S/E/D/? and not M. If M counted, a drum command could
    # mask a control loop that had stopped producing velocities.
    link, fw = sim_link(robot)
    link.open(pump=lambda: fw.step(DT))
    link.enable()
    link.send_velocity(0.3, 0.0)
    pump(fw, link, 0.5, command=lambda: link.magnet(True))
    assert fw.n_watchdog_trips == 1


def test_recovery_from_a_watchdog_trip_requires_an_explicit_enable(robot):
    # Commands returning is not evidence that whatever stopped them is fixed.
    link, fw = sim_link(robot)
    link.open(pump=lambda: fw.step(DT))
    link.enable()
    pump(fw, link, 0.2, command=lambda: link.send_velocity(0.3, 0.0))
    pump(fw, link, 0.5)  # silence -> trip
    assert fw.watchdog_fired

    pump(fw, link, 0.2, command=lambda: link.send_velocity(0.3, 0.0))
    assert fw.sim.v_true == 0.0, "it must not resume just because commands came back"

    link.enable()
    pump(fw, link, 0.2, command=lambda: link.send_velocity(0.3, 0.0))
    assert fw.sim.v_true > 0.2


def test_the_link_reports_how_close_the_watchdog_is(robot):
    link, _ = sim_link(robot)
    assert link.ms_since_last_command == math.inf
    link.send_velocity(0.0, 0.0)
    assert link.ms_since_last_command < 50
    assert link.watchdog_margin_ms > 200


def test_a_latched_driver_fault_refuses_motion(robot):
    link, fw = sim_link(robot)
    link.open(pump=lambda: fw.step(DT))
    link.enable()
    fw.latch_driver_fault()
    pump(fw, link, 0.2, command=lambda: link.send_velocity(0.3, 0.0))
    assert fw.sim.v_true == 0.0
    assert link.last_telemetry.flags.fault_state
    assert link.last_telemetry.flags.refuses_motion
