"""The serial link to the motor firmware. Codec, transport, handshake, watchdog.

**Read ``docs/protocol.md`` before changing anything in this file.** That
document is the contract between two repos and neither side changes it
unilaterally; this module is one side's implementation of it, and it is written
to be readable next to the document rather than clever.

Note the standing caveat from CLAUDE.md: ``docs/protocol.md`` is a *proposal*.
Teemy has not reviewed it. Everything that would have to change if he asks for
a different framing is inside this file.

Framing, briefly: newline-terminated ASCII, single-space separated, at most 120
bytes including the terminator, at most 3 decimal places, no exponents. A
longer line is dropped rather than truncated. An unrecognised first token is
ignored without erroring. A recognised command with a malformed field is
dropped **whole** -- never acted on in part -- and logged.

The watchdog is the reason this file is careful. If the ESP32 has not received
a valid ``V``/``S``/``E``/``D``/``?`` line for 300 ms it zeroes both wheels and
brakes. Nav's side of that contract is a ``V`` at a fixed 50 Hz whenever the
link is open, including ``V 0.000 0.000``, including when nothing changed, and
never skipped as an optimisation. Without it, a crashed Pi process leaves the
last velocity command executing forever, which is a robot accelerating
unattended into an active machine shop.
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass, field
from typing import Callable, Iterator, Protocol

from ..config import Config

__all__ = [
    "PROTO_VERSION",
    "MAX_LINE_BYTES",
    "WATCHDOG_TIMEOUT_MS",
    "TELEMETRY_RATE_HZ",
    "COMMAND_RATE_HZ",
    "Flags",
    "Telemetry",
    "Info",
    "LogLine",
    "ProtocolError",
    "HandshakeError",
    "LineReader",
    "Transport",
    "SerialTransport",
    "LoopbackTransport",
    "Esp32Link",
    "encode_velocity",
    "encode_stop",
    "encode_enable",
    "encode_disable",
    "encode_magnet",
    "encode_telemetry_request",
    "encode_handshake",
    "encode_telemetry",
    "encode_info",
    "parse_line",
    "parse_command",
    "CmdVelocity",
    "CmdMagnet",
    "CmdSimple",
    "WATCHDOG_FEEDING",
    "seq_delta",
]

#: Bump this and tell Teemy the same day if docs/protocol.md is edited.
PROTO_VERSION = 1

#: Longer lines are a protocol error and are dropped, not truncated.
MAX_LINE_BYTES = 120

#: docs/protocol.md section 6. Lives in the firmware; stated here so nav's
#: obligations can be checked against it.
WATCHDOG_TIMEOUT_MS = 300

TELEMETRY_RATE_HZ = 50
COMMAND_RATE_HZ = 50

_LEVELS = ("D", "I", "W", "E")


class ProtocolError(ValueError):
    """A line that does not conform. Dropped and counted, never acted on."""


class HandshakeError(RuntimeError):
    """The firmware and ``config/robot.yaml`` disagree. Refuse to run."""


# ---------------------------------------------------------------------------
# status flags
# ---------------------------------------------------------------------------


class Flags(int):
    """The status bitfield from a ``T`` line, with names."""

    MOTORS_ENABLED = 0x01
    WATCHDOG_FIRED = 0x02
    DRIVER_FAULT = 0x04
    OBSTACLE = 0x08
    ENCODER_FAULT = 0x10
    CLAMPED = 0x20
    BATTERY_LOW = 0x40
    FAULT_STATE = 0x80

    _NAMES = (
        (0x01, "motors_enabled"),
        (0x02, "watchdog_fired"),
        (0x04, "driver_fault"),
        (0x08, "obstacle"),
        (0x10, "encoder_fault"),
        (0x20, "clamped"),
        (0x40, "battery_low"),
        (0x80, "fault_state"),
    )

    @property
    def motors_enabled(self) -> bool:
        return bool(self & self.MOTORS_ENABLED)

    @property
    def watchdog_fired(self) -> bool:
        return bool(self & self.WATCHDOG_FIRED)

    @property
    def driver_fault(self) -> bool:
        return bool(self & self.DRIVER_FAULT)

    @property
    def obstacle(self) -> bool:
        return bool(self & self.OBSTACLE)

    @property
    def encoder_fault(self) -> bool:
        return bool(self & self.ENCODER_FAULT)

    @property
    def clamped(self) -> bool:
        return bool(self & self.CLAMPED)

    @property
    def battery_low(self) -> bool:
        return bool(self & self.BATTERY_LOW)

    @property
    def fault_state(self) -> bool:
        return bool(self & self.FAULT_STATE)

    @property
    def refuses_motion(self) -> bool:
        """Conditions under which commanding velocity is pointless or unsafe."""
        return bool(self & (self.FAULT_STATE | self.DRIVER_FAULT)) or not self.motors_enabled

    def names(self) -> list[str]:
        return [name for mask, name in self._NAMES if self & mask]

    def describe(self) -> str:
        return ",".join(self.names()) or "none"

    def __repr__(self) -> str:
        return f"Flags(0x{int(self):02x}: {self.describe()})"


# ---------------------------------------------------------------------------
# messages
# ---------------------------------------------------------------------------


@dataclass(frozen=True, slots=True)
class Telemetry:
    """One ``T`` line.

    ``ticks_l`` and ``ticks_r`` are cumulative and wrap as signed 32-bit; the
    Pi differences them (see :mod:`fodnav.odom`). Cumulative rather than
    deltas so that a dropped line costs nothing.

    ``v_meas`` and ``omega_meas`` are the firmware's own estimate and are a
    **cross-check only** -- not the odometry source.
    """

    seq: int
    t_ms: int
    ticks_l: int
    ticks_r: int
    v_meas: float
    omega_meas: float
    flags: Flags
    t_recv_monotonic: float = 0.0


@dataclass(frozen=True, slots=True)
class Info:
    """The ``I`` handshake reply. SI in here; millimetres only on the wire.

    The two lengths travel as ``_mm`` because the framing allows 3 decimal
    places and the handshake compares to 1e-6 m: in metres, a 32.5 mm wheel
    radius encodes as ``0.032`` and no firmware could ever pass its own
    handshake. See docs/protocol.md §5. The conversion happens in the codec,
    which is the wire boundary, and nothing above this line sees a millimetre.
    """

    proto_version: int
    fw_version: str
    ticks_per_rev: float
    wheel_radius_m: float
    track_width_m: float


@dataclass(frozen=True, slots=True)
class LogLine:
    """An ``L`` passthrough log line. For humans; never parse state out of it."""

    level: str
    message: str


# ---------------------------------------------------------------------------
# encoding
# ---------------------------------------------------------------------------


def _fmt(value: float) -> str:
    """Format a number the way section 2 allows: 3 decimals, no exponent.

    Negative zero is normalised away. ``-0.000`` is legal by the letter of the
    spec and is confusing in a log at 3 a.m.
    """
    if not math.isfinite(value):
        raise ProtocolError(f"cannot encode non-finite value {value!r}")
    s = f"{value:.3f}"
    return "0.000" if s == "-0.000" else s


def _line(text: str) -> bytes:
    out = (text + "\n").encode("ascii")
    if len(out) > MAX_LINE_BYTES:
        raise ProtocolError(
            f"line is {len(out)} bytes, over the {MAX_LINE_BYTES}-byte limit: {text!r}"
        )
    return out


def encode_velocity(v: float, omega: float) -> bytes:
    """``V <v> <omega>``. Forward positive, counter-clockwise positive.

    Out-of-range values are **not** rejected here. The ESP32 clamps and raises
    flag bit 5 for that cycle, deliberately: a rejected command in a 50 Hz loop
    becomes a watchdog trip and a sudden stop.
    """
    return _line(f"V {_fmt(v)} {_fmt(omega)}")


def encode_stop() -> bytes:
    """``S`` -- zero both wheels and brake. Does not disable."""
    return b"S\n"


def encode_enable() -> bytes:
    return b"E\n"


def encode_disable() -> bytes:
    """``D`` -- coast, drivers to high-Z."""
    return b"D\n"


def encode_magnet(on: bool) -> bytes:
    return b"M 1\n" if on else b"M 0\n"


def encode_telemetry_request() -> bytes:
    return b"?\n"


def encode_handshake() -> bytes:
    return b"H\n"


def encode_telemetry(t: Telemetry) -> bytes:
    """Encode a ``T`` line. Used by the simulated firmware and by tests."""
    return _line(
        f"T {t.seq & 0xFFFF} {t.t_ms & 0xFFFFFFFF} {t.ticks_l} {t.ticks_r} "
        f"{_fmt(t.v_meas)} {_fmt(t.omega_meas)} {int(t.flags):02x}"
    )


def encode_info(i: Info) -> bytes:
    """``I <proto> <fw> <ticks_per_rev> <wheel_radius_mm> <track_width_mm>``.

    Millimetres, not metres: 3 decimal places of a millimetre is 1e-6 m, which
    is exactly the tolerance the handshake asserts to.
    """
    return _line(
        f"I {i.proto_version} {i.fw_version} {_fmt(i.ticks_per_rev)} "
        f"{_fmt(i.wheel_radius_m * 1000.0)} {_fmt(i.track_width_m * 1000.0)}"
    )


def encode_log(level: str, message: str) -> bytes:
    if level not in _LEVELS:
        raise ProtocolError(f"log level must be one of {_LEVELS}, got {level!r}")
    return _line(f"L {level} {message}")


# ---------------------------------------------------------------------------
# decoding
# ---------------------------------------------------------------------------


def _int(tok: str, what: str) -> int:
    try:
        return int(tok)
    except ValueError:
        raise ProtocolError(f"{what}: {tok!r} is not an integer") from None


def _float(tok: str, what: str) -> float:
    # No exponents on the wire (section 2). Accepting them here would let a
    # firmware bug produce '1e30' and have nav quietly believe it.
    if "e" in tok or "E" in tok:
        raise ProtocolError(f"{what}: exponent notation is not allowed on the wire: {tok!r}")
    try:
        v = float(tok)
    except ValueError:
        raise ProtocolError(f"{what}: {tok!r} is not a number") from None
    if not math.isfinite(v):
        raise ProtocolError(f"{what}: {tok!r} is not finite")
    return v


def parse_line(line: str | bytes) -> Telemetry | Info | LogLine | None:
    """Parse one ESP32 -> Pi line.

    Returns ``None`` for a line whose first token is not recognised -- boot
    spew, line noise, a future message type -- because section 2 says to ignore
    those rather than error. Raises :class:`ProtocolError` for a line that
    *is* recognised but malformed, which the caller counts and logs. Either
    way, nothing partially parsed is ever acted on.
    """
    if isinstance(line, (bytes, bytearray)):
        try:
            line = line.decode("ascii")
        except UnicodeDecodeError:
            return None  # line noise, not a message
    line = line.strip("\r\n")
    if not line:
        return None

    parts = line.split(" ")
    tag = parts[0]

    if tag == "T":
        if len(parts) != 8:
            raise ProtocolError(f"T line has {len(parts) - 1} fields, expected 7: {line!r}")
        try:
            flags = Flags(int(parts[7], 16))
        except ValueError:
            raise ProtocolError(f"T flags {parts[7]!r} are not hex") from None
        return Telemetry(
            seq=_int(parts[1], "T seq"),
            t_ms=_int(parts[2], "T t_ms"),
            ticks_l=_int(parts[3], "T ticks_l"),
            ticks_r=_int(parts[4], "T ticks_r"),
            v_meas=_float(parts[5], "T v_meas"),
            omega_meas=_float(parts[6], "T omega_meas"),
            flags=flags,
            t_recv_monotonic=time.monotonic(),
        )

    if tag == "I":
        if len(parts) != 6:
            raise ProtocolError(f"I line has {len(parts) - 1} fields, expected 5: {line!r}")
        return Info(
            proto_version=_int(parts[1], "I proto_version"),
            fw_version=parts[2],
            ticks_per_rev=_float(parts[3], "I ticks_per_rev"),
            # Millimetres on the wire, metres above it.
            wheel_radius_m=_float(parts[4], "I wheel_radius_mm") / 1000.0,
            track_width_m=_float(parts[5], "I track_width_mm") / 1000.0,
        )

    if tag == "L":
        if len(parts) < 3 or parts[1] not in _LEVELS:
            raise ProtocolError(f"L line has no level or no message: {line!r}")
        return LogLine(level=parts[1], message=" ".join(parts[2:]))

    return None


@dataclass(frozen=True, slots=True)
class CmdVelocity:
    v: float
    omega: float


@dataclass(frozen=True, slots=True)
class CmdMagnet:
    on: bool


@dataclass(frozen=True, slots=True)
class CmdSimple:
    """``S``, ``E``, ``D``, ``?`` or ``H`` -- the commands with no arguments."""

    tag: str


#: The commands that feed the watchdog. Section 6 lists exactly these; ``M`` is
#: absent on purpose, so that a drum command cannot mask a dead control loop.
WATCHDOG_FEEDING = frozenset({"V", "S", "E", "D", "?"})


def parse_command(line: str | bytes) -> CmdVelocity | CmdMagnet | CmdSimple | None:
    """Parse one Pi -> ESP32 line. The firmware's side of the codec.

    Nav does not need this to drive a robot -- it needs it to *test* that what
    it encodes is what the firmware will decode, and the simulated firmware in
    :mod:`fodnav.sim.firmware` needs it to be a firmware. Keeping both
    directions in one file is what stops the two from drifting.
    """
    if isinstance(line, (bytes, bytearray)):
        try:
            line = line.decode("ascii")
        except UnicodeDecodeError:
            return None
    line = line.strip("\r\n")
    if not line:
        return None
    parts = line.split(" ")
    tag = parts[0]

    if tag == "V":
        if len(parts) != 3:
            raise ProtocolError(f"V line has {len(parts) - 1} fields, expected 2: {line!r}")
        return CmdVelocity(_float(parts[1], "V v"), _float(parts[2], "V omega"))
    if tag == "M":
        if len(parts) != 2 or parts[1] not in ("0", "1"):
            raise ProtocolError(f"M takes exactly 0 or 1: {line!r}")
        return CmdMagnet(parts[1] == "1")
    if tag in ("S", "E", "D", "?", "H"):
        if len(parts) != 1:
            raise ProtocolError(f"{tag} takes no arguments: {line!r}")
        return CmdSimple(tag)
    return None


def seq_delta(new: int, old: int) -> int:
    """Difference of two uint16 sequence numbers, across the wrap.

    A delta above 1 means telemetry lines were dropped. That is survivable --
    the tick counters are cumulative -- but it is worth counting, because a
    link that drops lines will eventually drop commands.
    """
    return (int(new) - int(old)) % 0x10000


# ---------------------------------------------------------------------------
# transport
# ---------------------------------------------------------------------------


class Transport(Protocol):
    """A byte pipe. Non-blocking in both directions, always."""

    def write(self, data: bytes) -> None: ...
    def read(self) -> bytes: ...
    def close(self) -> None: ...


class SerialTransport:
    """pyserial, opened by stable path.

    ``/dev/ttyACM0`` renumbers on replug, which is why ``robot.yaml`` holds a
    ``/dev/serial/by-id/...`` path and why this refuses the tempting one.
    """

    def __init__(self, port: str, baud: int = 115200) -> None:
        import serial  # imported late so the sim never needs pyserial present

        if port.startswith("/dev/ttyACM") or port.startswith("/dev/ttyUSB"):
            raise ValueError(
                f"{port!r} renumbers on replug and the run will then fail at a different "
                f"time than the mistake. Use the /dev/serial/by-id/... path; "
                f"'ls -l /dev/serial/by-id/' on the Pi with the ESP32 plugged in."
            )
        self.port = port
        self._ser = serial.Serial(port, baudrate=baud, timeout=0, write_timeout=0)

    def write(self, data: bytes) -> None:
        self._ser.write(data)

    def read(self) -> bytes:
        n = self._ser.in_waiting
        return self._ser.read(n) if n else b""

    def flush_input(self) -> None:
        self._ser.reset_input_buffer()

    def close(self) -> None:
        try:
            self._ser.close()
        except Exception:
            pass


class LoopbackTransport:
    """Two byte queues. For tests, and for driving the simulated firmware."""

    def __init__(self) -> None:
        self.to_device = bytearray()
        self.to_host = bytearray()

    def write(self, data: bytes) -> None:
        self.to_device += data

    def read(self) -> bytes:
        out = bytes(self.to_host)
        self.to_host.clear()
        return out

    # -- the device's side --
    def device_read(self) -> bytes:
        out = bytes(self.to_device)
        self.to_device.clear()
        return out

    def device_write(self, data: bytes) -> None:
        self.to_host += data

    def flush_input(self) -> None:
        self.to_host.clear()

    def close(self) -> None:
        pass


class LineReader:
    """Accumulate bytes, hand back complete lines, enforce the length limit.

    Over-length lines are dropped rather than truncated, and the drop continues
    until the next newline resynchronises the stream. Garbage before the first
    newline after connect is expected -- boot messages, line noise -- and is
    discarded by construction.
    """

    def __init__(self, max_len: int = MAX_LINE_BYTES) -> None:
        self.max_len = max_len
        self._buf = bytearray()
        self._skipping = False
        self.n_overlong = 0

    def feed(self, data: bytes) -> Iterator[bytes]:
        for byte in data:
            if byte == 0x0A:  # \n terminates
                if self._skipping:
                    self._skipping = False
                    self._buf.clear()
                    continue
                line = bytes(self._buf)
                self._buf.clear()
                yield line.rstrip(b"\r")  # \r accepted and ignored, never sent
            else:
                if self._skipping:
                    continue
                self._buf.append(byte)
                if len(self._buf) >= self.max_len:
                    self.n_overlong += 1
                    self._skipping = True
                    self._buf.clear()

    def reset(self) -> None:
        self._buf.clear()
        self._skipping = False


# ---------------------------------------------------------------------------
# the link
# ---------------------------------------------------------------------------


@dataclass
class LinkStats:
    lines_rx: int = 0
    telemetry_rx: int = 0
    logs_rx: int = 0
    malformed_rx: int = 0
    unknown_rx: int = 0
    overlong_rx: int = 0
    seq_gaps: int = 0
    lines_dropped_by_gaps: int = 0
    commands_tx: int = 0
    last_error: str = ""

    def as_dict(self) -> dict[str, object]:
        return dict(self.__dict__)


class Esp32Link:
    """Speaks docs/protocol.md over a transport.

    Everything here is non-blocking. The control loop must never stall on the
    serial port, because a stalled loop is a loop that is not feeding the
    watchdog, and the watchdog firing mid-drive is a sudden stop at best.
    """

    def __init__(
        self,
        transport: Transport,
        robot: Config | None = None,
        on_log: Callable[[LogLine], None] | None = None,
    ) -> None:
        self.transport = transport
        self.robot = robot
        self.on_log = on_log
        self.reader = LineReader()
        self.stats = LinkStats()
        self.info: Info | None = None
        self.last_telemetry: Telemetry | None = None
        self._last_seq: int | None = None
        self._last_command_monotonic: float | None = None
        self._enabled = False

    # -- sending --------------------------------------------------------

    def _send(self, data: bytes, feeds_watchdog: bool = True) -> None:
        self.transport.write(data)
        self.stats.commands_tx += 1
        if feeds_watchdog:
            self._last_command_monotonic = time.monotonic()

    def send_velocity(self, v: float, omega: float) -> None:
        self._send(encode_velocity(v, omega))

    def stop(self) -> None:
        """``S``. Zero and brake. Sent on every exception path there is."""
        self._send(encode_stop())

    def enable(self) -> None:
        self._send(encode_enable())
        self._enabled = True

    def disable(self) -> None:
        self._send(encode_disable())
        self._enabled = False

    def magnet(self, on: bool) -> None:
        # M does not feed the watchdog: section 6 lists V/S/E/D/? only, and
        # assuming otherwise would let a drum command mask a dead control loop.
        self._send(encode_magnet(on), feeds_watchdog=False)

    def request_telemetry(self) -> None:
        self._send(encode_telemetry_request())

    @property
    def ms_since_last_command(self) -> float:
        """How close the watchdog is to firing. ``inf`` if nothing was sent."""
        if self._last_command_monotonic is None:
            return float("inf")
        return (time.monotonic() - self._last_command_monotonic) * 1000.0

    @property
    def watchdog_margin_ms(self) -> float:
        return WATCHDOG_TIMEOUT_MS - self.ms_since_last_command

    # -- receiving ------------------------------------------------------

    def poll(self) -> list[Telemetry]:
        """Drain the port and return whatever telemetry arrived. Never blocks."""
        out: list[Telemetry] = []
        data = self.transport.read()
        if not data:
            return out
        for raw in self.reader.feed(data):
            self.stats.lines_rx += 1
            try:
                msg = parse_line(raw)
            except ProtocolError as e:
                self.stats.malformed_rx += 1
                self.stats.last_error = str(e)
                continue
            if msg is None:
                self.stats.unknown_rx += 1
                continue
            if isinstance(msg, Telemetry):
                self.stats.telemetry_rx += 1
                if self._last_seq is not None:
                    d = seq_delta(msg.seq, self._last_seq)
                    if d != 1:
                        self.stats.seq_gaps += 1
                        self.stats.lines_dropped_by_gaps += max(0, d - 1)
                self._last_seq = msg.seq
                self.last_telemetry = msg
                out.append(msg)
            elif isinstance(msg, Info):
                self.info = msg
            elif isinstance(msg, LogLine):
                self.stats.logs_rx += 1
                if self.on_log is not None:
                    self.on_log(msg)
        self.stats.overlong_rx = self.reader.n_overlong
        return out

    # -- startup --------------------------------------------------------

    def handshake(
        self,
        timeout_s: float = 3.0,
        now: Callable[[], float] = time.monotonic,
        pump: Callable[[], None] | None = None,
    ) -> Info:
        """Send ``H``, wait for ``I``, verify it. Section 7 steps 2 and 3.

        Verification is not optional and a mismatch is not a warning. The three
        physical constants are duplicated across two repos out of necessity --
        the firmware needs them for its wheel PID, the Pi for odometry -- and a
        silent divergence produces odometry that is subtly, consistently wrong
        in a way that looks like a controller tuning problem for days. This
        makes it a startup failure instead.
        """
        self.info = None
        self._send(encode_handshake(), feeds_watchdog=False)
        # ``pump`` exists for the simulated firmware, which only advances when
        # something steps it. Against real hardware it is None and this is an
        # ordinary poll-and-wait. Same code path either way.
        deadline = now() + timeout_s
        while now() < deadline:
            if pump is not None:
                pump()
            self.poll()
            if self.info is not None:
                break
            if pump is None:
                time.sleep(0.005)
        if self.info is None:
            raise HandshakeError(
                f"no I reply to H within {timeout_s:.1f} s. The firmware is not running, "
                f"is not speaking this protocol, or the port is wrong. "
                f"({self.stats.lines_rx} lines seen, {self.stats.unknown_rx} unrecognised)"
            )
        self.verify(self.info)
        return self.info

    def verify(self, info: Info, tol: float = 1e-6) -> None:
        if info.proto_version != PROTO_VERSION:
            raise HandshakeError(
                f"protocol version mismatch: firmware speaks {info.proto_version}, nav speaks "
                f"{PROTO_VERSION}. docs/protocol.md was edited without both sides being told. "
                f"Do not continue with a warning."
            )
        if self.robot is None:
            return
        expected = {
            "ticks_per_rev": self.robot.get("drive.ticks_per_rev"),
            "wheel_radius_m": self.robot.get("drive.wheel_radius_m"),
            "track_width_m": self.robot.get("drive.track_width_m"),
        }
        problems = []
        for name, want in expected.items():
            got = getattr(info, name)
            if abs(got - want) > tol:
                problems.append(f"  {name}: firmware {got!r}, {self.robot.source} {want!r}")
        if problems:
            raise HandshakeError(
                "the firmware and config/robot.yaml disagree about the robot:\n"
                + "\n".join(problems)
                + f"\n\nfirmware version: {info.fw_version}\n"
                "These constants are compiled into the firmware for its wheel PID and read "
                "here for odometry. Whoever changed one must change the other the same day "
                "(docs/HARDWARE.md §7). Refusing to run: odometry computed against the wrong "
                "constant does not crash, it just makes every distance and heading wrong by a "
                "fixed percentage."
            )

    def open(
        self,
        boot_wait_s: float = 2.0,
        handshake_timeout_s: float = 3.0,
        pump: Callable[[], None] | None = None,
    ) -> Info:
        """Section 7: wait for boot, flush, handshake, verify.

        Does not send ``E``. Enabling the motors is a decision for whoever is
        about to move the robot, not a side effect of opening a port.
        """
        if boot_wait_s > 0 and pump is None:
            time.sleep(boot_wait_s)
        flush = getattr(self.transport, "flush_input", None)
        if flush is not None:
            flush()
        self.reader.reset()
        return self.handshake(timeout_s=handshake_timeout_s, pump=pump)

    # -- shutdown -------------------------------------------------------

    def shutdown(self) -> None:
        """``S`` then ``D``, best effort, safe to call more than once.

        Wire this into an ``atexit`` handler and every exception path in the
        control loop. An unclean exit still has to stop the wheels.
        """
        for send in (self.stop, self.disable):
            try:
                send()
            except Exception:
                pass

    def close(self) -> None:
        self.shutdown()
        try:
            self.transport.close()
        except Exception:
            pass

    def __enter__(self) -> "Esp32Link":
        return self

    def __exit__(self, *exc: object) -> None:
        self.close()
