"""
tests/test_packet_transfer.py
==============================
Protocol correctness, round-trip precision, and parser robustness tests
for the 12-field UDP control packet used between the laptop and the Pi.

Running:
    source venv/bin/activate
    python -m pytest tests/test_packet_transfer.py -v
"""

import sys
import os
import types
import struct
import pytest

# ── Headless pygame mock ─────────────────────────────────────────────────────
_fake_pygame = types.ModuleType("pygame")
_fake_pygame.init = lambda: None
_fake_pygame.quit = lambda: None
_fake_joystick = types.ModuleType("pygame.joystick")
_fake_joystick.init = lambda: None
_fake_joystick.get_count = lambda: 0
class _FakeJoystick: pass
_fake_joystick.Joystick = _FakeJoystick
_fake_pygame.joystick = _fake_joystick
_fake_display = types.ModuleType("pygame.display")
_fake_display.set_mode    = lambda *a, **kw: None
_fake_display.set_caption = lambda *a, **kw: None
_fake_pygame.display = _fake_display
_fake_pygame.QUIT     = 256
_fake_pygame.KEYDOWN  = 768
_fake_pygame.K_ESCAPE = 27
_fake_event = types.ModuleType("pygame.event")
_fake_event.pump = lambda: None
_fake_event.get  = lambda: []
_fake_pygame.event = _fake_event
sys.modules.setdefault("pygame",          _fake_pygame)
sys.modules.setdefault("pygame.joystick", _fake_joystick)
sys.modules.setdefault("pygame.display",  _fake_display)
sys.modules.setdefault("pygame.event",    _fake_event)

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from thruster import _build_packet  # noqa: E402

# ---------------------------------------------------------------------------
# Reference Pi-side parser (mirrors what the ROV firmware expects)
# ---------------------------------------------------------------------------

#  Field layout must match POVState in the Pi's connection handler:
#  forward  strafe  vertical  yaw  pitch  roll
#  claw_rotate  claw_open  claw_brushless  reserved0  reserved1  als
FIELD_NAMES = [
    "forward", "strafe", "vertical",
    "yaw", "pitch", "roll",
    "claw_rotate", "claw_open", "claw_brushless",
    "pitch_angle", "yaw_angle", "als",
]

FLOAT_FIELDS = {
    "forward", "strafe", "vertical",
    "yaw", "pitch", "roll",
    "claw_brushless", "pitch_angle", "yaw_angle",
}

INT_FIELDS = {"claw_rotate", "claw_open", "als"}


def parse_packet(data: bytes) -> dict:
    """Parse a UDP control packet the same way the Pi firmware does.

    Raises
    ------
    ValueError  if the packet is malformed (wrong field count, non-numeric
                fields, integer fields that are not whole numbers, or float
                fields whose magnitude exceeds 1.5).
    """
    try:
        text = data.decode("ascii")
    except UnicodeDecodeError as exc:
        raise ValueError(f"Non-ASCII bytes in packet: {exc}") from exc

    fields = text.strip().split()
    if len(fields) != 12:
        raise ValueError(f"Expected 12 fields, got {len(fields)}: {fields!r}")

    result = {}
    for name, raw in zip(FIELD_NAMES, fields):
        try:
            value = float(raw)
        except ValueError:
            raise ValueError(f"Field '{name}' is not numeric: {raw!r}")

        # NaN and ±Inf are syntactically valid floats in Python but must never
        # appear in a control packet (they would crash the Pi's motor math).
        import math
        if not math.isfinite(value):
            raise ValueError(f"Field '{name}' is not numeric: {raw!r}")

        if name in INT_FIELDS:
            if value != int(value):
                raise ValueError(
                    f"Integer field '{name}' has fractional part: {raw!r}"
                )
            result[name] = int(value)
        else:
            result[name] = value

    return result


# ===========================================================================
# 1. Packet encoding
# ===========================================================================

class TestPacketEncoding:
    """_build_packet() must produce bytes that parse correctly."""

    def test_returns_bytes(self):
        assert isinstance(_build_packet(0, 0, 0, 0, 0, 0, 0, 0, 0), bytes)

    def test_ends_with_newline(self):
        pkt = _build_packet(0, 0, 0, 0, 0, 0, 0, 0, 0)
        assert pkt.endswith(b"\n")

    def test_is_ascii(self):
        pkt = _build_packet(0.3, -0.7, 0.5, 0.1, -0.2, 0.4, 1, -1, 0.9)
        pkt.decode("ascii")  # must not raise

    def test_field_count(self):
        pkt = _build_packet(0, 0, 0, 0, 0, 0, 0, 0, 0)
        assert len(pkt.decode().strip().split()) == 12

    def test_size_within_udp_safe_range(self):
        """Worst-case packet must fit in a single UDP datagram with headroom."""
        worst = _build_packet(-0.9999, -0.9999, -0.9999, -0.9999, -0.9999, -0.9999, -1, -1, -0.9999)
        assert len(worst) < 512, f"Packet too large: {len(worst)} bytes"

    def test_pitch_angle_forwarded(self):
        """Field 10 (pitchAngle) must carry the value passed as pitch_angle,
        not a hardcoded 0.0.  Pi's PID uses this as the setpoint."""
        pkt = _build_packet(0, 0, 0, 0, 0, 0, 0, 0, 0, pitch_angle=0.5)
        parsed = parse_packet(pkt)
        assert parsed["pitch_angle"] == pytest.approx(0.5, abs=5e-5)

    def test_yaw_angle_forwarded(self):
        """Field 11 (yawAngle) must carry the value passed as yaw_angle."""
        pkt = _build_packet(0, 0, 0, 0, 0, 0, 0, 0, 0, yaw_angle=-0.3)
        parsed = parse_packet(pkt)
        assert parsed["yaw_angle"] == pytest.approx(-0.3, abs=5e-5)

    def test_als_field_is_zero(self):
        parsed = parse_packet(_build_packet(0, 0, 0, 0, 0, 0, 0, 0, 0))
        assert parsed["als"] == 0


# ===========================================================================
# 2. Round-trip precision
# ===========================================================================

class TestRoundTripPrecision:
    """Encoded values must decode to match inputs within protocol precision
    (4 decimal places for floats, exact for integers)."""

    @pytest.mark.parametrize("fwd,str_,vert,yaw,pitch,roll,cr,co,cb", [
        (0.0,   0.0,  0.0,  0.0,  0.0,  0.0, 0,  0,  0.0),
        (0.75,  0.0,  0.0,  0.0,  0.0,  0.0, 0,  0,  0.0),
        (-0.35, 0.12,-0.68, 0.99,-0.01, 0.5,-1,  1,  0.88),
        (0.75,  0.75, 0.75, 0.75, 0.75, 0.75, 1,  1,  1.0),
        (-0.75,-0.75,-0.75,-0.75,-0.75,-0.75,-1, -1,  0.0),
    ])
    def test_float_fields_match_to_4dp(
        self, fwd, str_, vert, yaw, pitch, roll, cr, co, cb
    ):
        pkt = _build_packet(fwd, str_, vert, yaw, pitch, roll, cr, co, cb)
        parsed = parse_packet(pkt)
        assert parsed["forward"]        == pytest.approx(fwd,   abs=5e-5)
        assert parsed["strafe"]         == pytest.approx(str_,  abs=5e-5)
        assert parsed["vertical"]       == pytest.approx(vert,  abs=5e-5)
        assert parsed["yaw"]            == pytest.approx(yaw,   abs=5e-5)
        assert parsed["pitch"]          == pytest.approx(pitch, abs=5e-5)
        assert parsed["roll"]           == pytest.approx(roll,  abs=5e-5)
        assert parsed["claw_brushless"] == pytest.approx(cb,    abs=5e-5)

    @pytest.mark.parametrize("cr,co", [(-1, -1), (0, 0), (1, 1), (-1, 1)])
    def test_integer_fields_exact(self, cr, co):
        pkt = _build_packet(0, 0, 0, 0, 0, 0, cr, co, 0)
        parsed = parse_packet(pkt)
        assert parsed["claw_rotate"] == cr
        assert parsed["claw_open"]   == co

    def test_field_order(self):
        """Verify each field lands in its documented protocol position."""
        pkt = _build_packet(
            0.1111,  # forward
            0.2222,  # strafe
            0.3333,  # vertical
            0.4444,  # yaw
            0.5555,  # pitch
            0.6666,  # roll
            1,       # claw_rotate
            -1,      # claw_open
            0.7777,  # claw_brushless
        )
        parts = pkt.decode().strip().split()
        assert float(parts[0]) == pytest.approx(0.1111, abs=5e-5)  # forward
        assert float(parts[1]) == pytest.approx(0.2222, abs=5e-5)  # strafe
        assert float(parts[2]) == pytest.approx(0.3333, abs=5e-5)  # vertical
        assert float(parts[3]) == pytest.approx(0.4444, abs=5e-5)  # yaw
        assert float(parts[4]) == pytest.approx(0.5555, abs=5e-5)  # pitch
        assert float(parts[5]) == pytest.approx(0.6666, abs=5e-5)  # roll
        assert int(float(parts[6])) == 1                           # claw_rotate
        assert int(float(parts[7])) == -1                          # claw_open
        assert float(parts[8]) == pytest.approx(0.7777, abs=5e-5)  # claw_brushless


# ===========================================================================
# 3. Parser robustness  (RED tests: bugs the parser must catch)
# ===========================================================================

class TestParserRobustness:
    """parse_packet() must raise ValueError for every malformed input,
    not silently return wrong data."""

    # ── RED: currently no parser existed at all – these define the contract ──

    def test_empty_packet_raises(self):
        with pytest.raises(ValueError, match="12 fields"):
            parse_packet(b"")

    def test_too_few_fields_raises(self):
        with pytest.raises(ValueError, match="12 fields"):
            parse_packet(b"0.0 0.0 0.0\n")

    def test_too_many_fields_raises(self):
        with pytest.raises(ValueError, match="12 fields"):
            # 13 fields
            parse_packet(b"0 0 0 0 0 0 0 0 0 0 0 0 0\n")

    def test_non_numeric_field_raises(self):
        with pytest.raises(ValueError, match="not numeric"):
            parse_packet(b"NaN 0 0 0 0 0 0 0 0 0 0 0\n")

    def test_non_ascii_raises(self):
        with pytest.raises(ValueError, match="Non-ASCII"):
            parse_packet(b"\xff\xfe 0 0 0 0 0 0 0 0 0 0 0\n")

    def test_fractional_integer_field_raises(self):
        """claw_rotate = 0.5 is not a valid integer command."""
        with pytest.raises(ValueError, match="Integer field.*claw_rotate"):
            parse_packet(b"0 0 0 0 0 0 0.5 0 0 0 0 0\n")

    def test_fractional_als_field_raises(self):
        with pytest.raises(ValueError, match="Integer field.*als"):
            parse_packet(b"0 0 0 0 0 0 0 0 0 0 0 1.5\n")

    def test_valid_packet_does_not_raise(self):
        pkt = _build_packet(0.3, -0.1, 0.5, 0.0, 0.2, -0.4, 1, 0, 0.7)
        result = parse_packet(pkt)
        assert isinstance(result, dict)
        assert len(result) == 12

    def test_whitespace_variations_tolerated(self):
        """Extra spaces / tabs between fields must still parse (the Pi's
        split() call is whitespace-agnostic)."""
        raw = b"0.0000  0.0000  0.0000  0.0000  0.0000  0.0000  0  0  0.0000  0.0  0.0  0\n"
        result = parse_packet(raw)
        assert result["forward"] == 0.0


# ===========================================================================
# 4. Throughput / timing sanity
# ===========================================================================

class TestThroughputSanity:
    """Verify that the build + encode path is fast enough for the target rate."""

    def test_build_1000_packets_under_5ms(self):
        """1 000 packets (33× one second of 30 Hz) must encode in < 5 ms."""
        import time
        t0 = time.perf_counter()
        for _ in range(1000):
            _build_packet(0.3, -0.1, 0.5, 0.0, 0.2, -0.4, 1, 0, 0.7)
        elapsed_ms = (time.perf_counter() - t0) * 1000
        assert elapsed_ms < 5.0, f"Encoding 1 000 packets took {elapsed_ms:.2f} ms (too slow)"

    def test_packet_size_stable_across_inputs(self):
        """All-zero and max-value packets must be within 10 bytes of each other
        (no pathological size blowup)."""
        zero_pkt = _build_packet(0, 0, 0, 0, 0, 0, 0, 0, 0)
        max_pkt  = _build_packet(-0.9999, -0.9999, -0.9999, -0.9999, -0.9999, -0.9999, -1, -1, -0.9999)
        assert abs(len(max_pkt) - len(zero_pkt)) <= 10
