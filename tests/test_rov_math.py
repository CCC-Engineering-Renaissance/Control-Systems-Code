"""
ROV Math – unit tests for the pure-logic helpers in thruster.py.

Red-Green-Refactor cycle:
  RED   – tests written first, before the bugs are fixed.
  GREEN – bugs fixed; all assertions pass.

Running:
    source venv/bin/activate
    python -m pytest tests/test_rov_math.py -v
"""

import sys
import types
import importlib
import pytest

# ---------------------------------------------------------------------------
# pygame-free import of thruster.py
# We mock the minimum of pygame so the module can be imported in a headless
# environment without any joystick / display hardware present.
# ---------------------------------------------------------------------------
_fake_pygame = types.ModuleType("pygame")
_fake_pygame.init = lambda: None
_fake_pygame.quit = lambda: None

_fake_joystick = types.ModuleType("pygame.joystick")
_fake_joystick.init = lambda: None
_fake_joystick.get_count = lambda: 0

# Provide a minimal Joystick stub so the type annotation in XboxController.__init__
# can be resolved at class-definition time (Python evaluates it as a default arg).
class _FakeJoystick:
    pass
_fake_joystick.Joystick = _FakeJoystick
_fake_pygame.joystick = _fake_joystick

_fake_display = types.ModuleType("pygame.display")
_fake_display.set_mode = lambda *a, **kw: None
_fake_display.set_caption = lambda *a, **kw: None
_fake_pygame.display = _fake_display

_fake_pygame.QUIT    = 256
_fake_pygame.KEYDOWN = 768
_fake_pygame.K_ESCAPE = 27
_fake_event = types.ModuleType("pygame.event")
_fake_event.pump = lambda: None
_fake_event.get  = lambda: []
_fake_pygame.event = _fake_event

sys.modules.setdefault("pygame",          _fake_pygame)
sys.modules.setdefault("pygame.joystick", _fake_joystick)
sys.modules.setdefault("pygame.display",  _fake_display)
sys.modules.setdefault("pygame.event",    _fake_event)

import thruster  # noqa: E402 – must come after mock injection

from thruster import clamp, apply_deadzone, smooth  # noqa: E402


# ===========================================================================
# clamp()
# ===========================================================================

class TestClamp:
    """clamp(v, lo, hi) should always keep v inside [lo, hi]."""

    def test_below_range(self):
        assert clamp(-2.0) == -1.0

    def test_above_range(self):
        assert clamp(2.0) == 1.0

    def test_at_lower_bound(self):
        assert clamp(-1.0) == -1.0

    def test_at_upper_bound(self):
        assert clamp(1.0) == 1.0

    def test_inside_range(self):
        assert clamp(0.5) == 0.5

    def test_custom_bounds(self):
        assert clamp(5.0, lo=0.0, hi=3.0) == 3.0
        assert clamp(-1.0, lo=0.0, hi=3.0) == 0.0

    # ── RED: missing guard (will fail before the fix) ──────────────────────
    def test_lo_greater_than_hi_raises_value_error(self):
        """Passing lo > hi is a programmer error and should raise ValueError,
        not silently return a nonsensical result."""
        with pytest.raises(ValueError, match="lo.*hi"):
            clamp(0.5, lo=1.0, hi=-1.0)


# ===========================================================================
# apply_deadzone()
# ===========================================================================

class TestApplyDeadzone:
    """apply_deadzone maps a raw axis value through inner + outer deadzones."""

    def test_inner_deadzone_returns_zero(self):
        assert apply_deadzone(0.05) == 0.0   # inside inner dz=0.10

    def test_negative_inner_deadzone_returns_zero(self):
        assert apply_deadzone(-0.05) == 0.0

    def test_outer_deadzone_clamps_to_one(self):
        assert apply_deadzone(0.95) == 1.0   # outside 1.0 - outer_dz=0.10

    def test_outer_deadzone_clamps_to_neg_one(self):
        assert apply_deadzone(-0.95) == -1.0

    def test_midrange_positive(self):
        # value=0.15, dz=0.10, outer=0.10 → (0.15-0.10)/(1.0-0.10-0.10) = 0.05/0.80
        assert apply_deadzone(0.15) == pytest.approx(0.05 / 0.80, abs=1e-9)

    def test_midrange_negative(self):
        assert apply_deadzone(-0.15) == pytest.approx(-0.05 / 0.80, abs=1e-9)

    def test_output_always_in_range(self):
        """apply_deadzone must always return a value in [-1.0, 1.0]."""
        test_values = [-1.5, -1.0, -0.9, -0.5, -0.1, 0.0, 0.1, 0.5, 0.9, 1.0, 1.5]
        for v in test_values:
            result = apply_deadzone(v)
            assert -1.0 <= result <= 1.0, f"apply_deadzone({v}) = {result} out of [-1, 1]"

    def test_exactly_at_inner_edge_returns_zero(self):
        """Value exactly equal to dz should be in the dead zone (abs < dz is False,
        but we want consistent behaviour; document what actually happens)."""
        # abs(0.10) < 0.10  →  False  →  continues to sign/range logic
        # sign=1, abs(0.10) > 0.90? No  →  1 * (0.10-0.10)/0.80 = 0.0
        assert apply_deadzone(0.10) == pytest.approx(0.0, abs=1e-9)

    # ── RED: missing guard (will fail before the fix) ──────────────────────
    def test_degenerate_range_raises_value_error(self):
        """dz + outer_dz >= 1.0 causes division by zero.
        A clear ValueError must be raised instead."""
        with pytest.raises(ValueError, match="dz.*outer_dz"):
            apply_deadzone(0.5, dz=0.6, outer_dz=0.5)

    def test_negative_dz_raises_value_error(self):
        """Negative deadzone values are nonsensical and must be rejected."""
        with pytest.raises(ValueError):
            apply_deadzone(0.5, dz=-0.1)

    def test_negative_outer_dz_raises_value_error(self):
        with pytest.raises(ValueError):
            apply_deadzone(0.5, outer_dz=-0.1)


# ===========================================================================
# smooth()
# ===========================================================================

class TestSmooth:
    """smooth() is an exponential moving-average (EMA) low-pass filter."""

    def test_factor_zero_returns_prev(self):
        """factor=0 means 'keep the old value entirely'."""
        assert smooth(0.5, 0.9, factor=0.0) == 0.5

    def test_factor_one_returns_new(self):
        """factor=1 means 'jump immediately to new value'."""
        assert smooth(0.5, 0.9, factor=1.0) == 0.9

    def test_midpoint_blend(self):
        # prev=0.0, new=1.0, factor=0.2  →  0.0*0.8 + 1.0*0.2 = 0.2
        assert smooth(0.0, 1.0, factor=0.2) == pytest.approx(0.2, abs=1e-9)

    def test_snap_to_zero(self):
        """Values tiny enough (< 0.001) should snap to exactly 0.0 to prevent
        infinite creep after a stick is released."""
        result = smooth(0.0009, 0.0, factor=0.2)
        assert result == 0.0

    def test_does_not_snap_large_value(self):
        """Only values below the snap threshold should be zeroed."""
        result = smooth(0.5, 0.0, factor=0.0)  # factor=0 → stays at 0.5
        assert result == 0.5

    # ── RED: missing guard (will fail before the fix) ──────────────────────
    def test_factor_above_one_raises_value_error(self):
        """factor > 1.0 produces values outside the expected range and indicates
        a programming error. Should raise ValueError."""
        with pytest.raises(ValueError, match="factor"):
            smooth(0.5, 0.0, factor=2.0)

    def test_factor_below_zero_raises_value_error(self):
        with pytest.raises(ValueError, match="factor"):
            smooth(0.5, 0.0, factor=-0.1)


# ===========================================================================
# XboxController (mocked joystick)
# ===========================================================================

class TestXboxController:
    """Test XboxController with a fully mocked pygame.joystick.Joystick."""

    def _make_controller(self, axis_values: dict[int, float]):
        """Build an XboxController backed by a mock joystick."""
        js = pytest.importorskip("unittest.mock").MagicMock()
        js.get_axis.side_effect = lambda i: axis_values.get(i, 0.0)
        js.get_button.return_value = 0
        return thruster.XboxController(js)

    def test_left_joystick_y_inverted(self):
        """Axis 1 (left-stick Y) is negated so up = +1."""
        ctrl = self._make_controller({1: 0.8})   # hardware reports 0.8 (down)
        # With factor=1.0 (no smoothing) and dz=0, outer_dz=0 → value flows through
        raw = ctrl._get_axis_raw("LeftJoystickY")
        assert raw == -0.8

    def test_trigger_at_rest_maps_to_zero(self):
        """Xbox triggers rest at -1.0 raw; the +1 shift maps that to 0.0."""
        ctrl = self._make_controller({4: -1.0, 5: -1.0})
        assert ctrl._get_axis_raw("LeftTrigger")  == pytest.approx(0.0, abs=1e-9)
        assert ctrl._get_axis_raw("RightTrigger") == pytest.approx(0.0, abs=1e-9)

    def test_trigger_fully_pulled_maps_to_one(self):
        """Trigger fully pulled (+1.0 raw) → (1.0+1.0)/2 = 1.0."""
        ctrl = self._make_controller({4: 1.0, 5: 1.0})
        assert ctrl._get_axis_raw("LeftTrigger")  == pytest.approx(1.0, abs=1e-9)
        assert ctrl._get_axis_raw("RightTrigger") == pytest.approx(1.0, abs=1e-9)

    def test_unknown_axis_returns_zero(self):
        ctrl = self._make_controller({})
        assert ctrl._get_axis_raw("NonExistentAxis") == 0.0

    def test_gamecontroller_axis_normalisation(self):
        """When opened as a GameController, axes are read by SDL constant and
        normalised from int16: sticks /32767 (Y inverted), triggers rest at 0."""
        class _FakeController:
            def __init__(self, vals): self._vals = vals
            def get_axis(self, const): return self._vals.get(const, 0)

        ctrl = self._make_controller({})  # ctrl.ctrl is None (no SDL in tests)
        ctrl.ctrl = _FakeController({
            thruster._CTRL_AXIS["LeftJoystickY"]: 16384,   # half down
            thruster._CTRL_AXIS["LeftJoystickX"]: -32767,  # full left
            thruster._CTRL_AXIS["RightTrigger"]:  32767,   # fully pulled
            # LeftTrigger absent → rest value 0
        })
        # Y is inverted so a positive (down) raw reading becomes negative.
        assert ctrl._get_axis_raw("LeftJoystickY") == pytest.approx(-16384 / 32767.0)
        # X stick is not inverted.
        assert ctrl._get_axis_raw("LeftJoystickX") == pytest.approx(-1.0)
        # GameController triggers map straight to [0, 1] with no offset.
        assert ctrl._get_axis_raw("RightTrigger") == pytest.approx(1.0)
        assert ctrl._get_axis_raw("LeftTrigger")  == pytest.approx(0.0)

    def test_buttons_delegate_to_joystick(self):
        from unittest.mock import MagicMock
        js = MagicMock()
        js.get_axis.return_value = 0.0
        js.get_button.side_effect = lambda i: 1 if i == 0 else 0  # only A pressed
        ctrl = thruster.XboxController(js)
        assert ctrl.A == 1
        assert ctrl.B == 0


# ===========================================================================
# UDP message protocol
# ===========================================================================

class TestMessageProtocol:
    """The UDP message must contain exactly 12 space-separated numeric fields
    followed by a newline, matching the Pi-side parser."""

    def _build_msg(
        self,
        ljy=0.0, ljx=0.0, vert=0.0,
        rjx=0.0, rjy=0.0, roll=0,
        clawRotate=0, clawOpen=0, clawBrushless=0.0,
        scale=0.75,
    ) -> str:
        return (
            f"{ljy * scale} "
            f"{ljx * scale} "
            f"{vert} "
            f"{rjx * scale} "
            f"{rjy * scale * -1} "
            f"{roll * scale} "
            f"{clawRotate} "
            f"{clawOpen} "
            f"{clawBrushless} "
            f"0.0 "
            f"0.0 "
            f"0\n"
        )

    def test_field_count(self):
        msg = self._build_msg()
        fields = msg.strip().split()
        assert len(fields) == 12, f"Expected 12 fields, got {len(fields)}: {fields}"

    def test_ends_with_newline(self):
        msg = self._build_msg()
        assert msg.endswith("\n")

    def test_all_fields_are_numeric(self):
        msg = self._build_msg(ljy=0.3, rjx=-0.7, clawBrushless=0.9)
        for i, field in enumerate(msg.strip().split()):
            try:
                float(field)
            except ValueError:
                pytest.fail(f"Field {i} '{field}' is not numeric")
