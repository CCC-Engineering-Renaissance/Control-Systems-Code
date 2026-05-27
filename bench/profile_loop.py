"""
bench/profile_loop.py
=====================
Simulates 10 000 control-loop iterations (≈ 5.5 min of real flight at 30 Hz)
and profiles which functions consume the most CPU.

Run:
    source venv/bin/activate
    python bench/profile_loop.py          # cProfile report
    python bench/profile_loop.py --time   # wall-clock benchmark only
"""

import sys
import os
# Make sure the project root (parent of bench/) is on sys.path so that
# `import thruster` works regardless of the cwd the script is launched from.
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import types
import cProfile
import pstats
import io
import time
import random
import argparse

# ── Headless pygame mock (same as test suite) ────────────────────────────────
_fake_pygame = types.ModuleType("pygame")
_fake_pygame.init  = lambda: None
_fake_pygame.quit  = lambda: None
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

from thruster import apply_deadzone, smooth, _build_packet, AxisFilter  # noqa: E402

# ── Synthetic axis generator ──────────────────────────────────────────────────
rng = random.Random(42)

def fake_raw(prev: float) -> float:
    """Simulate a raw hardware reading: Gaussian random walk clamped to [-1, 1]."""
    v = prev + rng.gauss(0, 0.05)
    return 1.0 if v > 1.0 else -1.0 if v < -1.0 else v


# ── Baseline: two separate function calls per axis ────────────────────────────

def hot_loop_baseline(n: int = 10_000) -> None:
    """Baseline: apply_deadzone() + smooth() called separately each frame."""
    ljy = ljx = lt = rt = rjy = rjx = cb = 0.0

    for _ in range(n):
        ljy = smooth(ljy, apply_deadzone(fake_raw(ljy), 0.10, 0.05), 1.0)
        ljx = smooth(ljx, apply_deadzone(fake_raw(ljx), 0.10, 0.05), 1.0)
        lt  = smooth(lt,  apply_deadzone(fake_raw(lt),  0.05, 0.05), 1.0)
        rt  = smooth(rt,  apply_deadzone(fake_raw(rt),  0.05, 0.05), 1.0)
        rjy = smooth(rjy, apply_deadzone(fake_raw(rjy), 0.10, 0.05), 1.0)
        rjx = smooth(rjx, apply_deadzone(fake_raw(rjx), 0.10, 0.05), 1.0)
        cb  = smooth(cb,  apply_deadzone(fake_raw(cb),  0.05, 0.05), 1.0)

        scale = 0.75
        vert  = (lt - rt) * scale
        _build_packet(ljy*scale, ljx*scale, vert, rjx*scale, rjy*scale*-1.0,
                      0.0, 0, 0, cb)


# ── Optimised: AxisFilter.__call__ per axis ───────────────────────────────────

def hot_loop_optimised(n: int = 10_000) -> None:
    """PGO: one AxisFilter object per axis, one call per frame."""
    f_ljy = AxisFilter(dz=0.10, outer_dz=0.05, factor=1.0)
    f_ljx = AxisFilter(dz=0.10, outer_dz=0.05, factor=1.0)
    f_lt  = AxisFilter(dz=0.05, outer_dz=0.05, factor=1.0)
    f_rt  = AxisFilter(dz=0.05, outer_dz=0.05, factor=1.0)
    f_rjy = AxisFilter(dz=0.10, outer_dz=0.05, factor=1.0)
    f_rjx = AxisFilter(dz=0.10, outer_dz=0.05, factor=1.0)
    f_cb  = AxisFilter(dz=0.05, outer_dz=0.05, factor=1.0)

    ljy = ljx = lt = rt = rjy = rjx = cb = 0.0

    for _ in range(n):
        ljy = f_ljy(fake_raw(ljy))
        ljx = f_ljx(fake_raw(ljx))
        lt  = f_lt( fake_raw(lt))
        rt  = f_rt( fake_raw(rt))
        rjy = f_rjy(fake_raw(rjy))
        rjx = f_rjx(fake_raw(rjx))
        cb  = f_cb( fake_raw(cb))

        scale = 0.75
        vert  = (lt - rt) * scale
        _build_packet(ljy*scale, ljx*scale, vert, rjx*scale, rjy*scale*-1.0,
                      0.0, 0, 0, cb)


# ── Entry point ──────────────────────────────────────────────────────────────

def _bench(label: str, fn, n: int) -> float:
    t0 = time.perf_counter()
    fn(n)
    elapsed = time.perf_counter() - t0
    per_iter = elapsed / n * 1e6
    print(f"  {label:<25s}  {elapsed*1000:7.1f} ms  |  {per_iter:6.2f} µs/iter")
    return elapsed


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--profile", action="store_true", help="cProfile report on optimised loop")
    parser.add_argument("--n", type=int, default=10_000)
    args = parser.parse_args()

    n = args.n

    if args.profile:
        pr = cProfile.Profile()
        pr.enable()
        hot_loop_optimised(n)
        pr.disable()
        buf = io.StringIO()
        pstats.Stats(pr, stream=buf).sort_stats("cumulative").print_stats(20)
        print(buf.getvalue())
        return

    print(f"\n{'─'*60}")
    print(f"  Benchmark: {n:,} control-loop iterations")
    print(f"{'─'*60}")
    t_base = _bench("baseline (2 fn calls/axis)", hot_loop_baseline, n)
    t_opt  = _bench("optimised (AxisFilter)",     hot_loop_optimised, n)
    speedup = t_base / t_opt
    saving  = (1.0 - t_opt / t_base) * 100
    print(f"{'─'*60}")
    print(f"  Speed-up: {speedup:.2f}×  ({saving:.0f} % faster)\n")


if __name__ == "__main__":
    main()
