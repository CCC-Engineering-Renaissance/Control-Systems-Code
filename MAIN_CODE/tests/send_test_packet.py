"""Sends one known test vector to the C++ UDP server using the REAL
thruster.py _build_packet(), for tests/udp_roundtrip_check.cpp.

Stubs pygame (same approach as tests/test_packet_transfer.py) so thruster.py
can be imported headless without controllers attached.
"""

import os
import socket
import sys
import time
import types

# ── Headless pygame stub (thruster.py imports pygame at module level) ──
_fake_pygame = types.ModuleType("pygame")
_fake_pygame.init = lambda: None
_fake_pygame.quit = lambda: None
_fake_joystick = types.ModuleType("pygame.joystick")
_fake_joystick.init = lambda: None
_fake_joystick.get_count = lambda: 0
class _FakeJoystick:  # noqa: E301
    pass
_fake_joystick.Joystick = _FakeJoystick
_fake_pygame.joystick = _fake_joystick
_fake_display = types.ModuleType("pygame.display")
_fake_display.set_mode = lambda *a, **kw: None
_fake_display.set_caption = lambda *a, **kw: None
_fake_pygame.display = _fake_display
_fake_event = types.ModuleType("pygame.event")
_fake_event.pump = lambda: None
_fake_event.get = lambda: []
_fake_pygame.event = _fake_event
_fake_pygame.QUIT = 256
_fake_pygame.KEYDOWN = 768
_fake_pygame.K_ESCAPE = 27
sys.modules.setdefault("pygame", _fake_pygame)
sys.modules.setdefault("pygame.joystick", _fake_joystick)
sys.modules.setdefault("pygame.display", _fake_display)
sys.modules.setdefault("pygame.event", _fake_event)

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", ".."))
from thruster import _build_packet  # noqa: E402

# Must match the expected values in udp_roundtrip_check.cpp
packet = _build_packet(
    0.5,     # forward        (left stick up)
    -0.25,   # strafe         (left stick left)
    0.6,     # vertical       (right trigger = ascend)
    0.3,     # yaw            (right stick right = twist right)
    0.2,     # pitch          (right stick back = nose up)
    0.75,    # roll           (right bumper = roll right)
    1,       # claw_rotate
    -1,      # claw_open
    0.9,     # claw_brushless
    pitch_angle=0.1,
    yaw_angle=-0.2,
    als=1,
)

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
# Send a short burst so the C++ side can't miss it while starting up.
for _ in range(20):
    sock.sendto(packet, ("127.0.0.1", 5007))
    time.sleep(0.05)
print("sent:", packet)
