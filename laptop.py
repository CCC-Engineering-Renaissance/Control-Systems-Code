# laptop_send_rjoy.py
# Sends ONLY right joystick Y over UDP to the Raspberry Pi.
#
# Install:
#   pip install inputs
#
# Run:
#   python3 laptop_send_rjoy.py

import socket
import time
from inputs import get_gamepad

PI_IP = "10.0.0.195"   # <-- CHANGE to your Pi IP
PORT  = 5005             # must match PORT in Constants.h / connection layer

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# Xbox stick range is typically -32768..32767
MAX_JOY = 32767.0

right_y = 0.0

def clamp(x, lo=-1.0, hi=1.0):
    return lo if x < lo else hi if x > hi else x

def normalize_axis(v_raw):
    # v_raw is int in [-32768, 32767]
    v = float(v_raw) / MAX_JOY
    return clamp(v)

print(f"Sending right joystick Y to {PI_IP}:{PORT} ...")

last_send = 0.0
SEND_HZ = 50
period = 1.0 / SEND_HZ

while True:
    # read any controller events
    events = get_gamepad()
    for e in events:
        # Right stick Y is commonly "ABS_RY"
        if e.code == "ABS_RY":
            # Many controllers report up as negative; invert if you want "up=+1"
            right_y = -normalize_axis(e.state)

    now = time.time()
    if now - last_send >= period:
        # Message format: simple, explicit key=value
        # Your server/parser must read this and update POVState.pitch accordingly
        msg = f"pitch={right_y:.3f}\n".encode("utf-8")
        sock.sendto(msg, (PI_IP, PORT))
        last_send = now

