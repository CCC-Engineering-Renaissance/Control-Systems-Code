# laptop_send_rjoy.py
# Sends RIGHT joystick Y to Raspberry Pi in full 13-field format
#
# Install:
#   pip install inputs
#
# Run:
#   python3 laptop_send_rjoy.py

import socket
import time
from inputs import get_gamepad


PI_IP = "10.0.0.195"
PORT  = 5005   # must match Constants.h

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

MAX_JOY = 32767.0
right_y = 0.0

def clamp(x, lo=-1.0, hi=1.0):
    return lo if x < lo else hi if x > hi else x

def normalize_axis(v_raw):
    v = float(v_raw) / MAX_JOY
    return clamp(v)

print(f"Sending right joystick Y to {PI_IP}:{PORT} ...")

SEND_HZ = 50
period = 1.0 / SEND_HZ
last_send = 0.0

while True:
    events = get_gamepad()

    for e in events:
        if e.code == "ABS_RY":
            # invert so pushing forward = positive
            right_y = -normalize_axis(e.state)

    now = time.time()
    if now - last_send >= period:

        # Format must match connection.cpp parsing order:
        # forward strafe vertical yaw pitch roll
        # clawPitch clawOpen claw1Open clawRotate
        # pitchAngle yawAngle alsInt

        msg = f"0 0 0 0 {right_y:.3f} 0 0 0 0 0 0 0 0\n".encode("utf-8")

        sock.sendto(msg, (PI_IP, PORT))
        last_send = now
