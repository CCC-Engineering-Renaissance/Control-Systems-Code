import socket
import os
import math
from inputs import get_gamepad, devices
import threading
import time

class XboxController(object):
    Max_Trig_Val = math.pow(2, 8)
    Max_Joy_Val = math.pow(2, 15)

    def __init__(self, gamepad):
        self.gamepad = gamepad
        self.LeftJoystickY = 0
        self.LeftJoystickX = 0
        self.RightJoystickY = 0
        self.RightJoystickX = 0
        self.LeftTrigger = 0
        self.RightTrigger = 0
        self.LeftBumper = 0
        self.RightBumper = 0
        self.A = 0
        self.X = 0
        self.Y = 0
        self.B = 0
        self.LeftThumb = 0
        self.RightBumper = 0
        self.Back = 0
        self.Start = 0
        self.Start = 0
        self.LeftDPad = 0
        self.RightDPad = 0
        self.UpDPad = 0
        self.DownDPad = 0
        joy  = 1.0 / self.Max_Joy_Val
        trig = 1.0 / self.Max_Trig_Val
        # maps all event vals to self gamepad stuff
        self._event_map = {
            "ABS_Y": ("LeftJoystickY",  joy),
            "ABS_X": ("LeftJoystickX",  joy),
            "ABS_RY": ("RightJoystickY", joy),
            "ABS_RX": ("RightJoystickX", joy),

            "ABS_Z":  ("LeftTrigger",  trig),
            "ABS_RZ": ("RightTrigger", trig),

            "BTN_TL": ("LeftBumper",  1.0),
            "BTN_TR": ("RightBumper", 1.0),

            "BTN_SOUTH": ("A", 1.0),
            "BTN_NORTH": ("Y", 1.0),
            "BTN_WEST":  ("X", 1.0),
            "BTN_EAST":  ("B", 1.0),

            "BTN_THUMBL": ("LeftThumb",  1.0),
            "BTN_THUMBR": ("RightThumb", 1.0),

            "BTN_SELECT": ("Back",  1.0),
            "BTN_START":  ("Start", 1.0),

            "BTN_TRIGGER_HAPPY1": ("LeftDPad",  1.0),
            "BTN_TRIGGER_HAPPY2": ("RightDPad", 1.0),
            "BTN_TRIGGER_HAPPY3": ("UpDPad",    1.0),
            "BTN_TRIGGER_HAPPY4": ("DownDPad",  1.0),
        }

        self._monitor_thread = threading.Thread(target=self._monitor_controller, args=(), daemon=True)
        self._monitor_thread.start()

    def read(self):
        x = self.LeftJoystickX
        y = self.LeftJoystickY
        a = self.A
        b = self.X
        rb = self.RightBumper
        return [x, y, a, b, rb]

    def apply_deadzone(self, value, dz = 0.05):
        return 0 if abs(value) < dz else value #deadzones added no stick drift

    def smooth(self, prev, new, factor=0.2):
        return prev * (1- factor) + new * factor

    def _monitor_controller(self):
        while True:
            for event in self.gamepad.read():
                mapping = self._event_map.get(event.code)
                if mapping is None:
                    continue  # ignore events we don't care about

                attr, mul = mapping
                # Example:
                #   event.code="ABS_Y" -> attr="LeftJoystickY", mul=1/MAX_JOY_VAL
                #   event.code="BTN_SOUTH" -> attr="A", mul=1
                setattr(self, attr, event.state * mul)

SERVER_IP = ''
SERVER_PORT = 

            
joyROV = XboxController(devices.gamepads[0])
joyClaw = XboxController(devices.gamepads[1])

pushed = False
pitchAngle = 0
yawAngle = 0
als = False

with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
    sock.settimeout(2.0)

    while True:
        if als:
            pitchAngle += pow(joyROV.RightJoystickY, 3) * 0.001
            yawAngle += pow(joyROV.RightJoystickX, 3) * 0.001

        if joyROV.Y == 1 and not pushed:
            als = not als
        pushed = (joyROV.Y == 1)

        if pitchAngle < -180:
            pitchAngle += 360
        elif pitchAngle > 180:
            pitchAngle -= 360

        if yawAngle < -180:
            yawAngle += 360
        elif yawAngle > 180:
            yawAngle -= 360

        scale = 0.5
        if joyROV.A == 1:
            scale = 1.0
        if joyROV.B == 1:
            scale = 0.25
        if joyROV.X == 1:
            scale = 1.5

        out = 1 if als else 0

        MESSAGE = (
            str(joyROV.LeftJoystickY * scale * 1.5) + " " +
            str(joyROV.LeftJoystickX * scale * -1) + " " +
            str((joyROV.RightTrigger - joyROV.LeftTrigger) / -3.0 * scale) + " " +
            str(joyROV.RightJoystickX * 0.66 * scale * -2) + " " +
            str(joyROV.RightJoystickY * 0.66 * scale * 2) + " " +
            str((joyROV.RightBumper - joyROV.LeftBumper) * scale) + " " +
            str(round(pow(joyClaw.RightJoystickY, 3), 1) * 0.4) + " " +
            str(int(joyClaw.B) - int(joyClaw.A) * 1.4) + " " +
            str(round(pow(joyClaw.RightJoystickX, 3), 1) * 0.15) + " " +
            str(pow(joyClaw.LeftJoystickY, 3) * -0.25) + " " +
            str(pitchAngle) + " " +
            str(yawAngle) + " " +
            str(out)
        )

        print(
            f"X: {joyROV.LeftJoystickY:.2f}",
            f"Y: {joyROV.LeftJoystickX:.2f}",
            f"Z: {(joyROV.RightTrigger - joyROV.LeftTrigger) / 4.0:.2f}",
            f"Roll: {joyROV.RightBumper - joyROV.LeftBumper}",
            f"Pitch: {joyROV.RightJoystickY:.2f}",
            f"Yaw: {joyROV.RightJoystickX:.2f}",
            f"Claw Pitch: {pow(joyClaw.RightJoystickY * -1, 3):.1f}",
            f"Claw Open: {joyClaw.RightTrigger - joyClaw.LeftTrigger:.1f}",
            f"Claw Yaw: {joyClaw.RightJoystickX:.1f}",
            f"Claw Rotate: {joyClaw.LeftJoystickX:.1f}",
            f"PitchAngle: {pitchAngle:.1f}",
            f"YawAngle: {yawAngle:.1f}",
            als
        )

        sock.sendto(MESSAGE.encode(), (SERVER_IP, SERVER_PORT))

        time.sleep(0.01)





