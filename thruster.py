import math
import socket
import sys
import time
import pygame

PI_IP = "192.168.8.128"
PORT = 5005
SEND_HZ = 20

# Xbox-style button mapping for many pygame controller setups
BTN_A     = 0
BTN_B     = 1
BTN_X     = 2
BTN_Y     = 3
BTN_LB    = 4
BTN_RB    = 5
BTN_BACK  = 6
BTN_START = 7

ALS_DZ = 0.15  # tighter deadzone for angle accumulation

def clamp(v, lo=-1.0, hi=1.0):
    return lo if v < lo else hi if v > hi else v

def apply_deadzone(value, dz=0.05):
    if abs(value) < dz:
        return 0.0
    # Rescale so output starts at 0.0 right at the deadzone edge, no lurch
    sign = 1.0 if value > 0 else -1.0
    return sign * (abs(value) - dz) / (1.0 - dz)

def smooth(prev, new, factor=0.2):
    return prev * (1.0 - factor) + new * factor

class XboxController:
    def __init__(self, joystick: pygame.joystick.Joystick):
        self.js = joystick
        self.filtered = {
            "LeftJoystickX":  0.0,
            "LeftJoystickY":  0.0,
            "RightJoystickX": 0.0,
            "RightJoystickY": 0.0,
            "LeftTrigger":    0.0,
            "RightTrigger":   0.0,
        }

    def _get_axis_raw(self, name: str) -> float:
        if name == "LeftJoystickX":
            return self.js.get_axis(0)
        if name == "LeftJoystickY":
            return -self.js.get_axis(1)
        if name == "RightJoystickX":
            return self.js.get_axis(3)
        if name == "RightJoystickY":
            return -self.js.get_axis(4)
        if name == "LeftTrigger":
            return (self.js.get_axis(2) + 1.0) / 2.0
        if name == "RightTrigger":
            return (self.js.get_axis(5) + 1.0) / 2.0
        return 0.0

    def axis(self, name, dz=0.10, factor=0.2):
        raw = self._get_axis_raw(name)
        raw = apply_deadzone(raw, dz)
        prev = self.filtered.get(name, 0.0)
        val = smooth(prev, raw, factor)
        self.filtered[name] = val
        return val

    @property
    def A(self): return self.js.get_button(BTN_A)
    @property
    def B(self): return self.js.get_button(BTN_B)
    @property
    def X(self): return self.js.get_button(BTN_X)
    @property
    def Y(self): return self.js.get_button(BTN_Y)
    @property
    def LeftBumper(self):  return self.js.get_button(BTN_LB)
    @property
    def RightBumper(self): return self.js.get_button(BTN_RB)

def get_controllers():
    pygame.init()
    pygame.joystick.init()

    count = pygame.joystick.get_count()
    if count < 2:
        raise RuntimeError(f"Need 2 gamepads (ROV + Claw). Found {count}")

    js0 = pygame.joystick.Joystick(1)
    js1 = pygame.joystick.Joystick(0)
    js0.init()
    js1.init()

    print("ROV controller:",  js0.get_name())
    print("Claw controller:", js1.get_name())

    return XboxController(js0), XboxController(js1)

def main():
    joyROV, joyClaw = get_controllers()

    pushed      = False
    pitchAngle  = 0.0
    yawAngle    = 0.0
    als         = False

    period = 1.0 / SEND_HZ
    last   = 0.0

    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        print(f"Sending to {PI_IP}:{PORT}")

        while True:
            pygame.event.pump()

            # Read right stick for ALS angle accumulation with tighter deadzone
            rjy_raw = joyROV._get_axis_raw("RightJoystickY")
            rjx_raw = joyROV._get_axis_raw("RightJoystickX")

            if als:
                rjy_als = apply_deadzone(rjy_raw, dz=ALS_DZ)
                rjx_als = apply_deadzone(rjx_raw, dz=ALS_DZ)
                pitchAngle += (rjy_als ** 3) * 0.001
                yawAngle   += (rjx_als ** 3) * 0.001

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

            ljy = joyROV.axis("LeftJoystickY",   dz=0.10, factor=0.2)
            ljx = joyROV.axis("LeftJoystickX",   dz=0.10, factor=0.2)
            lt  = joyROV.axis("LeftTrigger",      dz=0.05, factor=0.2)
            rt  = joyROV.axis("RightTrigger",     dz=0.05, factor=0.2)
            rjy = joyROV.axis("RightJoystickY",   dz=0.10, factor=0.2)
            rjx = joyROV.axis("RightJoystickX",   dz=0.10, factor=0.2)

            claw_rjy = joyClaw.axis("RightJoystickY", dz=0.10, factor=0.2)
            claw_rjx = joyClaw.axis("RightJoystickX", dz=0.10, factor=0.2)
            claw_ljy = joyClaw.axis("LeftJoystickY",  dz=0.10, factor=0.2)

            msg = (
                f"{ljy * scale * 1.5} "
                f"{ljx * scale * -1} "
                f"{(rt - lt) / -3.0 * scale} "
                f"{rjx * 0.66 * scale * -2} "
                f"{rjy * 0.66 * scale * 2} "
                f"{(joyROV.RightBumper - joyROV.LeftBumper) * scale} "
                f"{round((claw_rjx ** 3), 1) * 0.15} "
                f"{(int(joyClaw.B) - int(joyClaw.A)) * 1.4} "
                f"{round((claw_rjy ** 3), 1) * 0.4} "
                f"{(claw_ljy ** 3) * -0.25} "
                f"{pitchAngle} "
                f"{yawAngle} "
                f"{out}\n"
            )

            now = time.time()
            if now - last >= period:
                sock.sendto(msg.encode(), (PI_IP, PORT))
                last = now

                print(
                    f"X: {ljy:.2f}",
                    f"Y: {ljx:.2f}",
                    f"Z: {(rt - lt) / 4.0:.2f}",
                    f"Roll: {joyROV.RightBumper - joyROV.LeftBumper}",
                    f"Pitch: {rjy:.2f}",
                    f"Yaw: {rjx:.2f}",
                    f"Claw Pitch: {(-claw_rjy) ** 3:.1f}",
                    f"Claw Open: {(int(joyClaw.B) - int(joyClaw.A)):.1f}",
                    f"Claw Yaw: {claw_rjx:.1f}",
                    f"Claw Rotate: {claw_ljy:.1f}",
                    f"PitchAngle: {pitchAngle:.1f}",
                    f"YawAngle: {yawAngle:.1f}",
                    als
                )

            time.sleep(0.001)

if __name__ == "__main__":
    try:
        main()
    except Exception as exc:
        print(f"thruster.py error: {exc}", file=sys.stderr)
        raise SystemExit(1)
