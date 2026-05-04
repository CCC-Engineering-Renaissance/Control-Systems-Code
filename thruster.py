import math
import socket
import sys
import time
import pygame

PI_IP = "192.168.8.128"
PORT = 5005
SEND_HZ = 20

# Xbox-style button mapping
BTN_A     = 0
BTN_B     = 1
BTN_X     = 2
BTN_Y     = 3
BTN_LB    = 4
BTN_RB    = 5
BTN_BACK  = 6
BTN_START = 7

ALS_DZ = 0.15

def clamp(v, lo=-1.0, hi=1.0):
    return lo if v < lo else hi if v > hi else v

def apply_deadzone(value, dz=0.05):
    if abs(value) < dz:
        return 0.0
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

    # Create a real window so Windows doesn't kill the process
    # and so pygame captures controller input instead of the OS
    pygame.display.set_mode((300, 100))
    pygame.display.set_caption("ROV Control - Running")

    count = pygame.joystick.get_count()
    if count < 2:
        raise RuntimeError(f"Need 2 gamepads (ROV + Claw). Found {count}. Make sure both are plugged in before starting.")

    print("Available controllers:")
    joysticks = []
    for i in range(count):
        js = pygame.joystick.Joystick(i)
        js.init()
        print(f"  [{i}] {js.get_name()}")
        joysticks.append(js)

    # Let the user pick which controller is which
    try:
        rov_idx  = int(input("Select ROV controller index: "))
        claw_idx = int(input("Select Claw controller index: "))
    except ValueError:
        raise RuntimeError("Invalid controller index entered.")

    if rov_idx == claw_idx:
        raise RuntimeError("ROV and Claw controllers must be different indices.")
    if rov_idx >= count or claw_idx >= count:
        raise RuntimeError("Controller index out of range.")

    print(f"ROV controller:  {joysticks[rov_idx].get_name()}")
    print(f"Claw controller: {joysticks[claw_idx].get_name()}")

    return XboxController(joysticks[rov_idx]), XboxController(joysticks[claw_idx])


def main():
    joyROV, joyClaw = get_controllers()

    pushed     = False
    pitchAngle = 0.0
    yawAngle   = 0.0
    als        = False

    period = 1.0 / SEND_HZ
    last   = 0.0

    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        print(f"Sending to {PI_IP}:{PORT}")
        print("Press ESC or close the window to stop.")

        while True:
            # ── Drain the event queue every loop (fixes Windows freeze/crash) ──
            pygame.event.pump()
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    print("\nWindow closed. Exiting.")
                    return
                if event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE:
                    print("\nESC pressed. Exiting.")
                    return

            # ── ALS angle accumulation (ROV right stick) ──────────────
            rjy_raw = joyROV._get_axis_raw("RightJoystickY")
            rjx_raw = joyROV._get_axis_raw("RightJoystickX")

            if als:
                rjy_als = apply_deadzone(rjy_raw, dz=ALS_DZ)
                rjx_als = apply_deadzone(rjx_raw, dz=ALS_DZ)
                pitchAngle += (rjy_als ** 3) * 0.001
                yawAngle   += (rjx_als ** 3) * 0.001

            # Y on ROV controller toggles ALS
            if joyROV.Y == 1 and not pushed:
                als = not als
            pushed = (joyROV.Y == 1)

            # Wrap angles to [-180, 180]
            if pitchAngle < -180:
                pitchAngle += 360
            elif pitchAngle > 180:
                pitchAngle -= 360

            if yawAngle < -180:
                yawAngle += 360
            elif yawAngle > 180:
                yawAngle -= 360

            # ── ROV speed scale ───────────────────────────────────────
            scale = 0.5
            if joyROV.A == 1:
                scale = 1.0
            if joyROV.B == 1:
                scale = 0.25
            if joyROV.X == 1:
                scale = 1.5

            out = 1 if als else 0

            # ── ROV axes ──────────────────────────────────────────────
            ljy = joyROV.axis("LeftJoystickY",  dz=0.10, factor=0.2)
            ljx = joyROV.axis("LeftJoystickX",  dz=0.10, factor=0.2)
            lt  = joyROV.axis("LeftTrigger",     dz=0.05, factor=0.2)
            rt  = joyROV.axis("RightTrigger",    dz=0.05, factor=0.2)
            rjy = joyROV.axis("RightJoystickY",  dz=0.10, factor=0.2)
            rjx = joyROV.axis("RightJoystickX",  dz=0.10, factor=0.2)

            # ── Claw controller ───────────────────────────────────────
            claw_ljy = joyClaw.axis("LeftJoystickY", dz=0.10, factor=0.2)

            clawRotate = int(joyClaw.X) - int(joyClaw.A)
            clawOpen   = int(joyClaw.Y) - int(joyClaw.B)
            clawPitch  = int(joyClaw.LeftBumper) - int(joyClaw.axis("LeftTrigger", dz=0.05) > 0.1)
            claw1Open  = (claw_ljy ** 3) * -0.25

            msg = (
                f"{ljy * scale * 1.5} "
                f"{ljx * scale * -1} "
                f"{(rt - lt) / -3.0 * scale} "
                f"{rjx * 0.66 * scale * -2} "
                f"{rjy * 0.66 * scale * 2} "
                f"{(joyROV.RightBumper - joyROV.LeftBumper) * scale} "
                f"{clawRotate} "
                f"{clawOpen} "
                f"{clawPitch} "
                f"{claw1Open} "
                f"{pitchAngle} "
                f"{yawAngle} "
                f"{out}\n"
            )

            now = time.time()
            if now - last >= period:
                sock.sendto(msg.encode(), (PI_IP, PORT))
                last = now

                print(
                    f"Fwd: {ljy * scale * 1.5:.2f}",
                    f"Strafe: {ljx * scale * -1:.2f}",
                    f"Vert: {(rt - lt) / -3.0 * scale:.2f}",
                    f"Yaw: {rjx * 0.66 * scale * -2:.2f}",
                    f"Pitch: {rjy * 0.66 * scale * 2:.2f}",
                    f"Roll: {joyROV.RightBumper - joyROV.LeftBumper}",
                    f"ClawRotate(X/A): {clawRotate}",
                    f"ClawOpen(Y/B): {clawOpen}",
                    f"ClawPitch(LB/LT): {clawPitch}",
                    f"Claw1Open: {claw1Open:.2f}",
                    f"PitchAngle: {pitchAngle:.1f}",
                    f"YawAngle: {yawAngle:.1f}",
                    f"ALS: {als}",
                )

            time.sleep(0.001)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nStopped by user.")
    except Exception as exc:
        print(f"\nthruster.py error: {exc}", file=sys.stderr)
        input("Press Enter to exit...")  # keeps PowerShell open so you can read the error
        raise SystemExit(1)
    finally:
        pygame.quit()