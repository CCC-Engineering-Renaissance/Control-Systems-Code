import math
import socket
import sys
import time
import pygame

PI_IP = "192.168.8.128"
PORT = 5005
SEND_HZ = 30

# Xbox-style button mapping
BTN_A     = 0
BTN_B     = 1
BTN_X     = 2
BTN_Y     = 3
BTN_LB    = 4
BTN_RB    = 5
BTN_BACK  = 6
BTN_START = 7

def clamp(v, lo=-1.0, hi=1.0):
    return lo if v < lo else hi if v > hi else v

def apply_deadzone(value, dz=0.10, outer_dz=0.10):
    if abs(value) < dz:
        return 0.0
    sign = 1.0 if value > 0 else -1.0
    if abs(value) > 1.0 - outer_dz:
        return sign
    return sign * (abs(value) - dz) / (1.0 - outer_dz - dz)

def smooth(prev, new, factor=0.2):
    result = prev * (1.0 - factor) + new * factor
    # Snap to zero to prevent infinite creep after releasing sticks
    if abs(result) < 0.001:
        result = 0.0
    return result

class XboxController:
    def __init__(self, joystick: pygame.joystick.Joystick):
        self.js = joystick
        # Prime triggers with actual hardware state at startup.
        # On some Windows drivers, triggers report 0.0 at rest instead of -1.0,
        # which makes (0.0 + 1.0) / 2.0 = 0.5 — causing a false 0.66 vertical output.
        lt_init = (self.js.get_axis(4) + 1.0) / 2.0
        rt_init = (self.js.get_axis(5) + 1.0) / 2.0
        self.filtered = {
            "LeftJoystickX":  0.0,
            "LeftJoystickY":  0.0,
            "RightJoystickX": 0.0,
            "RightJoystickY": 0.0,
            "LeftTrigger":    lt_init,
            "RightTrigger":   rt_init,
        }

    def _get_axis_raw(self, name: str) -> float:
        if name == "LeftJoystickX":
            return self.js.get_axis(0)
        if name == "LeftJoystickY":
            return -self.js.get_axis(1)
        if name == "RightJoystickX":
            return self.js.get_axis(2)
        if name == "RightJoystickY":
            return -self.js.get_axis(3)
        if name == "LeftTrigger":
            return (self.js.get_axis(4) + 1.0) / 2.0
        if name == "RightTrigger":
            return (self.js.get_axis(5) + 1.0) / 2.0
        return 0.0

    def axis(self, name, dz=0.10, outer_dz=0.05, factor=0.2):
        raw = self._get_axis_raw(name)
        raw = apply_deadzone(raw, dz, outer_dz)
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

    slow_mode   = False
    slow_pushed = False

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

            # ── Slow mode: Y enables (50% power), B restores normal ───
            if joyROV.Y == 1 and not slow_pushed:
                slow_mode = True
            slow_pushed = (joyROV.Y == 1)

            if joyROV.B == 1:
                slow_mode = False

            scale = 0.35 if slow_mode else 0.75

            # ── ROV axes ──────────────────────────────────────────────
            ljy = joyROV.axis("LeftJoystickY",  dz=0.10, factor=1.0)  # forward/backward
            ljx = joyROV.axis("LeftJoystickX",  dz=0.10, factor=1.0)  # strafe left/right
            lt  = joyROV.axis("LeftTrigger",    dz=0.05, factor=1.0)  # translate up
            rt  = joyROV.axis("RightTrigger",   dz=0.05, factor=1.0)  # translate down
            rjy = joyROV.axis("RightJoystickY", dz=0.10, factor=1.0)  # pitch
            rjx = joyROV.axis("RightJoystickX", dz=0.10, factor=1.0)  # yaw

            # ── Claw controller ───────────────────────────────────────
            # Y/B: rotate servo (ch8) | X/A: open/close servo (ch9)
            clawRotate = int(joyClaw.Y) - int(joyClaw.B)
            clawOpen   = int(joyClaw.X) - int(joyClaw.A)

            vert = (lt - rt) * scale

            msg = (
                f"{ljy * scale} "
                f"{ljx * scale} "
                f"{vert} "
                f"{rjx * scale} "
                f"{rjy * scale * -1} "
                f"{(joyROV.RightBumper - joyROV.LeftBumper) * scale} "
                f"{clawRotate} "
                f"{clawOpen} "
                f"0.0 "
                f"0.0 "
                f"0\n"
            )

            now = time.time()
            if now - last >= period:
                sock.sendto(msg.encode(), (PI_IP, PORT))
                last = now

                print(
                    f"Fwd(LJ-Y): {ljy * scale:.2f}",
                    f"Strafe(LJ-X): {ljx * scale:.2f}",
                    f"Vert(LT-RT): {vert:.2f}",
                    f"Pitch(RJ-Y): {rjy * scale * -1:.2f}",
                    f"Yaw(RJ-X): {rjx * scale:.2f}",
                    f"Roll(RB/LB): {joyROV.RightBumper - joyROV.LeftBumper}",
                    f"SlowMode: {slow_mode}",
                    f"Servo1-Rotate(Y/B): {clawRotate}",
                    f"Servo2-Open(X/A): {clawOpen}",
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
