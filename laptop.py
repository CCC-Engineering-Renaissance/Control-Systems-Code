import socket, time
import pygame

PI_IP = "10.0.0.195"
PORT = 5005

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

pygame.init()
pygame.joystick.init()

if pygame.joystick.get_count() == 0:
    raise SystemExit("No joystick detected. Plug it in and retry.")

js = pygame.joystick.Joystick(0)
js.init()
print("Joystick:", js.get_name())
print(f"Sending to {PI_IP}:{PORT}")


BTN_A  = 0
BTN_B  = 1
BTN_X  = 2
BTN_Y  = 3
BTN_LB = 4
BTN_RB = 5


def btn_pair(neg_btn, pos_btn):
    #returns +1.0 if pos pressed, -1.0 if neg pressed, 0.0 if neither/both.
    pos = js.get_button(pos_btn)
    neg = js.get_button(neg_btn)
    return float(pos - neg)




SEND_HZ = 50
period = 1.0 / SEND_HZ
last = 0.0

while True:
    pygame.event.pump()

    # ── Manipulator inputs (buttons) ──
    claw_rotate = btn_pair(BTN_LB, BTN_RB)   # LB = -1, RB = +1
    claw_open   = btn_pair(BTN_A,  BTN_B)    # A  = -1, B  = +1
    claw_pitch  = btn_pair(BTN_X,  BTN_Y)    # X  = -1, Y  = +1

    # D-pad up/down for the second claw open
    hat_x, hat_y = js.get_hat(0)              # hat_y: up=+1, down=-1
    claw1_open = float(hat_y)

    # udp message
    # order must match POVState in connection
    # forward  strafe  vertical  yaw  pitch  roll
    # clawRotate  clawOpen  clawPitch  claw1Open
    # pitchAngle  yawAngle  als
    forward = 0.0
    strafe  = 0.0
    vertical = 0.0
    yaw     = 0.0
    pitch   = 0.0
    roll    = 0.0
    pitch_angle = 0.0
    yaw_angle   = 0.0
    als = 0

    now = time.time()
    if now - last >= period:
        msg = (
            f"{forward:.3f} {strafe:.3f} {vertical:.3f} "
            f"{yaw:.3f} {pitch:.3f} {roll:.3f} "
            f"{claw_rotate:.3f} {claw_open:.3f} {claw_pitch:.3f} {claw1_open:.3f} "
            f"{pitch_angle:.3f} {yaw_angle:.3f} {als}\n"
        )
        sock.sendto(msg.encode(), (PI_IP, PORT))
        last = now

    time.sleep(0.001)




# import socket, time
# import pygame
#
# PI_IP = "10.0.0.195"
# PORT = 5005
#
# sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
#
# pygame.init()
# pygame.joystick.init()
#
# if pygame.joystick.get_count() == 0:
#     raise SystemExit("No joystick detected. Plug it in and retry.")
#
# js = pygame.joystick.Joystick(0)
# js.init()
# print("Joystick:", js.get_name())
# print(f"Sending to {PI_IP}:{PORT}")
#
# SEND_HZ = 50
# period = 1.0 / SEND_HZ
# last = 0.0
# right_y = 0.0
#
# while True:
#     pygame.event.pump()
#
#     # Many controllers map right stick Y to axis 3 or 4 depending on driver.
#     # Try axis 3 first; if it doesn't move, change to 4.
#     axis_index = 3
#     right_y = -js.get_axis(axis_index)  # invert so up=+1
#
#     now = time.time()
#     if now - last >= period:
#     msg = f"{right_y:.3f} 0 0 0 0 0 0 0 0 0 0 0 0\n"
#         sock.sendto(msg, (PI_IP, PORT))
#         last = now
#
#     time.sleep(0.001)




