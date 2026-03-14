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

SEND_HZ = 50
period = 1.0 / SEND_HZ
last = 0.0
right_y = 0.0

while True:
    pygame.event.pump()

    # Many controllers map right stick Y to axis 3 or 4 depending on driver.
    # Try axis 3 first; if it doesn't move, change to 4.
    axis_index = 3
    right_y = -js.get_axis(axis_index)  # invert so up=+1

    now = time.time()
    if now - last >= period:
    msg = f"{right_y:.3f} 0 0 0 0 0 0 0 0 0 0 0 0\n"
        sock.sendto(msg, (PI_IP, PORT))
        last = now

    time.sleep(0.001)
