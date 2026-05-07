import pygame, time
pygame.init()
pygame.joystick.init()
pygame.display.set_mode((200, 100))
js = pygame.joystick.Joystick(0)
js.init()
print(f"Controller: {js.get_name()}")
print("Move one axis at a time and watch which number changes:\n")
while True:
    pygame.event.pump()
    axes = [round(js.get_axis(i), 2) for i in range(js.get_numaxes())]
    print(f"\rAxes: {axes}    ", end="")
    time.sleep(0.05)