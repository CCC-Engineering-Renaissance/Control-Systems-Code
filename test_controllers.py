import os
os.environ["SDL_JOYSTICK_HIDAPI_XBOX"] = "0"

import pygame
pygame.init()
pygame.joystick.init()

print(f"Total controllers found: {pygame.joystick.get_count()}")
for i in range(pygame.joystick.get_count()):
    js = pygame.joystick.Joystick(i)
    js.init()
    print(f"  [{i}] {js.get_name()} | GUID: {js.get_guid()}")
