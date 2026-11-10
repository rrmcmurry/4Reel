import json
import socket
import time
import pygame

MOWER_IP = "10.68.68.2"   # change this
PORT = 5005
SEND_RATE_HZ = 20

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

pygame.init()
pygame.joystick.init()

if pygame.joystick.get_count() == 0:
    raise RuntimeError("No joystick detected")

joystick = pygame.joystick.Joystick(0)
joystick.init()

print(f"Using joystick: {joystick.get_name()}")

def clamp(x, lo=-1.0, hi=1.0):
    return max(lo, min(hi, x))

def deadband(x, band=0.08):
    return 0.0 if abs(x) < band else x

def curve(x):
    return x ** 3

period = 1.0 / SEND_RATE_HZ

try:
    while True:
        pygame.event.pump()

        throttle = -joystick.get_axis(1)   # left stick Y
        steer = joystick.get_axis(0)       # left stick X

        throttle = curve(deadband(clamp(throttle))) * 0.6
        steer = curve(deadband(clamp(steer))) * 0.6

        msg = {
            "throttle": throttle,
            "steer": steer,
        }

        payload = json.dumps(msg).encode("utf-8")
        sock.sendto(payload, (MOWER_IP, PORT))

        print(f"TX throttle={throttle:.3f} steer={steer:.3f}", end="\r")
        time.sleep(period)

except KeyboardInterrupt:
    pass
finally:
    pygame.quit()