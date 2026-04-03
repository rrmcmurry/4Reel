import json
import socket
import threading
import time
import pigpio

UDP_IP = "0.0.0.0"
UDP_PORT = 5005

LEFT_PIN = 12
RIGHT_PIN = 13

CONTROL_HZ = 50.0
COMMAND_TIMEOUT = 0.25
TURN_GAIN = 0.35
DEADBAND = 0.08
RAMP_STEP = 0.03   # per control loop tick

# Shared command state
latest_throttle = 0.0
latest_steer = 0.0
latest_command_time = 0.0
state_lock = threading.Lock()


def clamp(x, lo=-1.0, hi=1.0):
    return max(lo, min(hi, x))


def deadband(x, band=DEADBAND):
    return 0.0 if abs(x) < band else x


def apply_curve(x):
    # Soft near center, stronger at extremes
    return x ** 3


def normalize_pair(left, right):
    m = max(1.0, abs(left), abs(right))
    return left / m, right / m


def approach(current, target, step):
    if target > current + step:
        return current + step
    if target < current - step:
        return current - step
    return target


def set_motor(pi, pin, value, invert=False):
    value = clamp(value)
    if invert:
        value = -value

    pulse = int(1500 + value * 500)
    pi.set_servo_pulsewidth(pin, pulse)


def stop_motor(pi, pin):
    pi.set_servo_pulsewidth(pin, 1500)
    time.sleep(0.1)
    pi.set_servo_pulsewidth(pin, 0)


def udp_receiver():
    global latest_throttle, latest_steer, latest_command_time

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((UDP_IP, UDP_PORT))
    sock.settimeout(0.1)

    print(f"Listening for UDP on {UDP_IP}:{UDP_PORT}")

    while True:
        try:
            data, addr = sock.recvfrom(4096)
            msg = json.loads(data.decode("utf-8"))

            throttle = float(msg.get("throttle", 0.0))
            steer = float(msg.get("steer", 0.0))

            throttle = clamp(throttle)
            steer = clamp(steer)

            with state_lock:
                latest_throttle = throttle
                latest_steer = steer
                latest_command_time = time.time()

        except socket.timeout:
            pass
        except Exception as e:
            print(f"UDP receive error: {e}")


def main():
    global latest_throttle, latest_steer, latest_command_time

    pi = pigpio.pi()
    if not pi.connected:
        raise RuntimeError("Could not connect to pigpiod")

    # Neutral on startup
    pi.set_servo_pulsewidth(LEFT_PIN, 1500)
    pi.set_servo_pulsewidth(RIGHT_PIN, 1500)
    time.sleep(2.0)

    recv_thread = threading.Thread(target=udp_receiver, daemon=True)
    recv_thread.start()

    left_cmd = 0.0
    right_cmd = 0.0

    period = 1.0 / CONTROL_HZ

    try:
        while True:
            loop_start = time.time()

            with state_lock:
                throttle = latest_throttle
                steer = latest_steer
                age = loop_start - latest_command_time

            if age > COMMAND_TIMEOUT:
                throttle = 0.0
                steer = 0.0

            throttle = apply_curve(deadband(throttle))
            steer = apply_curve(deadband(steer))

            # Reduce steering authority somewhat as speed increases
            steer *= (1.0 - 0.5 * abs(throttle))

            left_target = throttle + steer * TURN_GAIN
            right_target = throttle - steer * TURN_GAIN

            left_target, right_target = normalize_pair(left_target, right_target)

            left_cmd = approach(left_cmd, left_target, RAMP_STEP)
            right_cmd = approach(right_cmd, right_target, RAMP_STEP)

            set_motor(pi, LEFT_PIN, left_cmd, invert=False)
            set_motor(pi, RIGHT_PIN, right_cmd, invert=True)

            elapsed = time.time() - loop_start
            sleep_time = period - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    except KeyboardInterrupt:
        pass
    finally:
        stop_motor(pi, LEFT_PIN)
        stop_motor(pi, RIGHT_PIN)
        pi.stop()


if __name__ == "__main__":
    main()
