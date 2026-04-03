# 4Reel – Remote-Controlled Reel Mower Platform

## Overview

4Reel is a Raspberry Pi–based control system for a reel mower.
It receives real-time drive commands over the network and converts them into PWM signals for motor controllers.

The system is intentionally simple and modular:

* **Control input** (laptop, browser, future vision system)
* → **UDP network commands**
* → **Raspberry Pi control loop**
* → **PWM output via pigpio**
* → **Motor controllers (REV SPARK)**

This architecture allows multiple control sources (manual, automated, vision-based) without changing the motor control layer.

---

## Current Components

* `4reel.py`
  Main control service running on the Raspberry Pi.
  Receives UDP commands and drives motors.

* `udp_joystick_sender.py`
  Laptop-side script that reads an Xbox controller and sends throttle/steering commands over UDP.

* `4reel.service`
  systemd service definition for running `4reel.py` at startup.

---

## Requirements

### Hardware

* Raspberry Pi (tested on Pi 4)
* REV SPARK motor controller(s)
* Motors (e.g., CIM)
* Network connection (WiFi or Ethernet)

### Software

* Python 3
* pigpio

---

## Installing pigpio

pigpio is required for stable PWM signal generation.

### Install from source

```bash
sudo apt update
sudo apt install git build-essential python3

git clone https://github.com/joan2937/pigpio.git
cd pigpio
make
sudo make install
```

### Create pigpiod service

```bash
sudo nano /etc/systemd/system/pigpiod.service
```

Paste:

```ini
[Unit]
Description=Pigpio daemon
After=network.target

[Service]
Type=forking
ExecStart=/usr/local/bin/pigpiod
Restart=on-failure

[Install]
WantedBy=multi-user.target
```

Enable and start:

```bash
sudo systemctl daemon-reload
sudo systemctl enable pigpiod
sudo systemctl start pigpiod
```

Verify:

```bash
pigs t
```

---

## Installing 4Reel Service

Copy `4reel.service` to systemd:

```bash
sudo cp 4reel.service /etc/systemd/system/
```

Reload systemd:

```bash
sudo systemctl daemon-reload
```

Enable service at boot:

```bash
sudo systemctl enable 4reel
```

Start service:

```bash
sudo systemctl start 4reel
```

---

## Service Management

Restart after code changes:

```bash
sudo systemctl restart 4reel
```

View logs:

```bash
journalctl -u 4reel -f
```

Check status:

```bash
systemctl status 4reel
```

---

## Running the Joystick Sender

On a laptop with a connected controller:

```bash
python udp_joystick_sender.py
```

Make sure to update the mower IP address in the script.

---

## Control Model

Commands are sent as JSON over UDP:

```json
{
  "throttle": 0.3,
  "steer": -0.1
}
```

* `throttle` → forward/back (-1.0 to 1.0)
* `steer` → left/right correction (-1.0 to 1.0)

The Raspberry Pi:

* applies deadband and smoothing
* mixes throttle/steer into left/right motor outputs
* ramps changes to avoid current spikes
* defaults to neutral if commands stop

---

## Architecture Notes

* UDP is used for low-latency control
* No connection is required; commands are streamed continuously
* Loss of packets is tolerated (new commands arrive quickly)
* Safety model: **no command → neutral**

---

## Future Directions

* Web-based control UI (Gamepad API + WebSocket)
* Vision-based control (secondary Pi sending commands)
* Telemetry feedback (battery, current, position)
* Autonomous mowing patterns

---

## Disclaimer

This project controls physical hardware.
Test carefully with wheels off the ground before full operation.

---

## License

TBD
