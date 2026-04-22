#!/usr/bin/env python3
"""
sensor_sender.py

Broadcast BNO08X + NEO-6M GPS readings over UDP as JSON.

Tested design assumptions:
- Raspberry Pi
- BNO08X over I2C
- NEO-6M GPS over UART/serial
- Laptop receives UDP packets and logs / visualizes them

Install:
    pip3 install adafruit-blinka adafruit-circuitpython-bno08x pyserial

Pi setup reminders:
- Enable I2C
- Enable serial hardware if using UART GPS
- Disable login shell on serial if GPS is on /dev/serial0 or /dev/ttyAMA0
- For BNO08X on Pi, Adafruit suggests I2C at 400 kHz in /boot/config.txt:
      dtparam=i2c_arm_baudrate=400000
"""

from __future__ import annotations

import json
import math
import socket
import time
from datetime import datetime, timezone
from typing import Any, Optional

import board
import busio
import serial

from adafruit_bno08x import (
    BNO_REPORT_ACCELEROMETER,
    BNO_REPORT_GYROSCOPE,
    BNO_REPORT_MAGNETOMETER,
    BNO_REPORT_ROTATION_VECTOR,
    BNO_REPORT_LINEAR_ACCELERATION,
    BNO_REPORT_GRAVITY,
)
from adafruit_bno08x.i2c import BNO08X_I2C


# -----------------------------
# User config
# -----------------------------
UDP_IP = "192.168.1.123"   # <-- change to your laptop IP
UDP_PORT = 5808

GPS_PORT = "/dev/serial0"  # common on Pi; verify with `python3 -m serial.tools.list_ports`
GPS_BAUD = 9600

SEND_RATE_HZ = 10.0

# BNO08X report interval in microseconds.
# 100000 = 10 Hz, 50000 = 20 Hz
REPORT_INTERVAL_US = 100000

# Some BNO08X boards show up at 0x4A, others at 0x4B.
# Leave as None to use library default first.
BNO08X_ADDRESS: Optional[int] = None


# -----------------------------
# Helpers
# -----------------------------
def utc_now_iso() -> str:
    return datetime.now(timezone.utc).isoformat()


def clean_number(value: Any) -> Optional[float]:
    """Convert to float, but reject NaN/inf and None."""
    if value is None:
        return None
    try:
        f = float(value)
        if math.isnan(f) or math.isinf(f):
            return None
        return f
    except Exception:
        return None


def clean_tuple(value: Any) -> Optional[list[Optional[float]]]:
    """Convert tuple/list of sensor values into JSON-safe floats."""
    if value is None:
        return None
    try:
        return [clean_number(v) for v in value]
    except Exception:
        return None


def parse_lat_lon(raw_value: str, direction: str) -> Optional[float]:
    """
    Convert NMEA latitude/longitude to decimal degrees.

    Latitude  format: ddmm.mmmm
    Longitude format: dddmm.mmmm
    """
    if not raw_value or not direction:
        return None

    try:
        if direction in ("N", "S"):
            deg_len = 2
        elif direction in ("E", "W"):
            deg_len = 3
        else:
            return None

        degrees = float(raw_value[:deg_len])
        minutes = float(raw_value[deg_len:])
        decimal = degrees + minutes / 60.0

        if direction in ("S", "W"):
            decimal *= -1.0

        return decimal
    except Exception:
        return None


def parse_nmea_sentence(line: str) -> dict[str, Any]:
    """
    Minimal parser for the most useful NMEA lines:
    - RMC: position, speed, track angle, date/time
    - GGA: position, fix quality, satellites, altitude
    """
    result: dict[str, Any] = {
        "raw": line.strip(),
        "sentence_type": None,
        "latitude": None,
        "longitude": None,
        "speed_knots": None,
        "track_angle_deg": None,
        "altitude_m": None,
        "fix_quality": None,
        "satellites": None,
        "utc_time": None,
        "utc_date": None,
    }

    if not line.startswith("$"):
        return result

    parts = line.strip().split(",")
    sentence = parts[0]
    result["sentence_type"] = sentence

    try:
        if sentence in ("$GPRMC", "$GNRMC"):
            # RMC:
            # [1] time, [2] status, [3] lat, [4] N/S, [5] lon, [6] E/W,
            # [7] speed knots, [8] track angle, [9] date
            if len(parts) > 9:
                result["utc_time"] = parts[1] or None
                status = parts[2] or None
                result["latitude"] = parse_lat_lon(parts[3], parts[4])
                result["longitude"] = parse_lat_lon(parts[5], parts[6])
                result["speed_knots"] = clean_number(parts[7]) if parts[7] else None
                result["track_angle_deg"] = clean_number(parts[8]) if parts[8] else None
                result["utc_date"] = parts[9] or None
                result["fix_quality"] = 1 if status == "A" else 0

        elif sentence in ("$GPGGA", "$GNGGA"):
            # GGA:
            # [1] time, [2] lat, [3] N/S, [4] lon, [5] E/W,
            # [6] fix quality, [7] satellites, [9] altitude
            if len(parts) > 9:
                result["utc_time"] = parts[1] or None
                result["latitude"] = parse_lat_lon(parts[2], parts[3])
                result["longitude"] = parse_lat_lon(parts[4], parts[5])
                result["fix_quality"] = int(parts[6]) if parts[6] else None
                result["satellites"] = int(parts[7]) if parts[7] else None
                result["altitude_m"] = clean_number(parts[9]) if parts[9] else None
    except Exception:
        # Keep raw line even if parsing fails.
        pass

    return result


def merge_gps_state(latest: dict[str, Any], parsed: dict[str, Any]) -> dict[str, Any]:
    """Update only the GPS fields that were present in this sentence."""
    latest["raw"] = parsed.get("raw")
    latest["sentence_type"] = parsed.get("sentence_type")

    for key in (
        "latitude",
        "longitude",
        "speed_knots",
        "track_angle_deg",
        "altitude_m",
        "fix_quality",
        "satellites",
        "utc_time",
        "utc_date",
    ):
        if parsed.get(key) is not None:
            latest[key] = parsed[key]

    return latest


# -----------------------------
# Init sensors
# -----------------------------
def init_bno08x() -> BNO08X_I2C:
    # Adafruit's Pi guide uses I2C wiring for the BNO08X on Raspberry Pi.
    # We use 400 kHz here to match their recommendation for Pi operation.
    i2c = busio.I2C(board.SCL, board.SDA, frequency=400000)

    if BNO08X_ADDRESS is None:
        bno = BNO08X_I2C(i2c)
    else:
        bno = BNO08X_I2C(i2c, address=BNO08X_ADDRESS)

    # Enable the reports we want to read.
    bno.enable_feature(BNO_REPORT_ROTATION_VECTOR, REPORT_INTERVAL_US)
    bno.enable_feature(BNO_REPORT_ACCELEROMETER, REPORT_INTERVAL_US)
    bno.enable_feature(BNO_REPORT_GYROSCOPE, REPORT_INTERVAL_US)
    bno.enable_feature(BNO_REPORT_MAGNETOMETER, REPORT_INTERVAL_US)
    bno.enable_feature(BNO_REPORT_LINEAR_ACCELERATION, REPORT_INTERVAL_US)
    bno.enable_feature(BNO_REPORT_GRAVITY, REPORT_INTERVAL_US)

    return bno


def init_gps() -> serial.Serial:
    return serial.Serial(
        GPS_PORT,
        baudrate=GPS_BAUD,
        timeout=0.2,
    )


# -----------------------------
# Read BNO08X
# -----------------------------
def read_bno08x_payload(bno: BNO08X_I2C) -> dict[str, Any]:
    """
    Read a set of useful BNO08X outputs.

    Notes:
    - quaternion comes from rotation_vector
    - some properties may be temporarily unavailable
    """
    payload: dict[str, Any] = {}

    try:
        payload["acceleration_m_s2"] = clean_tuple(bno.acceleration)
    except Exception as e:
        payload["acceleration_error"] = str(e)

    try:
        payload["gyro_rad_s"] = clean_tuple(bno.gyro)
    except Exception as e:
        payload["gyro_error"] = str(e)

    try:
        payload["magnetometer_uT"] = clean_tuple(bno.magnetic)
    except Exception as e:
        payload["magnetometer_error"] = str(e)

    try:
        payload["linear_acceleration_m_s2"] = clean_tuple(bno.linear_acceleration)
    except Exception as e:
        payload["linear_accel_error"] = str(e)

    try:
        payload["gravity_m_s2"] = clean_tuple(bno.gravity)
    except Exception as e:
        payload["gravity_error"] = str(e)

    try:
        # rotation_vector is quaternion-ish: (i, j, k, real)
        payload["rotation_vector_quaternion"] = clean_tuple(bno.quaternion)
    except Exception as e:
        payload["rotation_vector_error"] = str(e)

    try:
        # These may be available depending on driver version / sensor report support.
        payload["euler_deg"] = clean_tuple(bno.euler)
    except Exception:
        # Not all driver versions expose euler; that is okay.
        payload["euler_deg"] = None

    return payload


# -----------------------------
# Main
# -----------------------------
def main() -> None:
    bno = init_bno08x()
    gps_serial = init_gps()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    latest_gps: dict[str, Any] = {
        "raw": None,
        "sentence_type": None,
        "latitude": None,
        "longitude": None,
        "speed_knots": None,
        "track_angle_deg": None,
        "altitude_m": None,
        "fix_quality": None,
        "satellites": None,
        "utc_time": None,
        "utc_date": None,
    }

    packet_seq = 0
    period = 1.0 / SEND_RATE_HZ

    while True:
        loop_start = time.monotonic()
        packet_seq += 1

        # Read one GPS line per loop.
        try:
            line = gps_serial.readline().decode("ascii", errors="replace").strip()
            if line:
                parsed = parse_nmea_sentence(line)
                latest_gps = merge_gps_state(latest_gps, parsed)
        except Exception as e:
            latest_gps["read_error"] = str(e)

        packet: dict[str, Any] = {
            "timestamp": utc_now_iso(),
            "sequence": packet_seq,
            "sender": "trailcutter-sensors",
            "imu_type": "BNO08X",
            "gps_type": "NEO-6M",
            "bno08x": read_bno08x_payload(bno),
            "gps": latest_gps,
        }

        data = json.dumps(packet, separators=(",", ":")).encode("utf-8")
        sock.sendto(data, (UDP_IP, UDP_PORT))

        elapsed = time.monotonic() - loop_start
        sleep_time = period - elapsed
        if sleep_time > 0:
            time.sleep(sleep_time)


if __name__ == "__main__":
    main()