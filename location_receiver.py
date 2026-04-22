#!/usr/bin/env python3
"""
sensor_receiver.py

Receives UDP JSON packets from Trailcutter's Pi sensor sender,
prints a compact status line, and logs packets to disk.

Usage:
    python3 sensor_receiver.py
    python3 sensor_receiver.py --port 5808 --jsonl sensor_log.jsonl --csv sensor_log.csv

"""

from __future__ import annotations

import argparse
import csv
import json
import math
import socket
import sys
import time
from datetime import datetime
from pathlib import Path
from typing import Any


def safe_get(d: dict[str, Any], *keys: str, default=None):
    cur: Any = d
    for key in keys:
        if not isinstance(cur, dict) or key not in cur:
            return default
        cur = cur[key]
    return cur


def fmt_num(value: Any, digits: int = 2, none_text: str = "-") -> str:
    try:
        if value is None:
            return none_text
        f = float(value)
        if math.isnan(f) or math.isinf(f):
            return none_text
        return f"{f:.{digits}f}"
    except Exception:
        return none_text


def quat_to_yaw_deg(quat: list[Any] | None) -> float | None:
    """
    Convert quaternion [x, y, z, w] to yaw in degrees.
    Returns None if unavailable or invalid.
    """
    if not quat or len(quat) != 4:
        return None

    try:
        x, y, z, w = [float(v) for v in quat]
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        yaw_rad = math.atan2(siny_cosp, cosy_cosp)
        yaw_deg = math.degrees(yaw_rad)
        if yaw_deg < 0:
            yaw_deg += 360.0
        return yaw_deg
    except Exception:
        return None


def open_csv_writer(csv_path: Path):
    csv_file = csv_path.open("a", newline="", encoding="utf-8")
    writer = csv.writer(csv_file)

    if csv_file.tell() == 0:
        writer.writerow([
            "received_at_local",
            "packet_timestamp",
            "sequence",
            "gps_latitude",
            "gps_longitude",
            "gps_altitude_m",
            "gps_speed_knots",
            "gps_track_angle_deg",
            "gps_fix_quality",
            "gps_satellites",
            "imu_yaw_deg",
            "imu_roll_deg",
            "imu_pitch_deg",
            "imu_accel_x",
            "imu_accel_y",
            "imu_accel_z",
            "imu_gyro_x",
            "imu_gyro_y",
            "imu_gyro_z",
        ])

    return csv_file, writer


def append_csv_row(writer, packet: dict[str, Any]):
    euler = safe_get(packet, "bno08x", "euler_deg", default=None)
    quat = safe_get(packet, "bno08x", "rotation_vector_quaternion", default=None)
    accel = safe_get(packet, "bno08x", "acceleration_m_s2", default=None)
    gyro = safe_get(packet, "bno08x", "gyro_rad_s", default=None)

    yaw = None
    roll = None
    pitch = None

    if isinstance(euler, list) and len(euler) == 3 and all(v is not None for v in euler):
        yaw, roll, pitch = euler
    else:
        yaw = quat_to_yaw_deg(quat)

    writer.writerow([
        datetime.now().isoformat(timespec="seconds"),
        packet.get("timestamp"),
        packet.get("sequence"),
        safe_get(packet, "gps", "latitude"),
        safe_get(packet, "gps", "longitude"),
        safe_get(packet, "gps", "altitude_m"),
        safe_get(packet, "gps", "speed_knots"),
        safe_get(packet, "gps", "track_angle_deg"),
        safe_get(packet, "gps", "fix_quality"),
        safe_get(packet, "gps", "satellites"),
        yaw,
        roll,
        pitch,
        accel[0] if isinstance(accel, list) and len(accel) > 0 else None,
        accel[1] if isinstance(accel, list) and len(accel) > 1 else None,
        accel[2] if isinstance(accel, list) and len(accel) > 2 else None,
        gyro[0] if isinstance(gyro, list) and len(gyro) > 0 else None,
        gyro[1] if isinstance(gyro, list) and len(gyro) > 1 else None,
        gyro[2] if isinstance(gyro, list) and len(gyro) > 2 else None,
    ])


def compact_status_line(packet: dict[str, Any], addr: tuple[str, int]) -> str:
    seq = packet.get("sequence", "-")

    lat = safe_get(packet, "gps", "latitude")
    lon = safe_get(packet, "gps", "longitude")
    sats = safe_get(packet, "gps", "satellites")
    fixq = safe_get(packet, "gps", "fix_quality")
    speed_knots = safe_get(packet, "gps", "speed_knots")
    gps_track = safe_get(packet, "gps", "track_angle_deg")

    euler = safe_get(packet, "bno08x", "euler_deg", default=None)
    quat = safe_get(packet, "bno08x", "rotation_vector_quaternion", default=None)

    yaw = None
    roll = None
    pitch = None

    if isinstance(euler, list) and len(euler) == 3:
        yaw, roll, pitch = euler
    else:
        yaw = quat_to_yaw_deg(quat)

    return (
        f"[{datetime.now().strftime('%H:%M:%S')}] "
        f"{addr[0]} seq={seq} "
        f"lat={fmt_num(lat, 6)} lon={fmt_num(lon, 6)} "
        f"fix={fixq if fixq is not None else '-'} sats={sats if sats is not None else '-'} "
        f"spd_kn={fmt_num(speed_knots, 2)} track={fmt_num(gps_track, 1)} "
        f"yaw={fmt_num(yaw, 1)} roll={fmt_num(roll, 1)} pitch={fmt_num(pitch, 1)}"
    )


def main():
    parser = argparse.ArgumentParser(description="Receive and log Trailcutter sensor UDP packets.")
    parser.add_argument("--bind", default="0.0.0.0", help="IP to bind to (default: 0.0.0.0)")
    parser.add_argument("--port", type=int, default=5808, help="UDP port to listen on")
    parser.add_argument("--jsonl", default="sensor_log.jsonl", help="Path to JSONL log file")
    parser.add_argument("--csv", default="sensor_log.csv", help="Path to CSV log file")
    parser.add_argument("--quiet", action="store_true", help="Do not print live status lines")
    args = parser.parse_args()

    jsonl_path = Path(args.jsonl)
    csv_path = Path(args.csv)

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((args.bind, args.port))

    jsonl_file = jsonl_path.open("a", encoding="utf-8")
    csv_file, csv_writer = open_csv_writer(csv_path)

    print(f"Listening on UDP {args.bind}:{args.port}")
    print(f"Logging JSONL to {jsonl_path.resolve()}")
    print(f"Logging CSV   to {csv_path.resolve()}")

    packet_count = 0
    start_time = time.time()

    try:
        while True:
            data, addr = sock.recvfrom(65535)
            received_at = datetime.now().isoformat(timespec="milliseconds")

            try:
                packet = json.loads(data.decode("utf-8"))
            except Exception as exc:
                print(f"Failed to decode packet from {addr}: {exc}", file=sys.stderr)
                continue

            packet_count += 1

            log_record = {
                "received_at_local": received_at,
                "source_ip": addr[0],
                "source_port": addr[1],
                "packet": packet,
            }

            jsonl_file.write(json.dumps(log_record) + "\n")
            jsonl_file.flush()

            append_csv_row(csv_writer, packet)
            csv_file.flush()

            if not args.quiet:
                print(compact_status_line(packet, addr))

    except KeyboardInterrupt:
        elapsed = time.time() - start_time
        rate = packet_count / elapsed if elapsed > 0 else 0.0
        print("\nStopped.")
        print(f"Packets received: {packet_count}")
        print(f"Elapsed seconds:  {elapsed:.1f}")
        print(f"Average rate:     {rate:.2f} packets/sec")

    finally:
        jsonl_file.close()
        csv_file.close()
        sock.close()


if __name__ == "__main__":
    main()