#!/usr/bin/env python3
"""
imu_receiver.py  —  Run on the Jetson
Receives data from both STM32 controllers simultaneously.

    Base controller (mobility): $BASE,ax,ay,az,gx,gy,gz,rpm0,rpm1,rpm2,rpm3
    Arm  controller (docking):  $ARM, ax,ay,az,gx,gy,gz,sw1,sw2,sw3,sw4

Install:
    pip3 install pyserial

Usage:
    python3 imu_receiver.py

Find your ports:
    ls /dev/ttyUSB*   →  USB-TTL adapters
    ls /dev/ttyTHS*   →  Jetson hardware UART
"""

import serial
import threading
import time
import sys
from datetime import datetime

# ── CONFIG ───────────────────────────────────────────────────────────────────
BASE_PORT  = "/dev/ttyUSB0"   # Black Pill #2 — mobility controller
ARM_PORT   = "/dev/ttyUSB1"   # F401RCT6     — arm controller
BAUDRATE   = 921600
# ─────────────────────────────────────────────────────────────────────────────

base_data = {
    "accel_x": 0.0, "accel_y": 0.0, "accel_z": 0.0,
    "gyro_x":  0.0, "gyro_y":  0.0, "gyro_z":  0.0,
    "rpm":     [0.0, 0.0, 0.0, 0.0],
    "updated": False, "ts": ""
}

arm_data = {
    "accel_x": 0.0, "accel_y": 0.0, "accel_z": 0.0,
    "gyro_x":  0.0, "gyro_y":  0.0, "gyro_z":  0.0,
    "sw":      [0, 0, 0, 0],
    "docking": False,
    "updated": False, "ts": ""
}

lock = threading.Lock()


def parse_base(line: str) -> bool:
    line = line.strip()
    if not line.startswith("$BASE"):
        return False
    try:
        p = line.split(",")
        if len(p) != 12:
            return False
        with lock:
            base_data["accel_x"] = float(p[1])
            base_data["accel_y"] = float(p[2])
            base_data["accel_z"] = float(p[3])
            base_data["gyro_x"]  = float(p[4])
            base_data["gyro_y"]  = float(p[5])
            base_data["gyro_z"]  = float(p[6])
            base_data["rpm"]     = [float(p[7]), float(p[8]),
                                    float(p[9]), float(p[10])]
            base_data["updated"] = True
            base_data["ts"]      = datetime.now().strftime("%H:%M:%S.%f")[:12]
        return True
    except (ValueError, IndexError):
        return False


def parse_arm(line: str) -> bool:
    line = line.strip()
    if not line.startswith("$ARM"):
        return False
    try:
        p = line.split(",")
        if len(p) != 12:
            return False
        sw = [int(p[8]), int(p[9]), int(p[10]), int(p[11])]
        with lock:
            arm_data["accel_x"] = float(p[1])
            arm_data["accel_y"] = float(p[2])
            arm_data["accel_z"] = float(p[3])
            arm_data["gyro_x"]  = float(p[4])
            arm_data["gyro_y"]  = float(p[5])
            arm_data["gyro_z"]  = float(p[6])
            arm_data["sw"]      = sw
            arm_data["docking"] = any(sw)
            arm_data["updated"] = True
            arm_data["ts"]      = datetime.now().strftime("%H:%M:%S.%f")[:12]
        return True
    except (ValueError, IndexError):
        return False


def read_port(port, baud, parser, label):
    while True:
        try:
            print(f"[{label}] Connecting to {port} @ {baud}...")
            with serial.Serial(port, baud, timeout=2) as ser:
                print(f"[{label}] Connected.")
                while True:
                    raw  = ser.readline()
                    line = raw.decode("utf-8", errors="replace").strip()
                    if not line:
                        continue
                    if line.startswith("#"):
                        print(f"  [{label}] {line}")
                        continue
                    parser(line)
        except serial.SerialException as e:
            print(f"  [{label}] Port error: {e} — retrying in 3s")
            time.sleep(3)


def print_dashboard():
    while True:
        time.sleep(0.2)
        with lock:
            b = dict(base_data)
            a = dict(arm_data)

        print("\033[2J\033[H", end="")
        print("=" * 70)
        print("  RPOD SYSTEM MONITOR")
        print("=" * 70)

        print(f"\n  [BASE CONTROLLER]  {b['ts']}")
        print(f"  Accel (m/s2):  x={b['accel_x']:+7.3f}  y={b['accel_y']:+7.3f}  z={b['accel_z']:+7.3f}")
        print(f"  Gyro  (deg/s): x={b['gyro_x']:+7.3f}  y={b['gyro_y']:+7.3f}  z={b['gyro_z']:+7.3f}")
        r = b["rpm"]
        print(f"  Wheel RPM:  FL={r[0]:+6.1f}  FR={r[1]:+6.1f}  RL={r[2]:+6.1f}  RR={r[3]:+6.1f}")

        print(f"\n  [ARM CONTROLLER]   {a['ts']}")
        print(f"  Accel (m/s2):  x={a['accel_x']:+7.3f}  y={a['accel_y']:+7.3f}  z={a['accel_z']:+7.3f}")
        print(f"  Gyro  (deg/s): x={a['gyro_x']:+7.3f}  y={a['gyro_y']:+7.3f}  z={a['gyro_z']:+7.3f}")
        sw = a["sw"]
        print(f"  Switches: SW1={'X' if sw[0] else 'o'}  SW2={'X' if sw[1] else 'o'}  "
              f"SW3={'X' if sw[2] else 'o'}  SW4={'X' if sw[3] else 'o'}  "
              f"{'  *** DOCKING DETECTED ***' if a['docking'] else ''}")

        print("\n" + "=" * 70)
        print("  Ctrl+C to exit")


def main():
    threading.Thread(
        target=read_port,
        args=(BASE_PORT, BAUDRATE, parse_base, "BASE"),
        daemon=True
    ).start()

    threading.Thread(
        target=read_port,
        args=(ARM_PORT, BAUDRATE, parse_arm, "ARM"),
        daemon=True
    ).start()

    try:
        print_dashboard()
    except KeyboardInterrupt:
        print("\nStopped.")
        sys.exit(0)


if __name__ == "__main__":
    main()
