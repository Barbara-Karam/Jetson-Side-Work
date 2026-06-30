#!/usr/bin/env python3
"""
RPOD Lifter — Host-side communicator
  - Receives and prints ActuatorFB telemetry from STM32
  - Sends ControlCmd motor commands
  - Sends heartbeat and e-stop

Usage:
    python3 lifter_comm.py [port] [baud] [--quiet]

Defaults: /dev/ttyUSB0  115200
--quiet : suppress telemetry flood so you can type commands cleanly
"""

import serial
import struct
import sys
import threading
import time
from datetime import datetime

# ── Port config ───────────────────────────────────────────────────────────────
QUIET    = "--quiet" in sys.argv or "-q" in sys.argv
args     = [a for a in sys.argv[1:] if not a.startswith("-")]
PORT     = args[0] if len(args) > 0 else "/dev/ttyUSB0"
BAUDRATE = int(args[1]) if len(args) > 1 else 115200  # FIX: was 112500

# ── Protocol constants (must match comm.h) ────────────────────────────────────
SYNC1, SYNC2, SYNC3 = 0xAA, 0x55, 0xFF
SYNC = bytes([SYNC1, SYNC2, SYNC3])

MSG_CONTROL_CMD  = 0x01
MSG_ACTUATOR_FB  = 0x02
MSG_HEARTBEAT    = 0x10  # FIX: was 0x03
MSG_ESTOP        = 0xFF  # FIX: was 0x04

# ── SYS_STATUS bitmask (matches comm.h) ──────────────────────────────────────
SYS_STATUS_OK        = 0x00
SYS_STATUS_IMU_FAULT = 0x01
SYS_STATUS_ESTOP     = 0x02
SYS_STATUS_LIMIT_HIT = 0x04

# ── ActuatorFB_t layout — must exactly match comm.h struct (packed) ───────────
#   int16_t  pwm_out[2]        — 2h
#   uint8_t  limit_switches    — B
#   int16_t  accel_x_mg        — h
#   int16_t  accel_y_mg        — h
#   int16_t  accel_z_mg        — h
#   int16_t  gyro_x_cdps       — h
#   int16_t  gyro_y_cdps       — h
#   int16_t  gyro_z_cdps       — h
#   uint8_t  imu_valid         — B   FIX: was before accel fields
#   uint32_t timestamp_ms      — I
#   uint8_t  sys_status        — B   FIX: was before timestamp
# Total: 2+2 + 1 + 2+2+2+2+2+2 + 1 + 4 + 1 = 23 bytes
FB_FMT  = "<2hBhhhhhhBIB"
FB_SIZE = struct.calcsize(FB_FMT)

# ── CRC-8 (poly 0x07, matches crc8() in comm.c) ──────────────────────────────
def crc8(data: bytes) -> int:
    crc = 0
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = ((crc << 1) ^ 0x07) if (crc & 0x80) else (crc << 1)
            crc &= 0xFF
    return crc

# ── Frame builder ─────────────────────────────────────────────────────────────
def build_frame(msg_type: int, payload: bytes = b"") -> bytes:
    header = bytes([msg_type, len(payload)])
    crc    = crc8(header + payload)
    return SYNC + header + payload + bytes([crc])

# ── Frame reader (blocking, runs in RX thread) ────────────────────────────────
def read_frame(ser: serial.Serial):
    """Block until a valid frame arrives. Returns (msg_type, payload)."""
    buf = bytearray()
    while True:
        b = ser.read(1)
        if not b:
            continue
        buf += b
        idx = buf.find(SYNC)
        if idx == -1:
            buf = buf[-2:]
            continue
        buf = buf[idx:]
        while len(buf) < 5:
            chunk = ser.read(5 - len(buf))
            if chunk:
                buf += chunk
        msg_type    = buf[3]
        payload_len = buf[4]
        total       = 6 + payload_len
        while len(buf) < total:
            chunk = ser.read(total - len(buf))
            if chunk:
                buf += chunk
        payload  = bytes(buf[5:5 + payload_len])
        crc_recv = buf[5 + payload_len]
        crc_calc = crc8(bytes([msg_type, payload_len]) + payload)
        buf = buf[total:]
        if crc_recv != crc_calc:
            if not QUIET:
                print(f"  [CRC FAIL] expected {crc_calc:#04x} got {crc_recv:#04x}")
            continue
        return msg_type, payload

# ── Telemetry printer ─────────────────────────────────────────────────────────
def print_fb(payload: bytes):
    if QUIET:
        return
    if len(payload) != FB_SIZE:
        print(f"  [FB SIZE MISMATCH] got {len(payload)}, expected {FB_SIZE}")
        return
    (pwm0, pwm1,
     limit_sw,
     ax_mg, ay_mg, az_mg,
     gx_cdps, gy_cdps, gz_cdps,
     imu_valid,
     ts_ms,
     sys_status) = struct.unpack(FB_FMT, payload)

    flags = []
    if sys_status & SYS_STATUS_IMU_FAULT: flags.append("IMU_FAULT")
    if sys_status & SYS_STATUS_ESTOP:     flags.append("ESTOP")
    if sys_status & SYS_STATUS_LIMIT_HIT: flags.append("LIMIT_HIT")
    flag_str = " ".join(flags) if flags else "OK"

    ts = datetime.now().strftime("%H:%M:%S.%f")[:12]
    print(
        f"{ts} | "
        f"IMU={'OK' if imu_valid else 'FAULT':5s} | "
        f"AX={ax_mg/1000:+.3f}g AY={ay_mg/1000:+.3f}g AZ={az_mg/1000:+.3f}g | "
        f"GX={gx_cdps/100:+.1f} GY={gy_cdps/100:+.1f} GZ={gz_cdps/100:+.1f} dps | "
        f"PWM={pwm0:+5d},{pwm1:+5d} | "
        f"SW={limit_sw:#04x} | "
        f"[{flag_str}] | "
        f"t={ts_ms}ms"
    )

# ── TX helpers ────────────────────────────────────────────────────────────────
def send_motor(ser: serial.Serial, pwm0: int, pwm1: int):
    """Send ControlCmd. pwm range: -4199 to +4199."""
    pwm0 = max(-4199, min(4199, pwm0))
    pwm1 = max(-4199, min(4199, pwm1))
    payload = struct.pack("<hh", pwm0, pwm1)
    ser.write(build_frame(MSG_CONTROL_CMD, payload))
    print(f"  >> CMD  pwm0={pwm0:+5d}  pwm1={pwm1:+5d}")

def send_heartbeat(ser: serial.Serial):
    ser.write(build_frame(MSG_HEARTBEAT))
    print("  >> HEARTBEAT sent")

def send_estop(ser: serial.Serial):
    ser.write(build_frame(MSG_ESTOP))
    print("  >> E-STOP sent")

# ── RX thread ─────────────────────────────────────────────────────────────────
def rx_thread(ser: serial.Serial, stop_event: threading.Event):
    while not stop_event.is_set():
        try:
            msg_type, payload = read_frame(ser)
            if msg_type == MSG_ACTUATOR_FB:
                print_fb(payload)
            elif msg_type == MSG_HEARTBEAT:
                print("  << HEARTBEAT ack")
            else:
                if not QUIET:
                    print(f"  << Unknown msg_type={msg_type:#04x} len={len(payload)}")
        except serial.SerialException:
            break
        except Exception as e:
            print(f"  [RX ERROR] {e}")

# ── CLI ───────────────────────────────────────────────────────────────────────
HELP = """
Commands:
  m <pwm0> <pwm1>   Send motor command  e.g.  m 2000 2000
  s                 Stop both motors         (m 0 0)
  hb                Send heartbeat
  e                 Send E-STOP  (requires board reset to clear)
  q                 Quit
"""

def main():
    print(f"Connecting to {PORT} @ {BAUDRATE} baud...  {'[QUIET MODE]' if QUIET else '[TELEMETRY ON]'}")
    print(f"ActuatorFB payload size: {FB_SIZE} bytes")
    try:
        ser = serial.Serial(PORT, BAUDRATE, timeout=2)
    except serial.SerialException as e:
        print(f"ERROR: {e}")
        sys.exit(1)

    time.sleep(0.1)
    print("Connected.")
    print(HELP)

    stop_event = threading.Event()
    t = threading.Thread(target=rx_thread, args=(ser, stop_event), daemon=True)
    t.start()

    try:
        while True:
            try:
                raw = input("> ").strip()
            except EOFError:
                break
            if not raw:
                continue
            parts = raw.split()
            cmd   = parts[0].lower()

            if cmd == "q":
                break
            elif cmd == "m":
                if len(parts) != 3:
                    print("Usage: m <pwm0> <pwm1>")
                    continue
                try:
                    send_motor(ser, int(parts[1]), int(parts[2]))
                except ValueError:
                    print("PWM values must be integers")
            elif cmd == "s":
                send_motor(ser, 0, 0)
            elif cmd == "hb":
                send_heartbeat(ser)
            elif cmd == "e":
                send_estop(ser)
            else:
                print(HELP)

    except KeyboardInterrupt:
        pass
    finally:
        stop_event.set()
        send_motor(ser, 0, 0)
        ser.close()
        print("\nDisconnected.")

if __name__ == "__main__":
    main()
