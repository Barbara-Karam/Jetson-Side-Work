#!/usr/bin/env python3
# NOTICE THAT MAX SPEED IS 35

import serial
import struct
import sys
import threading
import time
from datetime import datetime

QUIET    = "--quiet" in sys.argv or "-q" in sys.argv
args     = [a for a in sys.argv[1:] if not a.startswith("-")]
PORT     = args[0] if len(args) > 0 else "/dev/ttyTHS1"
BAUDRATE = int(args[1]) if len(args) > 1 else 921600

SYNC1, SYNC2, SYNC3 = 0xAA, 0x55, 0xA5
SYNC = bytes([SYNC1, SYNC2, SYNC3])

MSG_CONTROL_CMD = 0x01
MSG_ACTUATOR_FB = 0x02
MSG_HEARTBEAT   = 0x03
MSG_ESTOP       = 0x04

FB_FMT  = "<4h4i4hIB"
FB_SIZE = struct.calcsize(FB_FMT)  # 37 bytes

# --- RATE LIMITER SETTING ---
PRINT_INTERVAL = 0.2  # 0.2 seconds = 5 prints per second
last_print_time = 0.0


def crc8(data: bytes) -> int:
    crc = 0
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = ((crc << 1) ^ 0x07) if (crc & 0x80) else (crc << 1)
            crc &= 0xFF
    return crc


def build_frame(msg_type: int, payload: bytes = b"") -> bytes:
    header = bytes([msg_type, len(payload)])
    return SYNC + header + payload + bytes([crc8(header + payload)])


def read_frame(ser):
    buf = bytearray()
    while True:
        b = ser.read(1)
        if not b:
            continue
        buf += b

        while len(buf) < 3:
            chunk = ser.read(3 - len(buf))
            if chunk:
                buf += chunk

        idx = -1
        for i in range(len(buf) - 2):
            if buf[i] == SYNC1 and buf[i+1] == SYNC2 and buf[i+2] == SYNC3:
                idx = i
                break
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
                print(f"  [CRC FAIL] got {crc_recv:#04x} expected {crc_calc:#04x}")
            continue

        return msg_type, payload


def print_fb(payload):
    global last_print_time
    if QUIET:
        return
    
    # --- RATE LIMITING LOGIC ---
    now = time.monotonic()
    if now - last_print_time < PRINT_INTERVAL:
        return
    last_print_time = now

    if len(payload) != FB_SIZE:
        print(f"  [FB SIZE] got {len(payload)} expected {FB_SIZE}")
        return
    
    u    = struct.unpack(FB_FMT, payload)
    rpms = [x / 10.0 for x in u[0:4]]
    encs = u[4:8]
    pwms = u[8:12]
    ts   = datetime.now().strftime("%H:%M:%S.%f")[:11]
    stat = "OK" if u[13] == 0 else f"FAULT:{u[13]:#04x}"
    print(
        f"{ts} | "
        f"RPM:[{rpms[0]:+6.1f},{rpms[1]:+6.1f},{rpms[2]:+6.1f},{rpms[3]:+6.1f}] "
        f"PWM:[{pwms[0]:+5d},{pwms[1]:+5d},{pwms[2]:+5d},{pwms[3]:+5d}] "
        f"ENC:[{encs[0]:+7d},{encs[1]:+7d},{encs[2]:+7d},{encs[3]:+7d}] "
        f"[{stat}]"
    )


def send_motor_targets(ser, r0, r1, r2, r3, mode=0):
    payload = struct.pack("<4hB",
                         int(r0 * 10), int(r1 * 10),
                         int(r2 * 10), int(r3 * 10), mode)
    frame = build_frame(MSG_CONTROL_CMD, payload)
    ser.write(frame)
    ser.flush()
    print(f"  >> M0={r0} M1={r1} M2={r2} M3={r3} | {frame.hex()}")


def send_estop(ser):
    frame = build_frame(MSG_ESTOP)
    ser.write(frame)
    ser.flush()
    print(f"  >> ESTOP | {frame.hex()}")


def rx_thread(ser, stop_event):
    while not stop_event.is_set():
        try:
            msg_type, payload = read_frame(ser)
            if msg_type == MSG_ACTUATOR_FB:
                print_fb(payload)
            elif msg_type == MSG_HEARTBEAT:
                pass
            else:
                if not QUIET:
                    print(f"  << unknown type={msg_type:#04x}")
        except serial.SerialException:
            break
        except Exception as e:
            if not stop_event.is_set():
                print(f"  [RX ERR] {e}")


HELP = """
Commands:
  m <r0> <r1> <r2> <r3>   send RPM targets  (e.g. m 50 50 50 50)
  s                        stop all motors
  e                        E-STOP
  q                        quit
"""


def main():
    print(f"Connecting {PORT} @ {BAUDRATE} {'[QUIET]' if QUIET else '[TELEMETRY ON]'}")
    try:
        ser = serial.Serial(PORT, BAUDRATE, timeout=1)
    except serial.SerialException as e:
        print(f"ERROR: {e}")
        sys.exit(1)

    time.sleep(2.0)
    ser.reset_input_buffer()
    ser.reset_output_buffer()
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
                if len(parts) != 5:
                    print("usage: m r0 r1 r2 r3")
                    continue
                try:
                    send_motor_targets(ser,
                                       float(parts[1]), float(parts[2]),
                                       float(parts[3]), float(parts[4]))
                except ValueError:
                    print("numbers only")
            elif cmd == "s":
                send_motor_targets(ser, 0, 0, 0, 0)
            elif cmd == "e":
                send_estop(ser)
            else:
                print(HELP)

    except KeyboardInterrupt:
        pass
    finally:
        stop_event.set()
        send_motor_targets(ser, 0, 0, 0, 0)
        ser.close()
        print("\nDisconnected.")


if __name__ == "__main__":
    main()
