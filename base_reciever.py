import serial
import struct
import time

class RPODMotorController:
    # Frame Constants
    SYNC1 = 0xAA
    SYNC2 = 0x55
    SYNC3 = 0xA5
    
    # Message Types
    MSG_CONTROL_CMD = 0x01

    def __init__(self, port='/dev/ttyUSB0', baudrate=921600):
        try:
            self.ser = serial.Serial(port, baudrate, timeout=0.1)
            print(f"Successfully connected to STM32 on {port} at {baudrate} baud.")
        except serial.SerialException as e:
            print(f"Failed to connect to STM32: {e}")
            raise

    def _calc_crc8(self, data: bytes) -> int:
        crc = 0x00
        for b in data:
            crc ^= b
            for _ in range(8):
                if crc & 0x80:
                    crc = ((crc << 1) ^ 0x07) & 0xFF
                else:
                    crc = (crc << 1) & 0xFF
        return crc

    def send_rpm_targets(self, rpm1: float, rpm2: float, rpm3: float, rpm4: float, mode: int = 0):
        # Convert floats to RPM x 10 integers
        r1_10 = int(rpm1 * 10.0)
        r2_10 = int(rpm2 * 10.0)
        r3_10 = int(rpm3 * 10.0)
        r4_10 = int(rpm4 * 10.0)

        # Pack payload: 4 little-endian short integers and 1 unsigned char (mode)
        payload = struct.pack('<4hB', r1_10, r2_10, r3_10, r4_10, mode)
        payload_len = len(payload)

        # Build header and calculate CRC
        header = struct.pack('<BB', self.MSG_CONTROL_CMD, payload_len)
        crc = self._calc_crc8(header + payload)

        # Construct and send final frame
        sync_bytes = struct.pack('<BBB', self.SYNC1, self.SYNC2, self.SYNC3)
        frame = sync_bytes + header + payload + struct.pack('<B', crc)

        self.ser.write(frame)
        self.ser.flush()

    def close(self):
        # HACK: Instead of a hard E-STOP, we send 0 RPM. 
        # This stops the motors but leaves the STM32 control loop awake for the next script run.
        print("Soft stopping motors (Sending 0 RPM)...")
        self.send_rpm_targets(0.0, 0.0, 0.0, 0.0)
        time.sleep(0.1) # Give the UART buffer a split second to flush
        if self.ser.is_open:
            self.ser.close()


if __name__ == '__main__':
    # Connect to the board
    chaser_drive = RPODMotorController(port='/dev/ttyUSB0', baudrate=921600)

    try:
        print("Spinning motor... (Expect 100% speed due to no encoders)")
        
        # Ask for 50 RPM on Motor 0 (PA8/PB13). The others stay at 0.
        chaser_drive.send_rpm_targets(50.0, 0.0, 0.0, 0.0)
        time.sleep(2)
        
        print("Reversing motor...")
        chaser_drive.send_rpm_targets(-50.0, 0.0, 0.0, 0.0)
        time.sleep(2)

    except KeyboardInterrupt:
        print("\nProcess interrupted by user.")
    finally:
        # Guarantee motors stop safely without locking up the STM32
        chaser_drive.close()
