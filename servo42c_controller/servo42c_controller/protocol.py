import rclpy
import serial
from typing import List, Optional
import threading

# Constants for commands
GET_PULSES = 0x33
GET_SERIAL_ENABLED = 0xf3
ROTATE = 0xfd
STOP = 0xf7

SERIAL_TIMEOUT = 1.0  # seconds
MAX_RECONNECT_ATTEMPTS = 3


class Servo42CProtocol:
    """Protocol implementation for Servo42C communication"""

    def __init__(self, device: str, baud_rate: int, logger=None):
        self.device = device
        self.baud_rate = baud_rate
        self.logger = logger
        self.serial_port: Optional[serial.Serial] = None
        self.serial_lock = threading.Lock()

    def log(self, level: str, msg: str):
        """Log message if logger is available"""
        if self.logger:
            getattr(self.logger, level)(msg)

    def connect(self) -> bool:
        """Establish serial connection with retries"""
        for attempt in range(MAX_RECONNECT_ATTEMPTS):
            try:
                if self.serial_port is not None:
                    self.serial_port.close()

                self.serial_port = serial.Serial(
                    f'/dev/{self.device}',
                    self.baud_rate,
                    timeout=SERIAL_TIMEOUT,
                    write_timeout=SERIAL_TIMEOUT
                )
                self.log('info', f'Connected to /dev/{self.device}')
                return True
            except serial.SerialException as e:
                self.log(
                    'error', f'Attempt {attempt + 1}/{MAX_RECONNECT_ATTEMPTS} failed: {str(e)}')
                if attempt < MAX_RECONNECT_ATTEMPTS - 1:
                    self.log('info', 'Retrying in 1 second...')
                    rclpy.sleep(1.0)

        self.log('error', 'Failed to establish serial connection')
        return False

    def ensure_connection(self) -> bool:
        """Ensure serial connection is active"""
        if self.serial_port is None or not self.serial_port.is_open:
            self.log('warn', 'Serial connection lost, attempting to reconnect...')
            return self.connect()
        return True

    def write(self, data: bytes) -> None:
        """Write data to serial port"""
        with self.serial_lock:
            if not self.ensure_connection():
                raise RuntimeError('Serial port not available')

            try:
                self.serial_port.write(data)
                self.serial_port.flush()
            except serial.SerialException as e:
                self.log('error', f'Failed to write to serial port: {str(e)}')
                raise

    def read(self, id: int, num_bytes: int) -> List[int]:
        """Read data from serial port"""
        with self.serial_lock:
            if not self.ensure_connection():
                raise RuntimeError('Serial port not available')

            try:
                data = self.serial_port.read(num_bytes + 1)  # +1 for id byte
                if len(data) != num_bytes + 1:
                    raise RuntimeError('Timeout reading from serial port')

                if data[0] != 0xe0 + id:
                    raise RuntimeError(
                        f'Invalid ID received: expected {0xe0 + id}, got {data[0]}')

                return list(data[1:])
            except serial.SerialException as e:
                self.log('error', f'Failed to read from serial port: {str(e)}')
                raise

    def send_command(self, id: int, msg: List[int], return_length: int = 1) -> List[int]:
        """Send command and read response"""
        self.write(bytes([0xe0 + id] + msg))
        if return_length > 0:
            response = self.read(id, return_length)
            return response
        return []

    def get_pulses(self, id: int) -> int:
        """Get current pulse count"""
        [a, b, c, d] = self.send_command(id, [GET_PULSES], 4)
        # Combine bytes into a signed 32-bit integer
        value = (a << 24) + (b << 16) + (c << 8) + d
        # Handle two's complement for negative values
        if value & 0x80000000:  # If highest bit is set (negative)
            value = -((~value & 0xFFFFFFFF) + 1)
        return value

    def rotate(self, id: int, speed: int, pulses: int) -> bool:
        """Rotate servo by specified number of pulses at given speed"""
        try:
            # Calculate pulses preserving sign
            abs_pulses = abs(pulses)
            sign_bit = 0b10000000 if pulses > 0 else 0

            while abs_pulses > 0:
                p = min(abs_pulses, 0xffff)
                abs_pulses -= p

                [ok] = self.send_command(id, [
                    ROTATE,
                    sign_bit + speed,
                    (p >> 8) & 0xff,
                    p & 0xff
                ], 1)

                if ok != 1:
                    return False

            return True

        except Exception as e:
            self.log('error', f'Failed to rotate servo {id}: {str(e)}')
            return False

    def stop(self, id: int) -> None:
        """Stop servo movement"""
        self.send_command(id, [STOP], 0)

    def get_serial_enabled(self, id: int) -> bool:
        """Check if serial is enabled for a servo"""
        if not self.serial_port:
            raise RuntimeError('Serial port not open')

        with self.serial_lock:
            self.serial_port.write(bytes([0xe0 + id, GET_SERIAL_ENABLED, 1]))
            data = self.serial_port.read(2)  # Read ID byte and status

            if len(data) != 2:
                raise RuntimeError('Timeout reading from serial port')

            if data[0] != 0xe0 + id:
                raise RuntimeError('Invalid ID received')

            return data[1] == 1

    def close(self) -> None:
        """Close serial connection"""
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
