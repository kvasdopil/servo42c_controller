import serial
import threading
import rclpy
from typing import Optional

# default values
SERIAL_TIMEOUT = 1.0  # seconds
MAX_RECONNECT_ATTEMPTS = 3


class SerialPort:
    """Generic serial port communication handler"""

    def __init__(self, device: str, baud_rate: int, logger=None, timeout=SERIAL_TIMEOUT, max_attempts=MAX_RECONNECT_ATTEMPTS):
        self.device = device
        self.baud_rate = baud_rate
        self.logger = logger
        self.timeout = timeout
        self.max_attempts = max_attempts
        self.serial_port: Optional[serial.Serial] = None
        self.serial_lock = threading.Lock()

    def connect(self) -> bool:
        """Establish serial connection with retries"""
        for attempt in range(self.max_attempts):
            try:
                if self.serial_port is not None:
                    self.serial_port.close()

                self.serial_port = serial.Serial(
                    f'/dev/{self.device}',
                    self.baud_rate,
                    timeout=self.timeout,
                    write_timeout=self.timeout
                )
                self.logger.info(f'Connected to /dev/{self.device}')
                return True
            except serial.SerialException as e:
                self.logger.error(
                    f'Attempt {attempt + 1}/{self.max_attempts} failed: {str(e)}')
                if attempt < self.max_attempts - 1:
                    self.logger.info('Retrying in 1 second...')
                    rclpy.sleep(1.0)

        self.logger.error('Failed to establish serial connection')
        return False

    def ensure_connection(self) -> bool:
        """Ensure serial connection is active"""
        if self.serial_port is None or not self.serial_port.is_open:
            self.logger.warn(
                'Serial connection lost, attempting to reconnect...')
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
                self.logger.error(
                    f'Failed to write to serial port: {str(e)}')
                raise

    def read(self, num_bytes: int) -> bytes:
        """Read data from serial port"""
        with self.serial_lock:
            if not self.ensure_connection():
                raise RuntimeError('Serial port not available')

            try:
                data = self.serial_port.read(num_bytes)
                if len(data) != num_bytes:
                    raise RuntimeError('Timeout reading from serial port')
                return data
            except serial.SerialException as e:
                self.logger.error(
                    f'Failed to read from serial port: {str(e)}')
                raise

    def close(self) -> None:
        """Close serial connection"""
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
            self.logger.info('Serial connection closed')