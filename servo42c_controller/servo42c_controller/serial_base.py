import serial
import asyncio
from typing import List, Optional
import rclpy

class SerialBase:
    def __init__(self, device: str, baud_rate: int, timeout: float = 1.0, max_reconnect_attempts: int = 3):
        self.device = device
        self.baud_rate = baud_rate
        self.timeout = timeout
        self.max_reconnect_attempts = max_reconnect_attempts
        self.serial_port: Optional[serial.Serial] = None
        self.serial_lock = asyncio.Lock()
        self.logger = rclpy.logging.get_logger('serial_base')
        
    async def _ensure_connection(self) -> bool:
        """Ensure serial connection is active, attempt reconnection if needed"""
        if self.serial_port is None or not self.serial_port.is_open:
            self.logger.warn('Serial connection lost, attempting to reconnect...')
            try:
                self._setup_serial()
            except Exception as e:
                self.logger.error(f'Failed to reconnect: {str(e)}')
                return False
        return True
        
    def _setup_serial(self):
        """Initialize serial port communication with reconnection logic"""
        for attempt in range(self.max_reconnect_attempts):
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
                return
            except serial.SerialException as e:
                self.logger.error(f'Attempt {attempt + 1}/{self.max_reconnect_attempts} failed: {str(e)}')
                if attempt < self.max_reconnect_attempts - 1:
                    self.logger.info('Retrying in 1 second...')
                    rclpy.sleep(1.0)
                else:
                    self.logger.error('Failed to establish serial connection')
                    raise

    async def write(self, data: bytes) -> None:
        """Write data to serial port with connection check"""
        async with self.serial_lock:
            if not await self._ensure_connection():
                raise RuntimeError('Serial port not available')
            
            try:
                self.serial_port.write(data)
                self.serial_port.flush()
            except serial.SerialException as e:
                self.logger.error(f'Failed to write to serial port: {str(e)}')
                raise

    async def read(self, num_bytes: int) -> bytes:
        """Read specified number of bytes from serial port with connection check"""
        async with self.serial_lock:
            if not await self._ensure_connection():
                raise RuntimeError('Serial port not available')
                
            try:
                data = self.serial_port.read(num_bytes)
                if len(data) != num_bytes:
                    raise RuntimeError('Timeout reading from serial port')
                return data
            except serial.SerialException as e:
                self.logger.error(f'Failed to read from serial port: {str(e)}')
                raise

    def cleanup(self):
        """Clean up serial resources"""
        if self.serial_port and self.serial_port.is_open:
            try:
                self.serial_port.close()
            except Exception as e:
                self.logger.error(f'Error closing serial port: {str(e)}') 