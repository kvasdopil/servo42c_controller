from typing import List
from .serial_port import SerialPort

# Constants for commands
GET_PULSES = 0x33
GET_SERIAL_ENABLED = 0xf3
ROTATE = 0xfd
STOP = 0xf7
SET_MSTEPS = 0x84

ID_BYTE = 0xe0


class Servo42CProtocol:
    """Protocol implementation for Servo42C communication"""

    def __init__(self, device: str, baud_rate: int, logger=None):
        self.serial = SerialPort(device, baud_rate, logger)

    def connect(self) -> bool:
        """Establish connection with the servo controller"""
        return self.serial.connect()

    def send_command(self, id: int, msg: List[int], return_length: int = 1) -> List[int]:
        """Send command and read response"""
        self.serial.write(bytes([ID_BYTE + id] + msg))
        if return_length > 0:
            response = self.serial.read(return_length + 1)  # +1 for id byte
            if response[0] != ID_BYTE + id:
                raise RuntimeError(
                    f'Invalid ID received: expected {ID_BYTE + id}, got {response[0]}')
            return list(response[1:])
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
    
    def set_msteps(self, id: int, msteps: int) -> bool:
        """Set microsteps per revolution"""
        return self.send_command(id, [SET_MSTEPS, msteps], 1) == [1]

    def rotate(self, id: int, speed: int, pulses: int) -> bool:
        """Rotate servo by specified number of pulses at given speed"""

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

    def stop(self, id: int) -> None:
        """Stop servo movement"""
        self.send_command(id, [STOP], 0)

    def get_serial_enabled(self, id: int) -> bool:
        """Check if serial is enabled for a servo"""
        self.serial.write(bytes([0xe0 + id, GET_SERIAL_ENABLED, 1]))
        data = self.serial.read(2)  # Read ID byte and status
        if data[0] != 0xe0 + id:
            raise RuntimeError('Invalid ID received')
        return data[1] == 1

    def close(self) -> None:
        """Close serial connection"""
        self.serial.close()
