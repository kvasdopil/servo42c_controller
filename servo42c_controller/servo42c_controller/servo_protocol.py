from .serial_base import SerialBase
from typing import List, Tuple

class ServoProtocol(SerialBase):
    # Constants for commands
    GET_PULSES = 0x33
    GET_SERIAL_ENABLED = 0xf3
    ROTATE = 0xfd
    STOP = 0xf7

    async def send_command(self, servo_id: int, command: int, data: List[int] = [], response_length: int = 1) -> List[int]:
        """Send a command to a servo and read the response"""
        # Construct and send command
        command_bytes = bytes([0xe0 + servo_id, command] + data)
        await self.write(command_bytes)
        
        # Read and validate response
        response = await self.read(response_length + 1)  # +1 for ID byte
        if response[0] != 0xe0 + servo_id:
            raise RuntimeError(f'Invalid ID received: expected {0xe0 + servo_id}, got {response[0]}')
        
        return list(response[1:])

    async def get_pulses(self, servo_id: int) -> int:
        """Get current pulse count from servo"""
        [a, b, c, d] = await self.send_command(servo_id, self.GET_PULSES, response_length=4)
        # Combine bytes into a signed 32-bit integer
        value = (a << 24) + (b << 16) + (c << 8) + d
        # Handle two's complement for negative values
        if value & 0x80000000:  # If highest bit is set (negative)
            value = -((~value & 0xFFFFFFFF) + 1)
        return value

    async def get_serial_enabled(self, servo_id: int) -> bool:
        """Check if serial control is enabled for servo"""
        [status] = await self.send_command(servo_id, self.GET_SERIAL_ENABLED, [1], 1)
        return status == 1

    async def rotate(self, servo_id: int, speed: int, pulses: int) -> bool:
        """Send rotation command to servo"""
        sign_bit = 0b10000000 if pulses > 0 else 0
        abs_pulses = abs(pulses)
        
        while abs_pulses > 0:
            p = min(abs_pulses, 0xffff)
            abs_pulses -= p
            
            [ok] = await self.send_command(
                servo_id,
                self.ROTATE,
                [sign_bit + speed, (p >> 8) & 0xff, p & 0xff],
                1
            )
            
            if ok != 1:
                return False
                
        return True

    async def stop(self, servo_id: int) -> bool:
        """Send stop command to servo"""
        [ok] = await self.send_command(servo_id, self.STOP)
        return ok == 1 