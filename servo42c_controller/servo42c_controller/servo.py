from rclpy.node import Node
from logging import Logger
from math import pi, radians
from .protocol import Servo42CProtocol

# Motor configuration
STEPS_PER_REV = 200  # Base steps per revolution
MICROSTEP_FACTOR = 8  # Microstepping factor
GEAR_RATIO = 37      # Gear reduction ratio

# defaults (motion limits in radians)
MIN_ANGLE = radians(-360.0)  # -2π radians
MAX_ANGLE = radians(360.0)   # 2π radians
POSITION_TOLERANCE = radians(0.5)  # ~0.00873 radians

SPEED_MAP = {
    5:  0x09,
    10: 0x4B,
    15: 0x62,
    20: 0x65,
    25: 0x69,
    30: 0x6E,
    35: 0x71,
    40: 0x72,
    50: 0x75,
    75: 0x79,
    100: 0x7A,
    150: 0x7C,
    200: 0x7D,
    250: 0x7D,
    300: 0x7D,
    350: 0x7D,
    400: 0x7E,
}


def get_xx_for_rpm(target_rpm):
    nearest_rpm = min(SPEED_MAP.keys(), key=lambda x: abs(x - target_rpm))
    return nearest_rpm, SPEED_MAP[nearest_rpm]


class Servo:
    """Class representing a single servo motor"""

    def __init__(self,
                 node: Node,
                 protocol: Servo42CProtocol,
                 servo_id: int,
                 logger: Logger,
                 min_angle: float = MIN_ANGLE,
                 max_angle: float = MAX_ANGLE,
                 position_tolerance: float = POSITION_TOLERANCE,
                 steps_per_rev: int = STEPS_PER_REV,
                 microstep_factor: int = MICROSTEP_FACTOR,
                 gear_ratio: int = GEAR_RATIO,
                 modbus_acceleration: int = 0xF0,
                 modbus_speed: int = 0xFF,
                 name: str = None
                 ):
        """Initialize servo instance"""
        self.protocol = protocol
        self.id = servo_id
        self.target_pulses = 0
        self.logger = logger
        self.min_angle = min_angle
        self.max_angle = max_angle
        self.position_tolerance = position_tolerance
        self.pulses_per_rotation = steps_per_rev * microstep_factor * gear_ratio
        self.is_enabled = False
        self.node = node  # Store node reference
        self.microstep_factor = microstep_factor
        self.name = name
        self.modbus_acceleration = modbus_acceleration & 0xFFFF
        self.modbus_speed = modbus_speed & 0xFFFF

    def _angle_to_pulses(self, angle_rad: float) -> int:
        """Convert angle in radians to pulses"""
        return round(angle_rad * self.pulses_per_rotation / (2 * pi))

    def _pulses_to_angle(self, pulses: int) -> float:
        """Convert pulses to angle in radians"""
        return float(pulses) * (2 * pi) / self.pulses_per_rotation

    def initialize(self) -> bool:
        """Initialize servo and get current position via Modbus"""
        try:
            self.logger.info(f'Initializing servo {self.id}')
            # Mark enabled and fetch initial position
            self.is_enabled = True
            self.target_pulses = self.protocol.get_pulses(self.id)

            self.logger.info(f'Successfully initialized servo {self.id}')
            return True

        except Exception as e:
            self.logger.error(
                f'Failed to initialize servo {self.id}: {str(e)}')
            return False

    def rotate(self, angle: float, rad_per_sec: float, accel_override: int = None, speed_override: int = None) -> bool:
        """Command absolute target angle (radians) using Modbus absolute move.

        accel_override and speed_override, if provided, are raw device units (0..65535).
        """

        self.logger.info(f'Rotating servo {self.id} to angle (rad): {angle}, rad_per_sec: {rad_per_sec}, accel_override: {accel_override}, speed_override: {speed_override}')
        if not self.is_enabled:
            self.logger.warn(f'Servo {self.id} is currently disabled')
            return False

        if not self.min_angle <= angle <= self.max_angle:
            self.logger.error(
                f'Angle {angle} out of bounds for servo {self.id}')
            return False

        new_target_pulses = self._angle_to_pulses(angle)
        if new_target_pulses == self.target_pulses:
            return True

        try:
            acceleration = self.modbus_acceleration if accel_override is None else (accel_override & 0xFFFF)
            speed = self.modbus_speed if speed_override is None else (speed_override & 0xFFFF)
            if self.protocol.move_absolute(self.id, acceleration, speed, new_target_pulses):
                self.target_pulses = new_target_pulses
                self.logger.info(f'Moving servo {self.id} to angle (rad): {angle}')
                return True
        except Exception as e:
            self.logger.error(
                f'Failed to rotate servo {self.id}: {str(e)}')

        return False

    def stop(self) -> None:
        """Stop servo movement"""
        try:
            self.protocol.stop(self.id)
            self.logger.info(f'Emergency stop sent to servo {self.id}')
        except Exception as e:
            self.logger.error(
                f'Failed to stop servo {self.id}: {str(e)}')

    def cleanup(self) -> None:
        """Clean up resources"""
        try:
            # Stop any ongoing movement
            if self.is_enabled:
                try:
                    self.protocol.stop(self.id)
                except Exception:
                    pass
                self.is_enabled = False

        except Exception:
            pass

    def __del__(self):
        """Destructor to ensure cleanup"""
        try:
            self.cleanup()
        except Exception:
            pass

    def get_angle(self) -> float:
        """Get current angle"""
        current_pulses = self.protocol.get_pulses(self.id)
        return self._pulses_to_angle(current_pulses)
