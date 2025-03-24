from rclpy.node import Node
from logging import Logger
from math import pi, radians
from .protocol import Servo42CProtocol

# Motor configuration
STEPS_PER_REV = 200  # Base steps per revolution
MICROSTEP_FACTOR = 8  # Microstepping factor
GEAR_RATIO = 37      # Gear reduction ratio

# defaults
MAX_SPEED = 128
MIN_SPEED = 1
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
                 max_speed: int = MAX_SPEED,
                 min_speed: int = MIN_SPEED,
                 steps_per_rev: int = STEPS_PER_REV,
                 microstep_factor: int = MICROSTEP_FACTOR,
                 gear_ratio: int = GEAR_RATIO,
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
        self.max_speed = max_speed
        self.min_speed = min_speed
        self.pulses_per_rotation = steps_per_rev * microstep_factor * gear_ratio
        self.is_enabled = False
        self.node = node  # Store node reference
        self.microstep_factor = microstep_factor
        self.name = name
        self.current_speed = 120  # Default speed in internal units

    def angle_to_pulses(self, angle_rad: float) -> int:
        """Convert angle in radians to pulses"""
        return round(angle_rad * self.pulses_per_rotation / (2 * pi))

    def pulses_to_angle(self, pulses: int) -> float:
        """Convert pulses to angle in radians"""
        return float(pulses) * (2 * pi) / self.pulses_per_rotation

    def initialize(self) -> bool:
        """Initialize servo and get current position"""
        try:
            self.logger.info(f'Initializing servo {self.id}')
            if not self.protocol.get_serial_enabled(self.id):
                return False

            self.is_enabled = True

            # Set microsteps per revolution
            self.protocol.set_msteps(self.id, self.microstep_factor)

            # Get initial position
            self.target_pulses = self.protocol.get_pulses(self.id)

            self.logger.info(f'Successfully initialized servo {self.id}')
            return True

        except Exception as e:
            self.logger.error(
                f'Failed to initialize servo {self.id}: {str(e)}')
            return False

    def rotate(self, angle: float, speed: int) -> bool:
        """Rotate servo to specified angle"""
        if not self.is_enabled:
            self.logger.warn(f'Servo {self.id} is currently disabled')
            return False

        if not self.min_angle <= angle <= self.max_angle:
            self.logger.error(
                f'Angle {angle} out of bounds for servo {self.id}')
            return False

        if not self.min_speed <= speed <= self.max_speed:
            self.logger.error(
                f'Speed {speed} out of bounds for servo {self.id}')
            return False

        new_target_pulses = self.angle_to_pulses(angle)
        diff = new_target_pulses - self.target_pulses

        # If no movement is needed, return success immediately
        if diff == 0:
            return True

        try:
            if self.protocol.rotate(self.id, speed, diff):
                self.target_pulses = new_target_pulses
                self.logger.info(f'Moving servo {self.id} to angle: {angle}')
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

    def get_pulses(self) -> int:
        """Get current pulses"""
        return self.protocol.get_pulses(self.id)
    
    def get_angle(self) -> float:
        """Get current angle"""
        current_pulses = self.get_pulses()
        return self.pulses_to_angle(current_pulses)
