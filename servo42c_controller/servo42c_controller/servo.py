from rclpy.node import Node
from std_msgs.msg import Float32, Bool
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
                 gear_ratio: int = GEAR_RATIO
                 ):
        """Initialize servo instance"""
        self.protocol = protocol
        self.id = servo_id
        self.current_pulses = 0
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
        
        # Don't create publishers/subscribers yet
        self.position_publisher = None
        self.command_subscriber = None
        self.emergency_stop_subscriber = None
        self.speed_subscriber = None
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
            if not self.protocol.get_serial_enabled(self.id):
                return False
            
            self.is_enabled = True

            # Set microsteps per revolution
            self.protocol.set_msteps(self.id, self.microstep_factor)

            # Get initial position
            self.current_pulses = self.protocol.get_pulses(self.id)
            self.target_pulses = self.current_pulses

            # Only create publishers and subscribers if servo is found
            self.position_publisher = self.node.create_publisher(
                Float32,
                f'servo42c/servo_{self.id}/position',
                10
            )

            self.command_subscriber = self.node.create_subscription(
                Float32,
                f'servo42c/servo_{self.id}/command',
                self._command_angle_callback,
                10
            )

            self.emergency_stop_subscriber = self.node.create_subscription(
                Bool,
                f'servo42c/servo_{self.id}/emergency_stop',
                self._emergency_stop_callback,
                1
            )

            self.speed_subscriber = self.node.create_subscription(
                Float32,
                f'servo42c/servo_{self.id}/set_speed',
                self._speed_callback,
                10
            )

            self.publish_status()
            self.logger.info(f'Successfully initialized servo {self.id}')
            return True

        except Exception as e:
            self.logger.error(f'Failed to initialize servo {self.id}: {str(e)}')
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
        diff = new_target_pulses - self.current_pulses

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
        """Clean up ROS2 resources"""
        try:
            # Stop any ongoing movement
            if self.is_enabled:
                self.stop()

            # Only destroy if publishers/subscribers were created
            if self.position_publisher:
                self.position_publisher.destroy()
            if self.command_subscriber:
                self.command_subscriber.destroy()
            if self.emergency_stop_subscriber:
                self.emergency_stop_subscriber.destroy()
            if self.speed_subscriber:
                self.speed_subscriber.destroy()

            self.logger.info(f'Cleaned up resources for servo {self.id}')
        except Exception as e:
            self.logger.error(f'Error during cleanup for servo {self.id}: {str(e)}')

    def __del__(self):
        """Destructor to ensure cleanup"""
        try:
            self.cleanup()
        except Exception:
            # Ignore errors during destructor
            pass

    def update_position(self) -> None:
        """Update current position"""
        try:
            self.current_pulses = self.protocol.get_pulses(self.id)
            self.publish_status()
        except Exception as e:
            self.logger.error(
                f'Failed to update position for servo {self.id}: {str(e)}')

    def publish_status(self) -> None:
        """Publish current position"""
        try:
            position_msg = Float32()
            position_msg.data = self.pulses_to_angle(self.current_pulses)
            self.position_publisher.publish(position_msg)
        except Exception as e:
            self.logger.error(
                f'Failed to publish status for servo {self.id}: {str(e)}')

    def _command_angle_callback(self, msg: Float32) -> None:
        """Handle command angle messages (in radians)"""
        try:
            angle = msg.data
            self.rotate(angle, self.current_speed)
        except Exception as e:
            self.logger.error(
                f'Failed to handle command angle for servo {self.id}: {str(e)}')

    def _emergency_stop_callback(self, msg: Bool) -> None:
        """Handle emergency stop messages"""
        try:
            if msg.data:  # If True, trigger emergency stop
                self.stop()
                self.is_enabled = False
                self.logger.warn(
                    f'Emergency stop triggered for servo {self.id}')
            else:  # If False, re-enable the servo
                self.is_enabled = True
                self.logger.info(f'Servo {self.id} re-enabled')
        except Exception as e:
            self.logger.error(
                f'Failed to handle emergency stop for servo {self.id}: {str(e)}')

    def _speed_callback(self, msg: Float32) -> None:
        """Handle speed command messages"""
        try:
            speed = int(msg.data)  # Convert float to int for internal speed units
            if self.min_speed <= speed <= self.max_speed:
                self.current_speed = speed
                self.logger.info(f'Speed updated for servo {self.id}: {speed}')
            else:
                self.logger.error(
                    f'Speed {speed} out of bounds for servo {self.id} (min: {self.min_speed}, max: {self.max_speed})')
        except Exception as e:
            self.logger.error(
                f'Failed to handle speed update for servo {self.id}: {str(e)}')
