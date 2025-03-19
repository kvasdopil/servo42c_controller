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
        self.is_enabled = True

        # Publisher
        self.position_publisher = node.create_publisher(
            Float32,
            f'servo42c/servo_{servo_id}/position',
            10
        )

        # Subscribers
        self.command_subscriber = node.create_subscription(
            Float32,
            f'servo42c/servo_{servo_id}/command',
            self._command_angle_callback,
            10
        )

        # Emergency stop subscriber
        self.emergency_stop_subscriber = node.create_subscription(
            Bool,
            f'servo42c/servo_{servo_id}/emergency_stop',
            self._emergency_stop_callback,
            1  # Higher priority QoS
        )

    def log(self, level: str, msg: str):
        """Log message if logger is available"""
        if self.logger:
            getattr(self.logger, level)(msg)

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

            self.current_pulses = self.protocol.get_pulses(self.id)
            self.target_pulses = self.current_pulses
            self.publish_status()
            return True

        except Exception as e:
            self.log(
                'error', f'Failed to initialize servo {self.id}: {str(e)}')
            return False

    def rotate(self, angle: float, speed: int = 120) -> bool:
        """Rotate servo to specified angle"""
        if not self.is_enabled:
            self.log('warn', f'Servo {self.id} is currently disabled')
            return False

        if not self.min_angle <= angle <= self.max_angle:
            self.log(
                'error', f'Angle {angle} out of bounds for servo {self.id}')
            return False

        if not self.min_speed <= speed <= self.max_speed:
            self.log(
                'error', f'Speed {speed} out of bounds for servo {self.id}')
            return False

        new_target_pulses = self.angle_to_pulses(angle)
        diff = new_target_pulses - self.current_pulses

        # If no movement is needed, return success immediately
        if diff == 0:
            return True

        try:
            if self.protocol.rotate(self.id, speed, diff):
                self.target_pulses = new_target_pulses
                self.log('info', f'Moving servo {self.id} to angle: {angle}')
                return True
        except Exception as e:
            self.log('error', f'Failed to rotate servo {self.id}: {str(e)}')

        return False

    def stop(self) -> None:
        """Stop servo movement"""
        try:
            self.protocol.stop(self.id)
            self.log('info', f'Emergency stop sent to servo {self.id}')
        except Exception as e:
            self.log('error', f'Failed to stop servo {self.id}: {str(e)}')

    def cleanup(self) -> None:
        """Clean up ROS2 resources"""
        try:
            # Stop any ongoing movement
            self.stop()

            # Destroy publishers and subscribers
            self.position_publisher.destroy()
            self.command_subscriber.destroy()
            self.emergency_stop_subscriber.destroy()

            self.log('info', f'Cleaned up resources for servo {self.id}')
        except Exception as e:
            self.log(
                'error', f'Error during cleanup for servo {self.id}: {str(e)}')

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
            self.log(
                'error', f'Failed to update position for servo {self.id}: {str(e)}')

    def publish_status(self) -> None:
        """Publish current position"""
        try:
            position_msg = Float32()
            position_msg.data = self.pulses_to_angle(self.current_pulses)
            self.position_publisher.publish(position_msg)
        except Exception as e:
            self.log(
                'error', f'Failed to publish status for servo {self.id}: {str(e)}')

    def _command_angle_callback(self, msg: Float32) -> None:
        """Handle command angle messages (in radians)"""
        try:
            self.rotate(msg.data)
        except Exception as e:
            self.log(
                'error', f'Failed to handle command angle for servo {self.id}: {str(e)}')

    def _emergency_stop_callback(self, msg: Bool) -> None:
        """Handle emergency stop messages"""
        try:
            if msg.data:  # If True, trigger emergency stop
                self.stop()
                self.is_enabled = False
                self.log(
                    'warn', f'Emergency stop triggered for servo {self.id}')
            else:  # If False, re-enable the servo
                self.is_enabled = True
                self.log('info', f'Servo {self.id} re-enabled')
        except Exception as e:
            self.log(
                'error', f'Failed to handle emergency stop for servo {self.id}: {str(e)}')
