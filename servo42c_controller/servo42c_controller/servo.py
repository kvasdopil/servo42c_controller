from rclpy.node import Node
from std_msgs.msg import Float32, Bool
from logging import Logger
from .protocol import Servo42CProtocol

# Motor configuration
STEPS_PER_REV = 200  # Base steps per revolution
MICROSTEP_FACTOR = 8  # Microstepping factor
GEAR_RATIO = 37      # Gear reduction ratio

# defaults
MAX_SPEED = 128
MIN_SPEED = 1
MIN_ANGLE = -360.0  # degrees
MAX_ANGLE = 360.0   # degrees
POSITION_TOLERANCE = 0.5  # degrees


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
        self.at_target = True
        self.logger = logger
        self.min_angle = min_angle
        self.max_angle = max_angle
        self.position_tolerance = position_tolerance
        self.max_speed = max_speed
        self.min_speed = min_speed
        self.pulses_per_rotation = steps_per_rev * microstep_factor * gear_ratio

        # Publishers
        self.angle_publisher = node.create_publisher(
            Float32,
            f'servo42c/servo_{servo_id}/angle',
            10
        )
        self.status_publisher = node.create_publisher(
            Bool,
            f'servo42c/servo_{servo_id}/at_target',
            10
        )

        # Subscriber
        self.target_subscriber = node.create_subscription(
            Float32,
            f'servo42c/servo_{servo_id}/target',
            self._target_angle_callback,
            10
        )

    def log(self, level: str, msg: str):
        """Log message if logger is available"""
        if self.logger:
            getattr(self.logger, level)(msg)

    def angle_to_pulses(self, angle: float) -> int:
        """Convert angle in degrees to pulses"""
        return round(angle * self.pulses_per_rotation / 360)

    def pulses_to_angle(self, pulses: int) -> float:
        """Convert pulses to angle in degrees"""
        return float(pulses) * 360.0 / self.pulses_per_rotation

    def initialize(self) -> bool:
        """Initialize servo and get current position"""
        try:
            if not self.protocol.get_serial_enabled(self.id):
                return False

            self.current_pulses = self.protocol.get_pulses(self.id)
            self.target_pulses = self.current_pulses
            self.at_target = True
            self.publish_status()
            return True

        except Exception as e:
            self.log(
                'error', f'Failed to initialize servo {self.id}: {str(e)}')
            return False

    def rotate(self, angle: float, speed: int = 120) -> bool:
        """Rotate servo to specified angle"""
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

        # Update status before starting movement
        self.at_target = False
        self.publish_status()

        try:
            if self.protocol.rotate(self.id, speed, diff):
                self.target_pulses = new_target_pulses
                self.log('info', f'Moving servo {self.id} to angle: {angle}')
                return True
        except Exception as e:
            self.log('error', f'Failed to rotate servo {self.id}: {str(e)}')

        # Reset status on failure
        self.at_target = True
        self.publish_status()
        return False

    def stop(self) -> None:
        """Stop servo movement"""
        try:
            self.protocol.stop(self.id)
            self.log('info', f'Emergency stop sent to servo {self.id}')
        except Exception as e:
            self.log('error', f'Failed to stop servo {self.id}: {str(e)}')

    def update_position(self) -> None:
        """Update current position and check if target reached"""
        try:
            self.current_pulses = self.protocol.get_pulses(self.id)
            current_angle = self.pulses_to_angle(self.current_pulses)

            # Check if at target
            at_target = abs(
                current_angle - self.pulses_to_angle(self.target_pulses)) <= self.position_tolerance

            # Publish updates if status changed
            if at_target != self.at_target:
                self.at_target = at_target
                if at_target:
                    self.log(
                        'info', f'Servo {self.id} reached target position')

            self.publish_status()

        except Exception as e:
            self.log(
                'error', f'Failed to update position for servo {self.id}: {str(e)}')

    def publish_status(self) -> None:
        """Publish current angle and at-target status"""
        try:
            # Publish current angle
            angle_msg = Float32()
            angle_msg.data = self.pulses_to_angle(self.current_pulses)
            self.angle_publisher.publish(angle_msg)

            # Publish at-target status
            status_msg = Bool()
            status_msg.data = self.at_target
            self.status_publisher.publish(status_msg)

        except Exception as e:
            self.log(
                'error', f'Failed to publish status for servo {self.id}: {str(e)}')

    def _target_angle_callback(self, msg: Float32) -> None:
        """Handle target angle messages"""
        try:
            self.rotate(msg.data)
        except Exception as e:
            self.log(
                'error', f'Failed to handle target angle for servo {self.id}: {str(e)}')
