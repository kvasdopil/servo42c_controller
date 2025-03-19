from rclpy.node import Node
from std_msgs.msg import Float32, Bool

from .protocol import Servo42CProtocol

# Motor configuration
STEPS_PER_REV = 200  # Base steps per revolution
MICROSTEP_FACTOR = 8  # Microstepping factor
GEAR_RATIO = 37      # Gear reduction ratio
PULSES_PER_ROTATION = STEPS_PER_REV * MICROSTEP_FACTOR * GEAR_RATIO

MAX_SPEED = 128
MIN_SPEED = 1


class Servo:
    """Class representing a single servo motor"""

    def __init__(self, node: Node, protocol: Servo42CProtocol, servo_id: int):
        """Initialize servo instance"""
        self.node = node
        self.protocol = protocol
        self.id = servo_id
        self.current_pulses = 0
        self.target_pulses = 0
        self.at_target = True

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

    @staticmethod
    def angle_to_pulses(angle: float) -> int:
        """Convert angle in degrees to pulses"""
        return round(angle * PULSES_PER_ROTATION / 360)

    @staticmethod
    def pulses_to_angle(pulses: int) -> float:
        """Convert pulses to angle in degrees"""
        return float(pulses) * 360.0 / PULSES_PER_ROTATION

    def _validate_angle(self, angle: float) -> bool:
        """Validate if angle is within allowed range"""
        min_angle = self.node.get_parameter('min_angle').value
        max_angle = self.node.get_parameter('max_angle').value
        return min_angle <= angle <= max_angle

    def _validate_speed(self, speed: int) -> bool:
        """Validate if speed is within allowed range"""
        return MIN_SPEED <= speed <= MAX_SPEED

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
            self.node.get_logger().error(
                f'Failed to initialize servo {self.id}: {str(e)}')
            return False

    def rotate(self, angle: float, speed: int = 120) -> bool:
        """Rotate servo to specified angle"""
        if not self._validate_angle(angle):
            self.node.get_logger().error(
                f'Angle {angle} out of bounds for servo {self.id}')
            return False

        if not self._validate_speed(speed):
            self.node.get_logger().error(
                f'Speed {speed} out of bounds for servo {self.id}')
            return False

        new_target_pulses = self.angle_to_pulses(angle)
        diff = new_target_pulses - self.current_pulses

        # If no movement is needed, return success immediately
        if diff == 0:
            return True

        # Update status before starting movement
        self.at_target = False
        self.publish_status()

        if self.protocol.rotate(self.id, speed, diff):
            self.target_pulses = new_target_pulses
            self.node.get_logger().info(
                f'Moving servo {self.id} to angle: {angle}')
            return True

        # Reset status on failure
        self.at_target = True
        self.publish_status()
        return False

    def stop(self) -> None:
        """Stop servo movement"""
        try:
            self.protocol.stop(self.id)
            self.node.get_logger().info(
                f'Emergency stop sent to servo {self.id}')
        except Exception as e:
            self.node.get_logger().error(
                f'Failed to stop servo {self.id}: {str(e)}')

    def update_position(self) -> None:
        """Update current position and check if target reached"""
        try:
            self.current_pulses = self.protocol.get_pulses(self.id)
            current_angle = self.pulses_to_angle(self.current_pulses)

            # Check if at target
            tolerance = self.node.get_parameter('position_tolerance').value
            at_target = abs(
                current_angle - self.pulses_to_angle(self.target_pulses)) <= tolerance

            # Publish updates if status changed
            if at_target != self.at_target:
                self.at_target = at_target
                if at_target:
                    self.node.get_logger().info(
                        f'Servo {self.id} reached target position')

            self.publish_status()

        except Exception as e:
            self.node.get_logger().error(
                f'Failed to update position for servo {self.id}: {str(e)}')

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
            self.node.get_logger().error(
                f'Failed to publish status for servo {self.id}: {str(e)}')

    def _target_angle_callback(self, msg: Float32) -> None:
        """Handle target angle messages"""
        try:
            self.rotate(msg.data)
        except Exception as e:
            self.node.get_logger().error(
                f'Failed to handle target angle for servo {self.id}: {str(e)}')
