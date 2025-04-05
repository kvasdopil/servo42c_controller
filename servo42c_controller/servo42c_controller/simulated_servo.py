from rclpy.node import Node
from math import pi
from .servo import MIN_ANGLE, MAX_ANGLE, STEPS_PER_REV, MICROSTEP_FACTOR, GEAR_RATIO


class SimulatedServo:
    """Simulated version of the Servo class"""

    def __init__(self,
                 node: Node,
                 protocol,
                 servo_id: int,
                 logger,
                 min_angle: float = MIN_ANGLE,
                 max_angle: float = MAX_ANGLE,
                 position_tolerance: float = 0.5,
                 max_speed: int = 128,
                 min_speed: int = 1,
                 steps_per_rev: int = STEPS_PER_REV,
                 microstep_factor: int = MICROSTEP_FACTOR,
                 gear_ratio: int = GEAR_RATIO,
                 name: str = None):
        self.id = servo_id
        self.logger = logger
        self.min_angle = min_angle
        self.max_angle = max_angle
        self.position_tolerance = position_tolerance
        self.target_angle = 0.0
        self.current_angle = 0.0
        self.is_enabled = True
        self.name = name
        self.node = node
        self.last_update_time = node.get_clock().now()
        self.current_speed_rpm = 0.0  # Store speed in RPM

    def initialize(self) -> bool:
        """Simulated initialization always succeeds"""
        self.logger.info(f'Simulated servo {self.id} initialized')
        return True

    def rotate(self, angle: float, rpm: float) -> bool:
        """Simulate rotation to target angle at specified RPM"""
        if not self.min_angle <= angle <= self.max_angle:
            self.logger.error(
                f'Angle {angle} out of bounds for servo {self.id}')
            return False

        self.target_angle = angle
        self.current_speed_rpm = rpm
        return True

    def stop(self) -> None:
        """Simulate stop command"""
        self.logger.info(f'Simulated stop for servo {self.id}')

    def cleanup(self) -> None:
        """Simulated cleanup"""
        pass

    def get_angle(self) -> float:
        """Get current angle with simulated movement"""
        now = self.node.get_clock().now()
        dt = (now - self.last_update_time).nanoseconds / 1e9

        # Calculate movement based on time elapsed and current speed
        degrees_per_second = self.current_speed_rpm * 360.0 / 60.0
        angle_change = degrees_per_second * dt

        if self.current_angle < self.target_angle:
            self.current_angle += min(
                angle_change,
                self.target_angle - self.current_angle
            )
        elif self.current_angle > self.target_angle:
            self.current_angle -= min(
                angle_change,
                self.current_angle - self.target_angle
            )

        self.last_update_time = now
        return self.current_angle
