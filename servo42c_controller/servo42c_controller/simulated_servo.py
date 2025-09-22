from rclpy.node import Node
# from math import pi - No longer needed directly here
import math # Import math for pi calculation and radians/degrees conversion
# Remove degree-based constants import, use radians
# from .servo import MIN_ANGLE, MAX_ANGLE, STEPS_PER_REV, MICROSTEP_FACTOR, GEAR_RATIO
from .servo import STEPS_PER_REV, MICROSTEP_FACTOR, GEAR_RATIO

# Define defaults in radians
MIN_ANGLE_RAD = math.radians(-360.0)
MAX_ANGLE_RAD = math.radians(360.0)
DEFAULT_POS_TOLERANCE_DEG = 0.5 # Keep default tolerance param in degrees for readability

class SimulatedServo:
    """Simulated version of the Servo class (using radians internally)"""

    def __init__(self,
                 node: Node,
                 protocol,
                 servo_id: int,
                 logger,
                 min_angle_deg: float = math.degrees(MIN_ANGLE_RAD), # Expect degrees from params
                 max_angle_deg: float = math.degrees(MAX_ANGLE_RAD), # Expect degrees from params
                 position_tolerance_deg: float = DEFAULT_POS_TOLERANCE_DEG,
                 # Remove max/min speed params if not used
                 # max_speed: int = 128,
                 # min_speed: int = 1,
                 steps_per_rev: int = STEPS_PER_REV,
                 microstep_factor: int = MICROSTEP_FACTOR,
                 gear_ratio: int = GEAR_RATIO,
                 name: str = None):
        self.id = servo_id
        self.logger = logger
        # Store limits and tolerance in radians
        self.min_angle_rad = math.radians(min_angle_deg)
        self.max_angle_rad = math.radians(max_angle_deg)
        self.position_tolerance_rad = math.radians(position_tolerance_deg)
        self.target_angle_rad = 0.0 # Initialize in radians
        self.current_angle_rad = 0.0 # Initialize in radians
        self.is_enabled = True
        self.name = name
        self.node = node
        self.last_update_time = node.get_clock().now()
        self.current_speed_rad_s = 0.0  # Store speed magnitude in rad/s

    def initialize(self) -> bool:
        """Simulated initialization always succeeds"""
        # Optionally set initial angle if needed, e.g., based on a parameter
        # self.current_angle_rad = self._get_initial_angle_from_param_or_sensor()
        self.target_angle_rad = self.current_angle_rad # Start with target = current
        self.logger.info(f'Simulated servo {self.id} initialized at {self.current_angle_rad:.4f} rad')
        return True

    def rotate(self, angle_rad: float, rad_per_sec: float, accel_override: int = None, speed_override: int = None) -> bool:
        """Simulate rotation to target angle (radians) at specified rad/s.

        accel_override and speed_override are accepted for API parity and ignored.
        """
        # Check bounds using radians
        if not self.min_angle_rad <= angle_rad <= self.max_angle_rad:
            self.logger.error(
                f'Angle {angle_rad:.4f} rad out of bounds ({self.min_angle_rad:.4f} to {self.max_angle_rad:.4f} rad) for servo {self.id}')
            return False

        self.target_angle_rad = angle_rad
        # Store the magnitude of the speed
        self.current_speed_rad_s = abs(rad_per_sec)
        self.logger.debug(f'Servo {self.id} rotate command: Target={angle_rad:.4f} rad, Speed={self.current_speed_rad_s:.4f} rad/s')
        return True

    def stop(self) -> None:
        """Simulate stop command by setting target to current"""
        self.target_angle_rad = self.current_angle_rad
        self.current_speed_rad_s = 0.0
        self.logger.info(f'Simulated stop for servo {self.id}')

    def cleanup(self) -> None:
        """Simulated cleanup"""
        pass

    def get_angle(self) -> float:
        """Get current angle (radians) with simulated movement"""
        now = self.node.get_clock().now()
        dt_duration = now - self.last_update_time
        dt = dt_duration.nanoseconds / 1e9

        if dt <= 0: # Avoid division by zero or negative time steps
             return self.current_angle_rad

        # Calculate movement based on time elapsed and current speed (already in rad/s)
        angle_change = self.current_speed_rad_s * dt

        if self.current_angle_rad < self.target_angle_rad:
            remaining_angle = self.target_angle_rad - self.current_angle_rad
            move = min(angle_change, remaining_angle)
            self.current_angle_rad += move
        elif self.current_angle_rad > self.target_angle_rad:
            remaining_angle = self.current_angle_rad - self.target_angle_rad
            move = min(angle_change, remaining_angle)
            self.current_angle_rad -= move

        self.last_update_time = now
        # self.logger.debug(f'Servo {self.id} get_angle: dt={dt:.4f}, change={angle_change:.4f}, current={self.current_angle_rad:.4f}')
        return self.current_angle_rad

    def get_velocity_rad_per_sec(self) -> float:
        """Get current simulated velocity in radians per second."""
        # Check if close enough to target to consider stopped
        # Perform comparison in radians using half the configured tolerance
        tolerance = self.position_tolerance_rad / 2.0

        # Compare angles in radians
        if abs(self.current_angle_rad - self.target_angle_rad) < tolerance:
            # If within tolerance, velocity is zero
            return 0.0
        else:
            # Otherwise, velocity is the stored magnitude with the correct sign
            if self.target_angle_rad > self.current_angle_rad:
                return self.current_speed_rad_s
            else: # target_angle_rad < self.current_angle_rad
                return -self.current_speed_rad_s
