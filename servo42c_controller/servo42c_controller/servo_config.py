from dataclasses import dataclass
from typing import Optional

@dataclass
class ServoConfig:
    # Motor configuration
    STEPS_PER_REV: int = 200  # Base steps per revolution
    MICROSTEP_FACTOR: int = 8  # Microstepping factor
    GEAR_RATIO: int = 37       # Gear reduction ratio
    
    # Limits and safety
    min_angle: float = -360.0  # degrees
    max_angle: float = 360.0   # degrees
    min_speed: int = 1
    max_speed: int = 120
    position_tolerance: float = 0.5  # degrees
    
    @property
    def pulses_per_rotation(self) -> int:
        return self.STEPS_PER_REV * self.MICROSTEP_FACTOR * self.GEAR_RATIO
    
    def angle_to_pulses(self, angle: float) -> int:
        """Convert angle in degrees to motor pulses"""
        return round(angle * self.pulses_per_rotation / 360)
    
    def pulses_to_angle(self, pulses: float) -> float:
        """Convert motor pulses to angle in degrees"""
        return float(pulses) * 360.0 / self.pulses_per_rotation
    
    def validate_angle(self, angle: float) -> bool:
        """Validate if angle is within allowed range"""
        return self.min_angle <= angle <= self.max_angle
    
    def validate_speed(self, speed: int) -> bool:
        """Validate if speed is within allowed range"""
        return self.min_speed <= speed <= self.max_speed
    
    def is_at_target(self, current_angle: float, target_angle: float) -> bool:
        """Check if current angle is within tolerance of target"""
        return abs(current_angle - target_angle) <= self.position_tolerance 