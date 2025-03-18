"""Servo discovery and management module."""

import rclpy
from typing import Dict, List, Optional
from .servo_protocol import ServoProtocol
from .servo_config import ServoConfig
from .async_utils import with_error_logging
from . import constants

class ServoManager:
    """Manages servo discovery and state tracking."""
    
    def __init__(self, protocol: ServoProtocol, config: ServoConfig):
        self.protocol = protocol
        self.config = config
        self.logger = rclpy.logging.get_logger('servo_manager')
        
        self.active_servos: List[int] = []
        self.target_positions: Dict[int, int] = {}
        self.servo_statuses: Dict[int, bool] = {}
    
    @with_error_logging('servo_manager')
    async def discover_servos(self) -> List[int]:
        """Find all available servos and initialize them."""
        for servo_id in range(10):  # Check IDs 0-9
            try:
                if await self.protocol.get_serial_enabled(servo_id):
                    initial_pulses = await self.protocol.get_pulses(servo_id)
                    self.active_servos.append(servo_id)
                    self.target_positions[servo_id] = initial_pulses
                    self.servo_statuses[servo_id] = True
                    self.logger.info(f'Found servo with ID {servo_id} at position {initial_pulses} pulses')
            except Exception as e:
                self.logger.debug(f'No servo found at ID {servo_id}: {str(e)}')
        
        return self.active_servos
    
    @with_error_logging('servo_manager')
    async def move_to_angle(self, servo_id: int, angle: float) -> bool:
        """Move servo to specified angle."""
        if servo_id not in self.active_servos:
            raise ValueError(f'Invalid servo ID {servo_id}')
            
        if not self.config.validate_angle(angle):
            raise ValueError(f'Angle {angle} out of bounds')
            
        # Update status before movement
        self.servo_statuses[servo_id] = False
        
        # Calculate target pulses and current position
        target_pulses = self.config.angle_to_pulses(angle)
        current_pulses = await self.protocol.get_pulses(servo_id)
        pulses_diff = target_pulses - current_pulses
        
        if pulses_diff == 0:
            self.servo_statuses[servo_id] = True
            return True
            
        success = await self.protocol.rotate(servo_id, self.config.max_speed, pulses_diff)
        if success:
            self.target_positions[servo_id] = target_pulses
            self.logger.info(f'Moving servo {servo_id} to angle: {angle}')
        else:
            self.logger.error(f'Failed to initiate movement for servo {servo_id}')
            self.servo_statuses[servo_id] = True
            
        return success
    
    @with_error_logging('servo_manager')
    async def emergency_stop(self, servo_id: Optional[int] = None) -> None:
        """Emergency stop one or all servos."""
        servos_to_stop = [servo_id] if servo_id is not None else self.active_servos
        
        for id in servos_to_stop:
            if id not in self.active_servos:
                self.logger.error(f'Invalid servo ID {id}')
                continue
                
            try:
                await self.protocol.stop(id)
                self.logger.info(f'Emergency stop sent to servo {id}')
            except Exception as e:
                self.logger.error(f'Failed to stop servo {id}: {str(e)}')
    
    @with_error_logging('servo_manager')
    async def update_servo_states(self) -> Dict[int, float]:
        """Update and return current angles of all active servos."""
        current_angles = {}
        
        for servo_id in self.active_servos:
            try:
                current_pulses = await self.protocol.get_pulses(servo_id)
                current_angle = self.config.pulses_to_angle(current_pulses)
                current_angles[servo_id] = current_angle
                
                if servo_id in self.target_positions:
                    target_angle = self.config.pulses_to_angle(self.target_positions[servo_id])
                    at_target = self.config.is_at_target(current_angle, target_angle)
                    
                    if at_target != self.servo_statuses[servo_id]:
                        self.servo_statuses[servo_id] = at_target
                        if at_target:
                            self.logger.info(f'Servo {servo_id} reached target position')
                            
            except Exception as e:
                self.logger.error(f'Failed to update state for servo {servo_id}: {str(e)}')
                
        return current_angles 