#!/usr/bin/env python3
"""Main ROS2 node for servo42c controller."""

import rclpy
from rclpy.node import Node
import sys
from .servo_protocol import ServoProtocol
from .servo_config import ServoConfig
from .servo_manager import ServoManager
from .publisher_manager import PublisherManager
from .async_utils import AsyncRunner
from . import constants

# TODO initialize pubsub immediately after servo is found
# TODO add stop command
# TODO add feedback for when servo is at target
# TODO add speed control 

# Constants for commands
GET_PULSES = 0x33
GET_SERIAL_ENABLED = 0xf3
ROTATE = 0xfd
STOP = 0xf7

# Motor configuration
STEPS_PER_REV = 200  # Base steps per revolution
MICROSTEP_FACTOR = 8  # Microstepping factor
GEAR_RATIO = 37      # Gear reduction ratio
PULSES_PER_ROTATION = STEPS_PER_REV * MICROSTEP_FACTOR * GEAR_RATIO

# Limits and safety
MIN_ANGLE = -360.0  # degrees
MAX_ANGLE = 360.0   # degrees
MIN_SPEED = 1
MAX_SPEED = 120
SERIAL_TIMEOUT = 1.0  # seconds
MAX_RECONNECT_ATTEMPTS = 3

class ServoControllerNode(Node):
    """ROS2 node for controlling Servo42C motors."""
    
    def __init__(self):
        super().__init__('servo42c_controller')
        
        # Initialize parameters
        self._init_parameters()
        
        # Initialize async runner
        self.async_runner = AsyncRunner()
        
        # Initialize servo communication
        self.protocol = ServoProtocol(
            device=self.get_parameter('device').value,
            baud_rate=self.get_parameter('baud_rate').value
        )
        
        # Initialize servo configuration
        self.config = ServoConfig(
            min_angle=self.get_parameter('min_angle').value,
            max_angle=self.get_parameter('max_angle').value,
            position_tolerance=self.get_parameter('position_tolerance').value
        )
        
        # Initialize managers
        self.servo_manager = ServoManager(self.protocol, self.config)
        self.publisher_manager = PublisherManager(self)
        
        # Initialize servos
        self._init_servos()
        
        # Create status publishing timer
        self.create_timer(constants.STATUS_UPDATE_RATE, self.publish_status)
        
        self.get_logger().info('Servo controller node initialized')
    
    def _init_parameters(self):
        """Initialize node parameters."""
        self.declare_parameter('device', 'ttyUSB0')
        self.declare_parameter('baud_rate', 9600)
        self.declare_parameter('min_angle', constants.MIN_ANGLE)
        self.declare_parameter('max_angle', constants.MAX_ANGLE)
        self.declare_parameter('position_tolerance', constants.POSITION_TOLERANCE)
    
    def _init_servos(self):
        """Initialize servo communication."""
        active_servos = self.async_runner.run_coroutine(self.servo_manager.discover_servos())
        
        for servo_id in active_servos:
            self.publisher_manager.setup_servo_communication(
                servo_id,
                self._handle_target_angle
            )
    
    def _handle_target_angle(self, servo_id: int, angle: float):
        """Handle new target angle command."""
        self.async_runner.run_coroutine(
            self.servo_manager.move_to_angle(servo_id, angle)
        )
    
    def publish_status(self):
        """Publish current servo angles and status."""
        current_angles = self.async_runner.run_coroutine(
            self.servo_manager.update_servo_states()
        )
        
        for servo_id, angle in current_angles.items():
            self.publisher_manager.publish_angle(servo_id, angle)
            self.publisher_manager.publish_status(
                servo_id,
                self.servo_manager.servo_statuses[servo_id]
            )
    
    def cleanup(self):
        """Clean up resources before shutdown."""
        try:
            # Stop all servos
            self.async_runner.run_coroutine(self.servo_manager.emergency_stop())
            
            # Clean up managers
            self.publisher_manager.cleanup()
            self.async_runner.cleanup()
            
            # Clean up protocol
            self.protocol.cleanup()
            
            self.get_logger().info('Cleanup completed successfully')
        except Exception as e:
            print(f'Error during cleanup: {str(e)}', file=sys.stderr)

def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)
    node = None
    
    def handle_shutdown(reason='Shutdown'):
        """Handle shutdown with timeout and forced exit if needed."""
        nonlocal node
        if not node:
            return
            
        print(f'{reason} received, cleaning up...', file=sys.stderr)
        try:
            import threading
            cleanup_thread = threading.Thread(target=lambda: [
                node.cleanup(),
                node.destroy_node()
            ])
            cleanup_thread.daemon = True
            cleanup_thread.start()
            cleanup_thread.join(timeout=2.0)
            
            if cleanup_thread.is_alive():
                print('Cleanup taking too long, forcing exit...', file=sys.stderr)
        except Exception as e:
            print(f'Error during cleanup: {str(e)}', file=sys.stderr)
        finally:
            try:
                rclpy.shutdown()
            except Exception as e:
                print(f'Error during ROS shutdown: {str(e)}', file=sys.stderr)
            import os
            os._exit(0)
    
    try:
        node = ServoControllerNode()
        
        # Set up signal handlers
        import signal
        signal.signal(signal.SIGINT, lambda sig, frame: handle_shutdown('SIGINT'))
        signal.signal(signal.SIGTERM, lambda sig, frame: handle_shutdown('SIGTERM'))
        
        rclpy.spin(node)
    except KeyboardInterrupt:
        handle_shutdown('KeyboardInterrupt')
    except Exception as e:
        print(f'Unexpected error: {str(e)}', file=sys.stderr)
        handle_shutdown('Error')

if __name__ == '__main__':
    main()