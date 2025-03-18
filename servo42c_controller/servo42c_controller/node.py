#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool
import asyncio
import threading
from functools import partial
import sys
from typing import Dict, List, Optional
from .servo_protocol import ServoProtocol
from .servo_config import ServoConfig

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
    def __init__(self):
        super().__init__('servo42c_controller')
        
        # Create event loop for async operations
        self.loop = asyncio.new_event_loop()
        self.loop_thread = threading.Thread(target=self._run_event_loop, daemon=True)
        self.loop_thread.start()
        
        # Initialize parameters
        self._init_parameters()
        
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
        
        # Store publishers and servo state
        self.angle_publishers: Dict[int, rclpy.publisher.Publisher] = {}
        self.status_publishers: Dict[int, rclpy.publisher.Publisher] = {}
        self.active_servos: List[int] = []
        self.target_positions: Dict[int, int] = {}
        self.servo_statuses: Dict[int, bool] = {}
        
        # Initialize servos
        self._init_servos()
        
        # Create status publishing timer
        self.create_timer(0.1, self.publish_status)  # 10Hz
        
        self.get_logger().info(f'Servo controller node initialized with {len(self.active_servos)} servos')
    
    def _init_parameters(self):
        """Initialize node parameters"""
        self.declare_parameter('device', 'ttyUSB0')
        self.declare_parameter('baud_rate', 9600)
        self.declare_parameter('min_angle', -360.0)
        self.declare_parameter('max_angle', 360.0)
        self.declare_parameter('position_tolerance', 0.5)
    
    def _init_servos(self):
        """Initialize servo communication and get initial positions"""
        for servo_id in range(10):  # Check IDs 0-9
            try:
                if self._run_coroutine(self.protocol.get_serial_enabled(servo_id)):
                    self.active_servos.append(servo_id)
                    initial_pulses = self._run_coroutine(self.protocol.get_pulses(servo_id))
                    self.target_positions[servo_id] = initial_pulses
                    self._setup_servo_communication(servo_id)
                    self.servo_statuses[servo_id] = True
                    self.get_logger().info(f'Found servo with ID {servo_id} at position {initial_pulses} pulses')
            except Exception as e:
                self.get_logger().debug(f'No servo found at ID {servo_id}: {str(e)}')
    
    def _setup_servo_communication(self, servo_id: int):
        """Set up publishers and subscribers for a servo"""
        # Create publishers
        self.angle_publishers[servo_id] = self.create_publisher(
            Float32,
            f'servo42c/servo_{servo_id}/angle',
            10
        )
        self.status_publishers[servo_id] = self.create_publisher(
            Bool,
            f'servo42c/servo_{servo_id}/at_target',
            10
        )
        
        # Create target subscriber
        self.create_subscription(
            Float32,
            f'servo42c/servo_{servo_id}/target',
            partial(self._target_angle_callback, servo_id),
            10
        )
    
    def _target_angle_callback(self, servo_id: int, msg: Float32):
        """Handle new target angle command"""
        self._run_coroutine(self._handle_target_angle(servo_id, msg.data))
    
    async def _handle_target_angle(self, servo_id: int, angle: float):
        """Process new target angle asynchronously"""
        try:
            # Validate angle
            if not self.config.validate_angle(angle):
                self.get_logger().error(f'Angle {angle} out of bounds for servo {servo_id}')
                return
            
            # Update status before movement
            self._update_servo_status(servo_id, False)
            
            # Calculate target pulses and current position
            target_pulses = self.config.angle_to_pulses(angle)
            current_pulses = await self.protocol.get_pulses(servo_id)
            pulses_diff = target_pulses - current_pulses
            
            if pulses_diff == 0:
                self._update_servo_status(servo_id, True)
                return
            
            # Attempt rotation
            success = await self.protocol.rotate(servo_id, self.config.max_speed, pulses_diff)
            if success:
                self.target_positions[servo_id] = target_pulses
                self.get_logger().info(f'Moving servo {servo_id} to angle: {angle}')
            else:
                self.get_logger().error(f'Failed to initiate movement for servo {servo_id}')
                self._update_servo_status(servo_id, True)
                
        except Exception as e:
            self.get_logger().error(f'Failed to rotate servo {servo_id}: {str(e)}')
            self._update_servo_status(servo_id, True)
    
    def _update_servo_status(self, servo_id: int, at_target: bool):
        """Update and publish servo status"""
        self.servo_statuses[servo_id] = at_target
        status_msg = Bool()
        status_msg.data = at_target
        self.status_publishers[servo_id].publish(status_msg)
        
        if at_target:
            self.get_logger().info(f'Servo {servo_id} reached target position')
    
    def publish_status(self):
        """Publish current servo angles and status"""
        self._run_coroutine(self._publish_status_async())
    
    async def _publish_status_async(self):
        """Async implementation of status publishing"""
        for servo_id in self.active_servos:
            try:
                # Get current angle
                current_pulses = await self.protocol.get_pulses(servo_id)
                current_angle = self.config.pulses_to_angle(current_pulses)
                
                # Publish angle
                angle_msg = Float32()
                angle_msg.data = current_angle
                self.angle_publishers[servo_id].publish(angle_msg)
                
                # Check and publish at-target status
                if servo_id in self.target_positions:
                    target_angle = self.config.pulses_to_angle(self.target_positions[servo_id])
                    at_target = self.config.is_at_target(current_angle, target_angle)
                    
                    if at_target != self.servo_statuses[servo_id]:
                        self._update_servo_status(servo_id, at_target)
                
            except Exception as e:
                self.get_logger().error(f'Failed to publish status for servo {servo_id}: {str(e)}')
    
    async def emergency_stop(self, servo_id: Optional[int] = None):
        """Emergency stop one or all servos"""
        servos_to_stop = [servo_id] if servo_id is not None else self.active_servos
        
        for id in servos_to_stop:
            if id not in self.active_servos:
                self.get_logger().error(f'Invalid servo ID {id}')
                continue
                
            try:
                await self.protocol.stop(id)
                self.get_logger().info(f'Emergency stop sent to servo {id}')
            except Exception as e:
                self.get_logger().error(f'Failed to stop servo {id}: {str(e)}')
    
    def _run_event_loop(self):
        """Run the event loop in a separate thread"""
        asyncio.set_event_loop(self.loop)
        self.loop.run_forever()
    
    def _run_coroutine(self, coro):
        """Helper to run coroutines in the event loop"""
        future = asyncio.run_coroutine_threadsafe(coro, self.loop)
        try:
            return future.result()
        except Exception as e:
            self.get_logger().error(f'Coroutine execution failed: {str(e)}')
            return None
    
    def cleanup(self):
        """Clean up resources before shutdown"""
        try:
            # Stop all servos first
            if hasattr(self, 'protocol'):
                for servo_id in self.active_servos:
                    try:
                        # Use direct protocol call to avoid event loop issues during shutdown
                        command_bytes = bytes([0xe0 + servo_id, self.protocol.STOP])
                        if self.protocol.serial_port and self.protocol.serial_port.is_open:
                            self.protocol.serial_port.write(command_bytes)
                            self.protocol.serial_port.flush()
                    except Exception:
                        pass  # Ignore errors during emergency stop
            
            # Stop the event loop
            if hasattr(self, 'loop') and self.loop.is_running():
                self.loop.call_soon_threadsafe(self.loop.stop)
                try:
                    self.loop_thread.join(timeout=1.0)
                except Exception:
                    pass  # Ignore join errors
            
            # Clean up protocol
            if hasattr(self, 'protocol'):
                self.protocol.cleanup()
            
            self.get_logger().info('Cleanup completed successfully')
        except Exception as e:
            print(f'Error during cleanup: {str(e)}', file=sys.stderr)
    
    def __del__(self):
        """Destructor to ensure cleanup"""
        try:
            self.cleanup()
        except Exception:
            pass

def main(args=None):
    rclpy.init(args=args)
    node = None
    
    def handle_shutdown(reason='Shutdown'):
        """Handle shutdown with timeout and forced exit if needed"""
        nonlocal node
        if not node:
            return
            
        print(f'{reason} received, cleaning up...', file=sys.stderr)
        try:
            # Set timeout for cleanup operations
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
            # Ensure the process exits
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