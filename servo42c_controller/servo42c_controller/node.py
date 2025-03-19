#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from typing import Dict
import sys
from .protocol import Servo42CProtocol
from .servo import Servo

# TODO add stop command
# TODO add feedback for when servo is at target
# TODO add speed control

# Limits and safety
MIN_ANGLE = -360.0  # degrees
MAX_ANGLE = 360.0   # degrees

UPDATE_RATE = 0.1  # seconds


class ServoControllerNode(Node):
    def __init__(self):
        super().__init__('servo42c_controller')

        # Parameters
        self.declare_parameter('device', 'ttyUSB0')
        self.declare_parameter('baud_rate', 9600)
        self.declare_parameter('min_angle', MIN_ANGLE)
        self.declare_parameter('max_angle', MAX_ANGLE)
        self.declare_parameter('position_tolerance', 0.5)  # degrees
        self.declare_parameter('update_rate', UPDATE_RATE)

        # Initialize protocol
        device = self.get_parameter('device').value
        baud_rate = self.get_parameter('baud_rate').value
        self.protocol = Servo42CProtocol(
            device, baud_rate, logger=self.get_logger())
        if not self.protocol.connect():
            raise RuntimeError('Failed to initialize servo controller')

        # Store active servos
        self.servos: Dict[int, Servo] = {}

        # Enumerate and initialize servos
        for servo_id in range(10):  # Check IDs 0-9
            try:
                servo = Servo(self,
                              protocol=self.protocol,
                              servo_id=servo_id,
                              logger=self.get_logger(),
                              min_angle=self.get_parameter('min_angle').value,
                              max_angle=self.get_parameter('max_angle').value,
                              position_tolerance=self.get_parameter('position_tolerance').value)
                if servo.initialize():
                    self.servos[servo_id] = servo
                    self.get_logger().info(
                        f'Found servo with ID {servo_id} at position {servo.current_pulses} pulses')
            except Exception as e:
                self.get_logger().debug(
                    f'No servo found at ID {servo_id}: {str(e)}')

        # Timer for updating servo positions and status
        update_rate = self.get_parameter('update_rate').value
        self.create_timer(update_rate, self.update_servos)

        self.get_logger().info(
            f'Servo controller node initialized with {len(self.servos)} servos')

    def update_servos(self) -> None:
        """Update all servo positions and status"""
        for servo in self.servos.values():
            servo.update_position()

    def cleanup(self):
        """Clean up resources before shutdown"""
        if hasattr(self, 'servos'):
            # Stop all servos
            for servo in self.servos.values():
                try:
                    servo.stop()
                except Exception:
                    # Ignore errors during emergency stop
                    pass

        if hasattr(self, 'protocol'):
            try:
                self.protocol.close()
            except Exception as e:
                self.get_logger().error(f'Error during cleanup: {str(e)}')

    def __del__(self):
        """Destructor to ensure cleanup"""
        try:
            self.cleanup()
        except Exception:
            # Ignore errors during destructor
            pass


def main(args=None):
    rclpy.init(args=args)
    node = None
    
    try:
        node = ServoControllerNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.get_logger().info('Shutting down...')
            node.cleanup()  # This will stop servos and close serial port
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
