#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from typing import Dict, List, Optional
import sys
import re
from .protocol import Servo42CProtocol
from .servo import Servo
from rcl_interfaces.msg import ParameterDescriptor


# Limits and safety
MIN_ANGLE = -360.0  # degrees
MAX_ANGLE = 360.0   # degrees
MAX_SERVOS = 16
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
        
        # Servo names
        for i in range(MAX_SERVOS):
            self.declare_parameter(
                f'servo.{i}.name', 
                f'joint{i}',
                ParameterDescriptor(
                    description=f'Name for servo {i} in joint_states message'
                )
            )

        # Initialize protocol
        device = self.get_parameter('device').value
        baud_rate = self.get_parameter('baud_rate').value
        self.protocol = Servo42CProtocol(
            device, baud_rate, logger=self.get_logger())
        if not self.protocol.connect():
            raise RuntimeError('Failed to initialize servo controller')

        # Store active servos
        self.servos: List[Servo] = []

        # Enumerate and initialize servos
        for servo_id in range(MAX_SERVOS):  # Cycle through all servos
            try:
                servo = Servo(self,
                              protocol=self.protocol,
                              name=self.get_parameter(f'servo.{servo_id}.name').value,
                              servo_id=servo_id,
                              logger=self.get_logger(),
                              min_angle=self.get_parameter('min_angle').value,
                              max_angle=self.get_parameter('max_angle').value,
                              position_tolerance=self.get_parameter('position_tolerance').value,
                              microstep_factor=8)
                if servo.initialize():
                    self.servos.append(servo)
                    self.get_logger().info(
                        f'Found servo with ID {servo_id} and name {servo.name} at position {servo.target_pulses} pulses')
                else:
                    continue
            except Exception as e:
                self.get_logger().error(
                    f'Unable to initialize servo {servo_id}: {str(e)}')

        # Create timer for updating servo states
        update_rate = self.get_parameter('update_rate').value
        self.create_timer(update_rate, self.update_servo_states)

        self.get_logger().info(
            f'Servo controller node initialized with {len(self.servos)} servos')

    def update_servo_states(self) -> None:
        """Update and publish states for all servos"""
        for servo in self.servos:
            servo.publish_state()

    def cleanup(self):
        """Clean up resources before shutdown"""
        if hasattr(self, 'servos'):
            # Stop all servos
            for servo in self.servos:
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
            pass


def main(args=None):
    rclpy.init(args=args)
    node = None
    
    try:
        node = ServoControllerNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error: {str(e)}', file=sys.stderr)
    finally:
        try:
            if node is not None:
                node.destroy_node()
                node.cleanup()
        except Exception as e:
            print(f'Error during shutdown: {str(e)}', file=sys.stderr)
        finally:
            try:
                rclpy.try_shutdown()
            except Exception:
                pass


if __name__ == '__main__':
    main()
