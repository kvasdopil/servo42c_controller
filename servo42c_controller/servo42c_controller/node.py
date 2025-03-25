#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from typing import Dict, List, Optional
import sys
import re
from .protocol import Servo42CProtocol
from .servo import Servo
from rcl_interfaces.msg import ParameterDescriptor
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool


# Limits and safety
MIN_ANGLE = -360.0  # degrees
MAX_ANGLE = 360.0   # degrees
MAX_SERVOS = 4
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
        # Map joint names to servo objects for quick lookup
        self.servo_map: Dict[str, Servo] = {}

        # Create publishers and subscribers for topic-based control
        self.joint_state_pub = self.create_publisher(
            JointState,
            '/servo/states',
            10
        )

        self.joint_command_sub = self.create_subscription(
            JointState,
            '/servo/commands',
            self._joint_command_callback,
            10
        )

        # Emergency stop subscriber
        self.e_stop_sub = self.create_subscription(
            Bool,
            '/e_stop',
            self._emergency_stop_callback,
            1  # Higher priority QoS
        )
        self.is_e_stopped = False

        # Enumerate and initialize servos
        for servo_id in range(MAX_SERVOS):  # Cycle through all servos
            try:
                servo = Servo(self,
                              protocol=self.protocol,
                              name=self.get_parameter(
                                  f'servo.{servo_id}.name').value,
                              servo_id=servo_id,
                              logger=self.get_logger(),
                              min_angle=self.get_parameter('min_angle').value,
                              max_angle=self.get_parameter('max_angle').value,
                              position_tolerance=self.get_parameter(
                                  'position_tolerance').value,
                              microstep_factor=8)
                if servo.initialize():
                    self.servos.append(servo)
                    self.servo_map[servo.name] = servo
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

    def _emergency_stop_callback(self, msg: Bool) -> None:
        """Handle emergency stop messages"""
        try:
            stop = msg.data
            self.get_logger().warn(f"Emergency stop: {stop}")

            if stop:
                # Stop all servos immediately
                for servo in self.servos:
                    try:
                        servo.stop()
                    except Exception as e:
                        self.get_logger().error(
                            f'Failed to emergency stop servo {servo.id}: {str(e)}')
            else:
                # E-stop released
                self.get_logger().info('Emergency stop released')

            self.is_e_stopped = stop
        except Exception as e:
            self.get_logger().error(
                f'Failed to handle emergency stop message: {str(e)}')

    def _joint_command_callback(self, msg: JointState) -> None:
        """Handle joint command messages for all servos"""
        try:
            # Process each joint in the command message
            for i, joint_name in enumerate(msg.name):
                if joint_name in self.servo_map:
                    servo = self.servo_map[joint_name]
                    try:
                        target_position = msg.position[i]
                        if not self.is_e_stopped:
                            servo.rotate(target_position, servo.current_speed)
                    except Exception as e:
                        self.get_logger().error(
                            f'Failed to handle command for servo {servo.id}: {str(e)}')
                else:
                    self.get_logger().warn(
                        f'Received command for unknown joint: {joint_name}')
        except Exception as e:
            self.get_logger().error(
                f'Failed to handle joint command message: {str(e)}')

    def update_servo_states(self) -> None:
        """Update and publish states for all servos"""
        try:
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = []
            msg.position = []
            msg.velocity = []
            msg.effort = []

            for servo in self.servos:
                try:
                    current_position = servo.get_angle()
                    msg.name.append(servo.name)
                    msg.position.append(current_position)
                    msg.velocity.append(0.0)  # We don't have velocity feedback
                    msg.effort.append(0.0)    # We don't have effort feedback
                except Exception as e:
                    self.get_logger().error(
                        f'Failed to get state for servo {servo.id}: {str(e)}')

            self.joint_state_pub.publish(msg)
        except Exception as e:
            self.get_logger().error(
                f'Failed to publish joint states: {str(e)}')

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
