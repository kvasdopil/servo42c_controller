#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from typing import Dict
import sys
import re
from .protocol import Servo42CProtocol
from .servo import Servo
from sensor_msgs.msg import JointState
from typing import List, Optional
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult

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

        # Add parameter callback
        self.add_on_set_parameters_callback(self._on_parameter_change)

        # Initialize protocol
        device = self.get_parameter('device').value
        baud_rate = self.get_parameter('baud_rate').value
        self.protocol = Servo42CProtocol(
            device, baud_rate, logger=self.get_logger())
        if not self.protocol.connect():
            raise RuntimeError('Failed to initialize servo controller')

        # Store active servos
        self.servos: List[Servo] = []

        # Create JointState publisher
        self.joint_state_pub = self.create_publisher(JointState, 'joint_states', 10)

        # Initialize JointState message with fixed fields
        self.joint_state_msg = JointState()

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

        # Pre-allocate arrays for namem position, velocity, and effort
        num_joints = len(self.servos)
        self.joint_state_msg.name = [servo.name for servo in self.servos]
        self.joint_state_msg.position = [0.0] * num_joints
        self.joint_state_msg.velocity = [0.0] * num_joints
        self.joint_state_msg.effort = [0.0] * num_joints

        # Timer for updating servo positions and status
        update_rate = self.get_parameter('update_rate').value
        self.create_timer(update_rate, self.update_servos)

        self.get_logger().info(
            f'Servo controller node initialized with {len(self.servos)} servos')

    def _on_parameter_change(self, params) -> SetParametersResult:
        """Handle parameter changes during runtime."""
        servo_name_pattern = re.compile(r'^servo.(\d+).name$')
        
        for param in params:
            # Check if this is a servo name parameter
            match = servo_name_pattern.match(param.name)
            if match:
                try:
                    servo_id = int(match.group(1))

                    # find servo by id
                    servo = self.servos.find(lambda x: x.id == servo_id)
                    if servo is None:
                        return SetParametersResult(successful=False, reason=f'Invalid servo ID in parameter: {param.name}')
                    
                    # check for duplcate names
                    if any(s.name == param.value for s in self.servos):
                        return SetParametersResult(successful=False, reason=f'Name already exists: {param.value}')

                    servo.name = param.value # will update the joint_state_msg.name on next update_servos call

                except ValueError:
                    return SetParametersResult(successful=False, reason=f'Invalid servo ID in parameter: {param.name}')
           
        return SetParametersResult(successful=True)


    def update_servos(self) -> None:
        """Update all servo positions and status and publish joint states"""
        # Update timestamp
        self.joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        
        # Update positions using pre-computed indices
        idx = 0
        for servo in self.servos:
            self.joint_state_msg.name[idx] = servo.name
            self.joint_state_msg.position[idx] = servo.get_angle()
            idx += 1
        
        # Publish the message
        self.joint_state_pub.publish(self.joint_state_msg)

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
    except Exception as e:
        print(f'Error: {str(e)}', file=sys.stderr)
    finally:
        try:
            if node is not None:
                # Stop publishing and receiving messages
                node.destroy_node()
                # Now it's safe to do cleanup that might log
                node.cleanup()
        except Exception as e:
            print(f'Error during shutdown: {str(e)}', file=sys.stderr)
        finally:
            try:
                rclpy.try_shutdown()
            except Exception:
                pass  # Ignore shutdown errors


if __name__ == '__main__':
    main()
