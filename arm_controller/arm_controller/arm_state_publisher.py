#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64

class ArmStatePublisher(Node):
    def __init__(self):
        super().__init__('arm_state_publisher')
        
        # Create publishers for joint states
        self.joint_state_publisher = self.create_publisher(
            JointState,
            'joint_states',
            10)
            
        # Create subscribers for servo commands
        self.joint1_subscriber = self.create_subscription(
            Float64,
            'joint1_position',
            self.joint1_callback,
            10)
            
        self.joint2_subscriber = self.create_subscription(
            Float64,
            'joint2_position',
            self.joint2_callback,
            10)
            
        # Initialize joint states
        self.joint_state = JointState()
        self.joint_state.name = ['joint1', 'joint2']
        self.joint_state.position = [0.0, 0.0]
        
        # Create timer for publishing joint states
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        self.get_logger().info('Arm State Publisher has been started')

    def joint1_callback(self, msg):
        self.joint_state.position[0] = msg.data
        
    def joint2_callback(self, msg):
        self.joint_state.position[1] = msg.data
        
    def timer_callback(self):
        self.joint_state.header.stamp = self.get_clock().now().to_msg()
        self.joint_state_publisher.publish(self.joint_state)

def main(args=None):
    rclpy.init(args=args)
    node = ArmStatePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 