#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import time

class TrajectoryTester(Node):
    def __init__(self):
        super().__init__('trajectory_tester')
        self._action_client = ActionClient(
            self,
            FollowJointTrajectory,
            'arm_trajectory_controller/follow_joint_trajectory'
        )

    def send_trajectory(self):
        # Wait for action server
        self.get_logger().info('Waiting for action server...')
        self._action_client.wait_for_server()
        
        # Create goal
        goal = FollowJointTrajectory.Goal()
        
        # Set joint names - adjust these to match your servo names
        goal.trajectory.joint_names = ['joint0', 'joint1', 'joint2']
        
        # Create a sequence of points
        points = []
        
        # Point 1: Move to initial position
        point1 = JointTrajectoryPoint()
        point1.positions = [0.0, 0.0, 0.0]  # radians
        point1.velocities = [0.0, 0.0, 0.0]
        point1.time_from_start = Duration(sec=2, nanosec=0)
        points.append(point1)
        
        # Point 2: Move to new position
        point2 = JointTrajectoryPoint()
        point2.positions = [1.0, -0.1, -0.1]  # radians
        point2.velocities = [0.0, 0.0, 0.0]
        point2.time_from_start = Duration(sec=5, nanosec=0)
        points.append(point2)
        
        # Point 3: Return to initial position
        point3 = JointTrajectoryPoint()
        point3.positions = [0.0, 0.0, 0.0]  # radians
        point3.velocities = [0.0, 0.0, 0.0]
        point3.time_from_start = Duration(sec=7, nanosec=0)
        points.append(point3)
        
        goal.trajectory.points = points
        
        # Send goal
        self.get_logger().info('Sending trajectory...')
        future = self._action_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected!')
            return
            
        self.get_logger().info('Goal accepted, waiting for result...')
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        
        self.get_logger().info('Trajectory completed!')

def main(args=None):
    rclpy.init(args=args)
    tester = TrajectoryTester()
    
    try:
        tester.send_trajectory()
    except KeyboardInterrupt:
        pass
    finally:
        tester.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 