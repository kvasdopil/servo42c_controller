"""ROS2 publisher and subscriber management module."""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool
from typing import Dict, Callable
from . import constants

class PublisherManager:
    """Manages ROS2 publishers and subscribers for servo communication."""
    
    def __init__(self, node: Node):
        self.node = node
        self.angle_publishers: Dict[int, rclpy.publisher.Publisher] = {}
        self.status_publishers: Dict[int, rclpy.publisher.Publisher] = {}
        self.target_subscribers: Dict[int, rclpy.subscription.Subscription] = {}
    
    def setup_servo_communication(self, servo_id: int, target_callback: Callable[[int, float], None]) -> None:
        """Set up publishers and subscribers for a servo."""
        # Create publishers
        self.angle_publishers[servo_id] = self.node.create_publisher(
            Float32,
            f'servo42c/servo_{servo_id}/angle',
            constants.PUBLISHER_QOS
        )
        
        self.status_publishers[servo_id] = self.node.create_publisher(
            Bool,
            f'servo42c/servo_{servo_id}/at_target',
            constants.PUBLISHER_QOS
        )
        
        # Create target subscriber
        self.target_subscribers[servo_id] = self.node.create_subscription(
            Float32,
            f'servo42c/servo_{servo_id}/target',
            lambda msg: target_callback(servo_id, msg.data),
            constants.PUBLISHER_QOS
        )
    
    def publish_angle(self, servo_id: int, angle: float) -> None:
        """Publish current angle for a servo."""
        if servo_id in self.angle_publishers:
            msg = Float32()
            msg.data = angle
            self.angle_publishers[servo_id].publish(msg)
    
    def publish_status(self, servo_id: int, at_target: bool) -> None:
        """Publish at-target status for a servo."""
        if servo_id in self.status_publishers:
            msg = Bool()
            msg.data = at_target
            self.status_publishers[servo_id].publish(msg)
    
    def cleanup(self) -> None:
        """Clean up publishers and subscribers."""
        for publisher in self.angle_publishers.values():
            publisher.destroy()
        for publisher in self.status_publishers.values():
            publisher.destroy()
        for subscriber in self.target_subscribers.values():
            self.node.destroy_subscription(subscriber) 