#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from typing import Dict, List, Optional
import sys
import re
from .protocol import Servo42CProtocol
from .simulated_servo import SimulatedServo
from .servo import Servo
from rcl_interfaces.msg import ParameterDescriptor
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import math # Import math for pi


# Limits and safety
MIN_ANGLE = -360.0  # degrees
MAX_ANGLE = 360.0   # degrees
MAX_SERVOS = 3
UPDATE_RATE = 0.1  # seconds

# DEFAULT_SIM_RPM = 30.0 # Default RPM for simulation if no velocity command -- REMOVE
DEFAULT_SIM_RAD_PER_SEC = 0.5 # Default rad/s for simulation if no velocity command (approx 5 RPM)

class ServoControllerNode(Node):
    def __init__(self):
        super().__init__('servo42c_controller')

        # Add simulation parameter
        self.declare_parameter('simulation', False)
        self.simulation_mode = self.get_parameter('simulation').value

        # Parameters
        self.declare_parameter('device', 'ttyUSB0')
        self.declare_parameter('baud_rate', 38400)
        self.declare_parameter('position_tolerance', 0.5)  # degrees
        self.declare_parameter('update_rate', UPDATE_RATE)
        # Modbus motion parameters
        self.declare_parameter('modbus_acceleration', 0xF0)
        self.declare_parameter('modbus_speed', 0xFF)
        self.declare_parameter('modbus_timeout', 1.0)
        # Number of servos expected (indices start at 0)
        self.declare_parameter('servo_count', 2)

        # Servo names and per-servo parameters
        for i in range(MAX_SERVOS):
            self.declare_parameter(
                f'servo.{i}.name',
                f'joint{i}',
                ParameterDescriptor(
                    description=f'Name for servo {i} in joint_states message'
                )
            )
            self.declare_parameter(
                f'servo.{i}.min_angle',
                MIN_ANGLE,
                ParameterDescriptor(
                    description=f'Minimum angle (degrees) for servo {i}'
                )
            )
            self.declare_parameter(
                f'servo.{i}.max_angle',
                MAX_ANGLE,
                ParameterDescriptor(
                    description=f'Maximum angle (degrees) for servo {i}'
                )
            )

        # Initialize protocol
        device = self.get_parameter('device').value
        baud_rate = self.get_parameter('baud_rate').value
        modbus_timeout = float(self.get_parameter('modbus_timeout').value)
        self.protocol = Servo42CProtocol(
            device, baud_rate, logger=self.get_logger(), timeout=modbus_timeout)

        if not self.simulation_mode and not self.protocol.connect():
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

        # Joint trajectory subscriber for per-move accel/speed overrides (device units)
        self.trajectory_sub = self.create_subscription(
            JointTrajectory,
            '/servo/trajectory',
            self._trajectory_callback,
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

        # Last-known state caches to ensure consistent publishing
        self._last_known_position: Dict[int, float] = {}
        self._last_known_velocity: Dict[int, float] = {}

        # Enumerate and initialize servos based on config
        expected_count = int(self.get_parameter('servo_count').value)
        initialized = 0
        for index in range(expected_count):
            # Default Modbus ID is index + 1
            self.declare_parameter(f'servo.{index}.modbus_id', index + 1)
            modbus_id = int(self.get_parameter(f'servo.{index}.modbus_id').value)
            try:
                servo = self._initialize_servo(modbus_id)
                if servo:
                    self.servos.append(servo)
                    self.servo_map[servo.name] = servo
                    self.get_logger().info(
                        f'Found servo with Modbus ID {modbus_id} and name {servo.name}')
                    initialized += 1
                else:
                    continue
            except Exception as e:
                self.get_logger().error(
                    f'Unable to initialize servo with Modbus ID {modbus_id}: {str(e)}')

        # Create timer for updating servo states
        update_rate = self.get_parameter('update_rate').value
        self.create_timer(update_rate, self.update_servo_states)

        if initialized != expected_count:
            raise RuntimeError(f'Expected {expected_count} servos, but initialized {initialized}. Failing startup.')
        self.get_logger().info(f'Servo controller node initialized with {len(self.servos)} servos')

    def _initialize_servo(self, servo_id: int) -> Optional[Servo]:
        """Initialize either real or simulated servo"""
        if self.simulation_mode:
            servo = SimulatedServo(
                self,
                protocol=self.protocol,
                name=self.get_parameter(f'servo.{servo_id}.name').value,
                servo_id=servo_id,
                logger=self.get_logger(),
                min_angle_deg=self.get_parameter(
                    f'servo.{servo_id}.min_angle').value,
                max_angle_deg=self.get_parameter(
                    f'servo.{servo_id}.max_angle').value,
                position_tolerance_deg=self.get_parameter(
                    'position_tolerance').value,
                microstep_factor=8
            )
        else:
            servo = Servo(
                self,
                protocol=self.protocol,
                name=self.get_parameter(f'servo.{servo_id}.name').value,
                servo_id=servo_id,
                logger=self.get_logger(),
                min_angle=self.get_parameter(
                    f'servo.{servo_id}.min_angle').value,
                max_angle=self.get_parameter(
                    f'servo.{servo_id}.max_angle').value,
                position_tolerance=self.get_parameter(
                    'position_tolerance').value,
                microstep_factor=8,
                modbus_acceleration=self.get_parameter('modbus_acceleration').value,
                modbus_speed=self.get_parameter('modbus_speed').value
            )

        if servo.initialize():
            return servo
        return None

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
        """Handle joint command messages for all servos with per-move overrides.

        positions: radians target
        velocity: device speed override (0..65535) if provided; also used as rad/s for simulation
        effort: device acceleration override (0..65535) if provided
        """
        try:
            # Process each joint in the command message
            for i, joint_name in enumerate(msg.name):
                if joint_name in self.servo_map:
                    servo = self.servo_map[joint_name]
                    try:
                        target_position = msg.position[i]
                        # Simulation rad/s default
                        target_velocity_rad_s = DEFAULT_SIM_RAD_PER_SEC

                        # Per-move device overrides
                        speed_override = None
                        accel_override = None

                        if i < len(msg.velocity):
                            try:
                                # Use velocity both for simulation rad/s and as device speed override
                                target_velocity_rad_s = float(msg.velocity[i]) if isinstance(msg.velocity[i], (float, int)) else DEFAULT_SIM_RAD_PER_SEC
                                speed_override = int(max(0, min(65535, round(msg.velocity[i]))))
                            except Exception:
                                pass

                        if i < len(msg.effort):
                            try:
                                accel_override = int(max(0, min(65535, round(msg.effort[i]))))
                            except Exception:
                                pass

                        if not self.is_e_stopped:
                            servo.rotate(target_position, target_velocity_rad_s, accel_override=accel_override, speed_override=speed_override)
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
                current_velocity = 0.0
                try:
                    # If in simulation mode, get velocity BEFORE updating the angle
                    if self.simulation_mode and hasattr(servo, 'get_velocity_rad_per_sec'):
                        current_velocity = float(servo.get_velocity_rad_per_sec())
                except Exception:
                    # Keep previous velocity if available
                    current_velocity = self._last_known_velocity.get(servo.id, 0.0)

                # Attempt to read position; fallback to last-known on failure
                try:
                    current_position = float(servo.get_angle())
                    self._last_known_position[servo.id] = current_position
                except Exception as e:
                    self.get_logger().warn(f'Using last-known position for servo {servo.id} due to read error: {str(e)}')
                    current_position = self._last_known_position.get(servo.id, 0.0)

                # Cache velocity as well
                self._last_known_velocity[servo.id] = current_velocity

                msg.name.append(servo.name)
                msg.position.append(current_position)
                msg.velocity.append(current_velocity)
                msg.effort.append(0.0)

            self.joint_state_pub.publish(msg)
        except Exception as e:
            self.get_logger().error(f'Failed to publish joint states: {str(e)}')

    def _trajectory_callback(self, msg: JointTrajectory) -> None:
        """Handle JointTrajectory for per-move accel/speed overrides.

        Interprets point.velocities and point.accelerations as raw device units
        for speed/acceleration for each joint. Uses only the first point.
        """
        try:
            if not msg.points:
                self.get_logger().warn('Received JointTrajectory with no points')
                return

            point = msg.points[0]
            names = msg.joint_names

            for i, joint_name in enumerate(names):
                if joint_name not in self.servo_map:
                    self.get_logger().warn(f'Trajectory for unknown joint: {joint_name}')
                    continue

                servo = self.servo_map[joint_name]

                # Positions are in radians as per ROS convention
                if i >= len(point.positions):
                    self.get_logger().warn(f'No position for joint {joint_name} in trajectory point')
                    continue

                target_angle = float(point.positions[i])

                # Optional overrides in raw device units
                speed_override = None
                accel_override = None
                if i < len(point.velocities):
                    try:
                        speed_override = int(max(0, min(65535, round(point.velocities[i]))))
                    except Exception:
                        speed_override = None
                if i < len(point.accelerations):
                    try:
                        accel_override = int(max(0, min(65535, round(point.accelerations[i]))))
                    except Exception:
                        accel_override = None

                # For simulation behavior, pass a rad/s; if not provided, use default
                rad_per_sec = DEFAULT_SIM_RAD_PER_SEC
                if i < len(point.velocities) and isinstance(point.velocities[i], float):
                    # If velocities are floats, they might be rad/s; but overrides above already captured device units.
                    # Keep simulation motion responsive; do not alter device overrides.
                    rad_per_sec = abs(point.velocities[i])

                if not self.is_e_stopped:
                    servo.rotate(target_angle, rad_per_sec, accel_override=accel_override, speed_override=speed_override)
        except Exception as e:
            self.get_logger().error(f'Failed to handle JointTrajectory: {str(e)}')

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
