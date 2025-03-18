#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool
import serial
from typing import List, Optional, Dict
import asyncio
import threading
from functools import partial
import sys

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
        
        # Parameters
        self.declare_parameter('device', 'ttyUSB0')
        self.declare_parameter('baud_rate', 9600)
        self.declare_parameter('min_angle', MIN_ANGLE)
        self.declare_parameter('max_angle', MAX_ANGLE)
        self.declare_parameter('position_tolerance', 0.5)  # degrees
    
        # Serial communication
        self.serial_port: Optional[serial.Serial] = None
        self.serial_lock = asyncio.Lock()
        
        # Store publishers and active servos
        self.angle_publishers: Dict[int, rclpy.publisher.Publisher] = {}
        self.status_publishers: Dict[int, rclpy.publisher.Publisher] = {}
        self.active_servos: List[int] = []
        self.target_positions: Dict[int, int] = {}
        self.servo_statuses: Dict[int, bool] = {}  # Track if servos are at target
        
        # Initialize serial port
        self._setup_serial()
        
        # Enumerate servos
        self.enumerate_servos()
        
        # Create publishers and subscribers for each active servo
        for servo_id in self.active_servos:
            self._setup_servo_communication(servo_id)
            self.servo_statuses[servo_id] = True  # Initialize as at target
        
        # Timer for publishing current angles and status
        self.create_timer(0.1, self.publish_status)  # 10Hz
        
        self.get_logger().info(f'Servo controller node initialized with {len(self.active_servos)} servos')
    
    def _setup_serial(self):
        """Initialize serial port communication with reconnection logic"""
        device = self.get_parameter('device').value
        baud_rate = self.get_parameter('baud_rate').value
        
        for attempt in range(MAX_RECONNECT_ATTEMPTS):
            try:
                if self.serial_port is not None:
                    self.serial_port.close()
                    
                self.serial_port = serial.Serial(
                    f'/dev/{device}',
                    baud_rate,
                    timeout=SERIAL_TIMEOUT,
                    write_timeout=SERIAL_TIMEOUT
                )
                self.get_logger().info(f'Connected to /dev/{device}')
                return
            except serial.SerialException as e:
                self.get_logger().error(f'Attempt {attempt + 1}/{MAX_RECONNECT_ATTEMPTS} failed: {str(e)}')
                if attempt < MAX_RECONNECT_ATTEMPTS - 1:
                    self.get_logger().info('Retrying in 1 second...')
                    rclpy.sleep(1.0)
                else:
                    self.get_logger().error('Failed to establish serial connection')
                    raise
    
    async def _ensure_serial_connection(self):
        """Ensure serial connection is active, attempt reconnection if needed"""
        if self.serial_port is None or not self.serial_port.is_open:
            self.get_logger().warn('Serial connection lost, attempting to reconnect...')
            try:
                self._setup_serial()
            except Exception as e:
                self.get_logger().error(f'Failed to reconnect: {str(e)}')
                return False
        return True

    async def write(self, data: bytes) -> None:
        """Write data to serial port with connection check"""
        async with self.serial_lock:
            if not await self._ensure_serial_connection():
                raise RuntimeError('Serial port not available')
            
            try:
                self.serial_port.write(data)
                self.serial_port.flush()  # Use flush instead of drain
            except serial.SerialException as e:
                self.get_logger().error(f'Failed to write to serial port: {str(e)}')
                raise

    async def read(self, id: int, num_bytes: int) -> List[int]:
        """Read specified number of bytes from serial port with connection check"""
        async with self.serial_lock:
            if not await self._ensure_serial_connection():
                raise RuntimeError('Serial port not available')
                
            try:
                data = self.serial_port.read(num_bytes + 1)  # +1 for id byte
                if len(data) != num_bytes + 1:
                    raise RuntimeError('Timeout reading from serial port')
                    
                if data[0] != 0xe0 + id:
                    raise RuntimeError(f'Invalid ID received: expected {0xe0 + id}, got {data[0]}')
                    
                return list(data[1:])
            except serial.SerialException as e:
                self.get_logger().error(f'Failed to read from serial port: {str(e)}')
                raise

    async def send(self, id: int, msg: List[int], return_length: int = 1) -> List[int]:
        """Send command and read response"""
        await self.write(bytes([0xe0 + id] + msg))
        response = await self.read(id, return_length)
        return response
    
    angle_to_pulses = lambda self, angle: round(angle * PULSES_PER_ROTATION / 360)
    pulses_to_angle = lambda self, pulses: float(pulses) * 360.0 / PULSES_PER_ROTATION

    def _validate_angle(self, angle: float) -> bool:
        """Validate if angle is within allowed range"""
        min_angle = self.get_parameter('min_angle').value
        max_angle = self.get_parameter('max_angle').value
        return min_angle <= angle <= max_angle

    def _validate_speed(self, speed: int) -> bool:
        """Validate if speed is within allowed range"""
        return MIN_SPEED <= speed <= MAX_SPEED

    async def rotate(self, id: int, speed: int, angle: float) -> bool:
        """Rotate servo to specified position"""
        if not self._validate_angle(angle):
            self.get_logger().error(f'Angle {angle} out of bounds for servo {id}')
            return False
            
        if not self._validate_speed(speed):
            self.get_logger().error(f'Speed {speed} out of bounds for servo {id}')
            return False

        new_target_pulses = self.angle_to_pulses(angle)
        prev_target_pulses = self.target_positions.get(id, 0)
        diff = new_target_pulses - prev_target_pulses

        # If no movement is needed, return success immediately
        if diff == 0:
            return True

        try:
            async with asyncio.timeout(SERIAL_TIMEOUT):
                # Calculate pulses preserving sign
                abs_diff = abs(diff)
                sign_bit = 0b10000000 if diff > 0 else 0
                
                ok = False
                while abs_diff > 0:
                    p = min(abs_diff, 0xffff)
                    abs_diff -= p
                    
                    [ok] = await self.send(id, [
                        ROTATE,
                        sign_bit + speed,
                        (p >> 8) & 0xff,
                        p & 0xff
                    ], 1)

                    if ok != 1:
                        return False

                # store target position
                self.target_positions[id] = new_target_pulses
                return True

        except asyncio.TimeoutError:
            self.get_logger().error(f'Rotation command timed out for servo {id}')
            return False
        except Exception as e:
            self.get_logger().error(f'Failed to rotate servo {id}: {str(e)}')
            return False

    async def get_pulses(self, id: int) -> int:
        """Get current pulse count"""
        [a, b, c, d] = await self.send(id, [GET_PULSES], 4)
        # Combine bytes into a signed 32-bit integer
        value = (a << 24) + (b << 16) + (c << 8) + d
        # Handle two's complement for negative values
        if value & 0x80000000:  # If highest bit is set (negative)
            value = -((~value & 0xFFFFFFFF) + 1)
        return value

    async def get_angle(self, id: int) -> float:
        """Get current angle"""
        pulses = await self.get_pulses(id)
        return self.pulses_to_angle(pulses)

    def enumerate_servos(self) -> None:
        """Detect active servos by checking serial enabled status and get initial positions"""
        for servo_id in range(10):  # Check IDs 0-9
            try:
                if self.get_serial_enabled_sync(servo_id):
                    self.active_servos.append(servo_id)
                    # Get initial position synchronously during enumeration
                    initial_pulses = self.get_pulses_sync(servo_id)
                    self.target_positions[servo_id] = initial_pulses
                    self.get_logger().info(f'Found servo with ID {servo_id} at position {initial_pulses} pulses')
            except Exception as e:
                self.get_logger().debug(f'No servo found at ID {servo_id}: {str(e)}')

    def get_pulses_sync(self, id: int) -> int:
        """Synchronous version of get_pulses for initialization"""
        if not self.serial_port:
            raise RuntimeError('Serial port not open')
            
        self.serial_port.write(bytes([0xe0 + id, GET_PULSES]))
        data = self.serial_port.read(5)  # Read ID byte and 4 bytes of position data
        
        if len(data) != 5:
            raise RuntimeError('Timeout reading from serial port')
            
        if data[0] != 0xe0 + id:
            raise RuntimeError('Invalid ID received')
            
        # Combine bytes into a signed 32-bit integer
        value = (data[1] << 24) + (data[2] << 16) + (data[3] << 8) + data[4]
        # Handle two's complement for negative values
        if value & 0x80000000:  # If highest bit is set (negative)
            value = -((~value & 0xFFFFFFFF) + 1)
        return value

    def get_serial_enabled_sync(self, id: int) -> bool:
        """Synchronous version of get_serial_enabled for initialization"""
        if not self.serial_port:
            raise RuntimeError('Serial port not open')
            
        self.serial_port.write(bytes([0xe0 + id, GET_SERIAL_ENABLED, 1]))
        data = self.serial_port.read(2)  # Read ID byte and status
        
        if len(data) != 2:
            raise RuntimeError('Timeout reading from serial port')
            
        if data[0] != 0xe0 + id:
            raise RuntimeError('Invalid ID received')
            
        return data[1] == 1

    def _setup_servo_communication(self, servo_id: int) -> None:
        """Set up publishers and subscribers for a servo"""
        # Angle publisher
        self.angle_publishers[servo_id] = self.create_publisher(
            Float32,
            f'servo42c/servo_{servo_id}/angle',
            10
        )
        
        # Status publisher
        self.status_publishers[servo_id] = self.create_publisher(
            Bool,
            f'servo42c/servo_{servo_id}/at_target',
            10
        )
        
        # Target subscriber with synchronous callback
        self.create_subscription(
            Float32,
            f'servo42c/servo_{servo_id}/target',
            partial(self._target_angle_callback_sync, servo_id),
            10
        )

    def _is_at_target(self, servo_id: int, current_angle: float) -> bool:
        """Check if servo is at target position within tolerance"""
        if servo_id not in self.target_positions:
            return True
            
        target_angle = self.pulses_to_angle(self.target_positions[servo_id])
        tolerance = self.get_parameter('position_tolerance').value
        return abs(current_angle - target_angle) <= tolerance

    def publish_status(self) -> None:
        """Synchronous wrapper for async publish_status"""
        self._run_coroutine(self._publish_status_async())

    async def _publish_status_async(self) -> None:
        """Async implementation of status publishing"""
        for servo_id in self.active_servos:
            try:
                # Get current angle
                angle = await self.get_angle(servo_id)
                
                # Publish angle
                angle_msg = Float32()
                angle_msg.data = float(angle)
                self.angle_publishers[servo_id].publish(angle_msg)
                
                # Check and publish at-target status
                at_target = self._is_at_target(servo_id, angle)
                if at_target != self.servo_statuses[servo_id]:
                    self.servo_statuses[servo_id] = at_target
                    status_msg = Bool()
                    status_msg.data = at_target
                    self.status_publishers[servo_id].publish(status_msg)
                    
                    if at_target:
                        self.get_logger().info(f'Servo {servo_id} reached target position')
                
            except Exception as e:
                self.get_logger().error(f'Failed to publish status for servo {servo_id}: {str(e)}')

    def _target_angle_callback_sync(self, servo_id: int, msg: Float32) -> None:
        """Synchronous wrapper for async target_angle_callback"""
        self._run_coroutine(self._target_angle_callback_async(msg, servo_id))

    async def _target_angle_callback_async(self, msg: Float32, servo_id: int) -> None:
        """Async implementation of target angle callback"""
        try:
            # Update status before starting movement
            self.servo_statuses[servo_id] = False
            status_msg = Bool()
            status_msg.data = False
            self.status_publishers[servo_id].publish(status_msg)
            
            # Attempt rotation
            success = await self.rotate(servo_id, 120, msg.data)
            if success:
                self.get_logger().info(f'Moving servo {servo_id} to angle: {msg.data}')
            else:
                self.get_logger().error(f'Failed to initiate movement for servo {servo_id}')
                # Reset status on failure
                self.servo_statuses[servo_id] = True
                status_msg.data = True
                self.status_publishers[servo_id].publish(status_msg)
                
        except Exception as e:
            self.get_logger().error(f'Failed to rotate servo {servo_id}: {str(e)}')

    async def emergency_stop(self, servo_id: Optional[int] = None) -> None:
        """Emergency stop one or all servos"""
        if servo_id is not None:
            if servo_id not in self.active_servos:
                self.get_logger().error(f'Invalid servo ID {servo_id}')
                return
            servos_to_stop = [servo_id]
        else:
            servos_to_stop = self.active_servos

        for id in servos_to_stop:
            try:
                await self.send(id, [STOP], 1)
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
        if hasattr(self, 'loop') and self.loop.is_running():
            self.loop.call_soon_threadsafe(self.loop.stop)
            self.loop_thread.join(timeout=1.0)
            
        if hasattr(self, 'serial_port') and self.serial_port and self.serial_port.is_open:
            try:
                # Try to stop all servos
                for servo_id in self.active_servos:
                    try:
                        self.serial_port.write(bytes([0xe0 + servo_id, STOP]))
                        self.serial_port.flush()
                    except Exception:
                        # Ignore errors during emergency stop
                        pass
                self.serial_port.close()
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
        
        def signal_handler(sig, frame):
            """Handle shutdown signals"""
            nonlocal node
            if node:
                node.get_logger().info('Shutdown signal received, cleaning up...')
                node.cleanup()
        
        import signal
        signal.signal(signal.SIGINT, signal_handler)
        signal.signal(signal.SIGTERM, signal_handler)
        
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node:
            try:
                node.cleanup()
                node.destroy_node()
            except Exception as e:
                print(f'Error during node cleanup: {str(e)}', file=sys.stderr)
        try:
            rclpy.shutdown()
        except Exception as e:
            print(f'Error during ROS shutdown: {str(e)}', file=sys.stderr)

if __name__ == '__main__':
    main()