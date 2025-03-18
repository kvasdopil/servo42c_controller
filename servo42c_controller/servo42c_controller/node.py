#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import serial
from typing import List, Optional, Dict

# TODO initialize pubsub immediately after servo is found
# TODO add stop command
# TODO add feedback for when servo is at target
# TODO add speed control 

GET_PULSES = 0x33
GET_SERIAL_ENABLED = 0xf3
ROTATE = 0xfd
STOP = 0xf7
PULSES_PER_ROTATION = 200 * 8 * 37

class ServoControllerNode(Node):
    def __init__(self):
        super().__init__('servo42c_controller')
        
        # Parameters
        self.declare_parameter('device', 'ttyUSB0')
        self.declare_parameter('baud_rate', 9600)
    
        # Serial communication
        self.serial_port: Optional[serial.Serial] = None
        
        # Store publishers and active servos
        self.angle_publishers: Dict[int, rclpy.publisher.Publisher] = {}
        self.active_servos: List[int] = []
        self.target_positions: Dict[int, int] = {}
        
        # Initialize serial port
        self._setup_serial()
        
        # Enumerate servos
        self.enumerate_servos()
        
        # Create publishers and subscribers for each active servo
        for servo_id in self.active_servos:
            # Create angle publisher
            self.angle_publishers[servo_id] = self.create_publisher(
                Float32,
                f'servo42c/servo_{servo_id}/angle',
                10
            )
            
            # Create target angle subscriber with proper async callback wrapper
            def make_callback(id: int):
                async def callback(msg: Float32):
                    await self.target_angle_callback(msg, id)
                return callback
            
            self.create_subscription(
                Float32,
                f'servo42c/servo_{servo_id}/target',
                make_callback(servo_id),
                10
            )
        
        # Timer for publishing current angles
        self.create_timer(0.1, self.publish_angles)  # 10Hz
        
        self.get_logger().info(f'Servo controller node initialized with {len(self.active_servos)} servos')
    
    def _setup_serial(self):
        """Initialize serial port communication"""
        device = self.get_parameter('device').value
        baud_rate = self.get_parameter('baud_rate').value
        
        try:
            self.serial_port = serial.Serial(
                f'/dev/{device}',
                baud_rate,
                timeout=1,
                write_timeout=1
            )
            self.get_logger().info(f'Connected to /dev/{device}')
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to open serial port: {str(e)}')
            raise
    
    async def write(self, data: bytes) -> None:
        """Write data to serial port"""
        if not self.serial_port:
            self.get_logger().error('Serial port not open')
            return
        
        try:
            self.serial_port.write(data)
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to write to serial port: {str(e)}')
            raise

    async def read(self, id: int, num_bytes: int) -> List[int]:
        """Read specified number of bytes from serial port"""
        if not self.serial_port:
            raise RuntimeError('Serial port not open')
            
        try:
            data = self.serial_port.read(num_bytes + 1)  # +1 for id byte
            if len(data) != num_bytes + 1:
                raise RuntimeError('Timeout reading from serial port')
                
            if data[0] != 0xe0 + id:
                raise RuntimeError('Invalid ID received')
                
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

    async def rotate(self, id: int, speed: int, angle: float) -> bool:
        """Rotate servo to specified position"""

        new_target_pulses = self.angle_to_pulses(angle)
        prev_target_pulses = self.target_positions.get(id, 0)
        diff = new_target_pulses - prev_target_pulses

        # If no movement is needed, return success immediately
        if diff == 0:
            return True

        # Calculate pulses preserving sign
        abs_diff = abs(diff)
        # Set direction bit based on original position value
        sign_bit = 0b10000000 if diff > 0 else 0
        
        ok = False  # Initialize ok variable
        while abs_diff > 0:
            p = min(abs_diff, 0xffff)
            abs_diff -= p
            
            [ok] = await self.send(id, [
                ROTATE,
                sign_bit + speed,
                (p >> 8) & 0xff,
                p & 0xff
            ], 1)

            if ok != 1:  # If any rotation command fails, return False
                return False

        # store target position
        self.target_positions[id] = new_target_pulses
            
        return True  # All rotation commands succeeded

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

    async def target_angle_callback(self, msg: Float32, servo_id: int) -> None:
        """Handle new target angle commands for specific servo"""
        try:
            await self.rotate(servo_id, 120, msg.data)
            self.get_logger().info(f'Moving servo {servo_id} to angle: {msg.data}')
        except Exception as e:
            self.get_logger().error(f'Failed to rotate servo {servo_id}: {str(e)}')
    
    async def publish_angles(self) -> None:
        """Publish current angles for all active servos"""
        for servo_id in self.active_servos:
            try:
                angle = await self.get_angle(servo_id)
                msg = Float32()
                msg.data = float(angle)
                self.angle_publishers[servo_id].publish(msg)
            except Exception as e:
                self.get_logger().error(f'Failed to publish angle for servo {servo_id}: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = ServoControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.serial_port:
            node.serial_port.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()