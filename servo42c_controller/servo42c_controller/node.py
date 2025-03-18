#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import serial
import time
from typing import List, Optional

class ServoControllerNode(Node):
    def __init__(self):
        super().__init__('servo_controller')
        
        # Parameters
        self.declare_parameter('device', 'ttyUSB0')
        self.declare_parameter('baud_rate', 9600)
        
        # Constants from your original code
        self.GET_PULSES = 0x33
        self.GET_SERIAL_ENABLED = 0xf3
        self.ROTATE = 0xfd
        self.STOP = 0xf7
        self.PULSES_PER_ROTATION = 200 * 8 * 37
        
        # Serial communication
        self.serial_port: Optional[serial.Serial] = None
        self.msg_queue = []
        self.buffer = []
        self.locked = False
        
        # Initialize serial port
        self._setup_serial()
        
        # Publishers
        self.angle_pub = self.create_publisher(
            Float32, 
            'servo/current_angle', 
            10
        )
        
        # Subscribers
        self.target_angle_sub = self.create_subscription(
            Float32,
            'servo/target_angle',
            self.target_angle_callback,
            10
        )
        
        # Timer for publishing current angle
        self.create_timer(0.1, self.publish_angle)  # 10Hz
        
        self.get_logger().info('Servo controller node initialized')
    
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
            self.get_logger().debug(f'Sending: {" ".join([f"{x:02x}" for x in data])}')
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
        while self.locked:
            await self.sleep(0.1)
        self.locked = True
        
        try:
            await self.write(bytes([0xe0 + id] + msg))
            response = await self.read(id, return_length)
            return response
        finally:
            self.locked = False

    async def rotate(self, id: int, speed: int, position: float) -> bool:
        """Rotate servo to specified position"""
        pos = abs(round(position * self.PULSES_PER_ROTATION / 360))
        sign_bit = 0b10000000 if position > 0 else 0
        
        while pos > 0:
            p = min(pos, 0xffff)
            pos -= p
            
            self.get_logger().debug(f'rotate: pos={pos} id={id} sign={sign_bit} speed={speed} p={p}')
            
            [ok] = await self.send(id, [
                self.ROTATE,
                sign_bit + speed,
                (p >> 8) & 0xff,
                p & 0xff
            ], 1)
            
        return ok == 1

    async def get_pulses(self, id: int) -> int:
        """Get current pulse count"""
        [a, b, c, d] = await self.send(id, [self.GET_PULSES], 4)
        return (a << 24) + (b << 16) + (c << 8) + d

    async def get_angle(self, id: int) -> float:
        """Get current angle"""
        pulses = await self.get_pulses(id)
        return pulses * 360 / self.PULSES_PER_ROTATION

    async def stop(self, id: int) -> None:
        """Stop servo"""
        await self.send(id, [self.STOP], 1)

    async def target_angle_callback(self, msg: Float32) -> None:
        """Handle new target angle commands"""
        try:
            await self.rotate(1, 120, msg.data)
            self.get_logger().info(f'Moving to angle: {msg.data}')
        except Exception as e:
            self.get_logger().error(f'Failed to rotate servo: {str(e)}')
    
    async def publish_angle(self) -> None:
        """Publish current angle periodically"""
        try:
            angle = await self.get_angle(1)
            msg = Float32()
            msg.data = float(angle)
            self.angle_pub.publish(msg)
        except Exception as e:
            self.get_logger().error(f'Failed to publish angle: {str(e)}')

    @staticmethod
    async def sleep(seconds: float) -> None:
        """Async sleep helper"""
        await rclpy.sleep(seconds)

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