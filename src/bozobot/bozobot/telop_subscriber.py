# A simple subscriber to the /cmd_vel topic, which sends commands to the robot using the motor driver in motor.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from bozobot.motor import MotorDriver, MotorDriverCommand

class TelopSubscriber(Node):
    def __init__(self):
        super().__init__('TelopSubscriber')
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('timeout', 0.1)
        md_logger = self.get_logger()
        self.motor_driver = MotorDriver(self.get_parameter('port').value, self.get_parameter('baudrate').value, md_logger, self.get_parameter('timeout').value)
        self.connection_event_publisher = self.create_publisher(String, 'connection_events', 10)
        
        if not self.motor_driver.wait_for_connection():
            self.get_logger().fatal("MotorDriver.WaitForConnectionFailed")
            self.connection_event_publisher.publish(String(data="MotorDriver.WaitForConnectionFailed"))
            self.destroy_node()
            return
        
        self.connection_event_publisher.publish(String(data="MotorDriver.Connected"))
        self.motor_driver.start()

    def mix_twist(self, twist):
        # Mix the linear and angular components of the twist message, to get the speed of the left and right motors
        left = twist.linear.x - twist.angular.z
        right = twist.linear.x + twist.angular.z
        # Convert to PWM values between -255 and 255
        left = int(left * 255)
        right = int(right * 255)
        # Clamp to the range -255 to 255
        left = max(-255, min(left, 255))
        right = max(-255, min(right, 255))
        return left, right

    def listener_callback(self, msg):        
        command = MotorDriverCommand('m', *self.mix_twist(msg))
        self.motor_driver.set_new_command(command, 0.1)

def main(args=None):
    rclpy.init(args=args)

    telop_subscriber = TelopSubscriber()
    try:
        rclpy.spin(telop_subscriber)
    except KeyboardInterrupt:
        telop_subscriber.get_logger().info('Keyboard interrupt received, shutting down')
    finally:
        telop_subscriber.motor_driver.kill()
        telop_subscriber.destroy_node()
        rclpy.shutdown()