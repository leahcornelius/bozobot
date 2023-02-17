import rclpy
from rclpy.node import Node

from std_msgs.msg import Int16


class TOFSensorPublisher(Node):
    def __init__(self):
        super().__init__('TOFSensorPublisher')
        self.publisher_ = self.create_publisher(Int16, 'TOFReadings', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = Int16()
        msg.data = self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = TOFSensorPublisher()

    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
