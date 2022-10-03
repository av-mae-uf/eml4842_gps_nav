import rclpy
from rclpy.node import Node

from gps_nav_interfaces.msg import LookAheadSpecs

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('motion_spec_provider')

        self.declare_parameter('look_ahead_dist', 8.0)
        self.declare_parameter('speed', 0.0)

        self.publisher_ = self.create_publisher(LookAheadSpecs, 'look_ahead_specs', 10)
        timer_period = 2.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = LookAheadSpecs()
        msg.look_ahead_dist = self.get_parameter('look_ahead_dist').value
        msg.speed = self.get_parameter('speed').value
        self.publisher_.publish(msg)
        #self.get_logger().info('Publishing: %f, %f' % (msg.look_ahead_dist, msg.speed))
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
