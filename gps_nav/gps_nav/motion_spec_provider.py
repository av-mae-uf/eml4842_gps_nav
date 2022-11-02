import rclpy
from rclpy.node import Node

from gps_nav_interfaces.msg import LookAheadSpecs

class MotionSpecProvider(Node):

    def __init__(self):
        super().__init__('motion_spec_provider')

        self.declare_parameter('look_ahead_dist',  8.0)
        self.declare_parameter('speed', 0.0)

        self.publisher_ = self.create_publisher(LookAheadSpecs, 'look_ahead_specs', 10)
        self.timer = self.create_timer(timer_period_sec=2.0, callback=self.timer_callback)

    def timer_callback(self):
        msg = LookAheadSpecs()

        msg.look_ahead_dist = self.get_parameter('look_ahead_dist').value
        msg.speed = self.get_parameter('speed').value

        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    motion_spec_provider = MotionSpecProvider()

    rclpy.spin(motion_spec_provider)

    motion_spec_provider.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
