import math
import numpy as np

import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster

from geometry_msgs.msg import Twist, PoseStamped, TransformStamped

from gps_nav.uf_support.route_support import update_vehicle_pose

class VehicleSimulator(Node):

    def __init__(self):
        super().__init__('vehicle_simulator')

        self.declare_parameter('starting_position', [0.0, 0.0, 0.0])
        self.declare_parameter('starting_ang_deg', 190.0)

        self.subscription = self.create_subscription(
            Twist, 'vehicle_command',
            self.vehicle_command_callback,
            1)

        self.publisher = self.create_publisher(PoseStamped, 'vehicle_pose', 10)

        # Initialize the transform broadcaster
        self.br = TransformBroadcaster(self)

        self.timer = self.create_timer(timer_period_sec=0.1, callback=self.timer_callback)

        param_value = self.get_parameter('starting_position').value

        self.old_position = np.array([0.0, 0.0, 0.0])
        self.old_position[0] = param_value[0]
        self.old_position[1] = param_value[1]
        self.old_position[2] = param_value[2]

        self.old_heading_deg = self.get_parameter('starting_ang_deg').value

        self.rad_of_curvature = 99999.0
        self.speed = 0.0
        self.cnt = 0

    def vehicle_command_callback(self, msg):

        if (math.isclose(msg.angular.z, 0.0, abs_tol=0.0001) or math.isclose(msg.linear.x, 0.0, abs_tol=0.001)):
            self.rad_of_curvature = 999999.9
        else:
            self.rad_of_curvature = msg.linear.x / msg.angular.z
        
        self.speed = msg.linear.x
        self.cnt +=1

    def timer_callback(self):
        # PN - Is this here because the parameter might not be updated by the time a control is received?
        if(self.cnt < 1):
            self.old_heading_deg = self.get_parameter('starting_ang_deg').value

        new_position, new_heading_deg = update_vehicle_pose(self.old_position, self.old_heading_deg, self.rad_of_curvature, self.speed/10.0)

        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'utm'
        msg.pose.position.x = new_position[0]
        msg.pose.position.y = new_position[1]
        msg.pose.position.z = new_position[2]
        msg.pose.orientation.w = math.cos(math.pi / 180.0 * new_heading_deg / 2.0)
        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = math.sin(math.pi / 180.0 * new_heading_deg / 2.0)
       
        self.publisher.publish(msg)

        # send out the tf2 for the vehicle pose
        t = TransformStamped()

        t.header.stamp = msg.header.stamp
        t.header.frame_id = 'utm'
        t.child_frame_id = 'vehicle'

        t.transform.translation.x = msg.pose.position.x
        t.transform.translation.y = msg.pose.position.y
        t.transform.translation.z = msg.pose.position.z

        t.transform.rotation.x = msg.pose.orientation.x
        t.transform.rotation.y = msg.pose.orientation.y
        t.transform.rotation.z = msg.pose.orientation.z
        t.transform.rotation.w = msg.pose.orientation.w

        # Send the transformation
        self.br.sendTransform(t)

        self.old_position = new_position
        self.old_heading_deg = new_heading_deg


def main(args=None):
    rclpy.init(args=args)

    vehicle_simulator = VehicleSimulator()

    rclpy.spin(vehicle_simulator)

    vehicle_simulator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()