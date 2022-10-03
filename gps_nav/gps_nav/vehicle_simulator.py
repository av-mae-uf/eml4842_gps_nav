from cmath import isclose
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock

import numpy as np
import math

from gps_nav.uf_nav_support import update_vehicle_pose
from gps_nav.uf_support import *

from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist

class VehicleSimulator(Node):

    def __init__(self):
        super().__init__('vehicle_simulator')

        self.declare_parameter('starting_position', [0.0, 0.0, 0.0])
        self.declare_parameter('starting_ang_deg', 190.0)

        self.subscription = self.create_subscription(
            Twist, 'vehicle_command',
            self.vehicle_command_callback,
            10)
        self.subscription  # prevent unused variable warning

        self.publisher = self.create_publisher(PoseStamped, 'vehicle_pose', 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.old_position = np.array([0.0, 0.0, 0.0])
        param_value = self.get_parameter('starting_position').value

        self.old_position[0] = param_value[0]
        self.old_position[1] = param_value[1]
        self.old_position[2] = param_value[2]
        self.old_heading_deg = self.get_parameter('starting_ang_deg').value

        self.rad_of_curvature = 99999.0
        self.speed = 0.0
        self.cnt = 0

        self.D2R = math.pi/180.0
        self.R2D = 180.0/math.pi

    def vehicle_command_callback(self, msg):
        #self.get_logger().info('I heard rad_of_curvature = "%lf"' % msg.radius_of_curvature)
        # just store the incoming values

        if (value_near(msg.angular.z, 0.0, 0.0001) or value_near(msg.linear.x, 0.0, 0.001)):
            self.rad_of_curvature = 999999.9
        else:
            self.rad_of_curvature = msg.linear.x/msg.angular.z
        self.speed = msg.linear.x
        self.cnt +=1

    def timer_callback(self):
        if(self.cnt < 1):
            self.old_heading_deg = self.get_parameter('starting_ang_deg').value
            #self.get_logger().info('FIRST angle = %lf deg' % self.old_heading_deg)

        # update the vehicle pose based on stored data
        #print('type of self.old_heading_deg = ', type(self.old_heading_deg), self.old_heading_deg)
        #print('type of self.old_position = ', type(self.old_position), self.old_position)
        #print('veh_sim; rad_of_curve = ', self.rad_of_curvature)

        ans = update_vehicle_pose(self.old_position, self.old_heading_deg, self.rad_of_curvature, self.speed)

        new_position    = ans[0]
        new_heading_deg = ans[1]

        msg = PoseStamped()
        time_stamp = Clock().now()
        msg.header.stamp = time_stamp.to_msg()
        msg.header.frame_id = 'my_frame'
        msg.pose.position.x = new_position[0]
        msg.pose.position.y = new_position[1]
        msg.pose.position.z = new_position[2]
        msg.pose.orientation.w = math.cos(self.D2R * new_heading_deg/2.0)
        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = math.sin(self.D2R * new_heading_deg/2.0)
       
        self.publisher.publish(msg)

        self.old_position = new_position
        self.old_heading_deg = new_heading_deg


def main(args=None):
    rclpy.init(args=args)

    my_vehicle_simulator = VehicleSimulator()

    rclpy.spin(my_vehicle_simulator)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    my_vehicle_simulator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()