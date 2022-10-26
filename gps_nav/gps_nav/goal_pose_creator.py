import sys
import math

import numpy as np
from pickletools import read_bytes1, read_bytes4
import rclpy
from rclpy.node import Node

from tf2_ros import TransformBroadcaster
from gps_nav_interfaces.srv import GetRoutePoses
from gps_nav_interfaces.msg import CurrentGoalPose
from gps_nav_interfaces.msg import LookAheadSpecs
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker

# from gps_nav.uf_nav_support import *
import gps_nav.uf_nav_support as uf_nav

D2R = math.pi/180.0
R2D = 180.0/math.pi

route_poses = []

class GoalPoseCreator(Node):

    def __init__(self):
        super().__init__('carrot_creator')
        self.cli = self.create_client(GetRoutePoses, 'get_route_poses')

        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = GetRoutePoses.Request()

        # the default condition is true ; send data to rviz
        self.declare_parameter('send_to_rviz', True)
          
        # subscribe to 'vehicle_pose' topic
        self.subscription_vehicle_pose = self.create_subscription(
            PoseStamped, 'vehicle_pose', self.vehicle_pose_callback, 1) #was 10

        # subscribe to 'look_ahead_specs' topic
        self.subscription_look_ahead_specs = self.create_subscription(
            LookAheadSpecs, 'look_ahead_specs', self.look_ahead_specs_callback, 10)

        # prepare to publish 'current_goal_pose' topic
        self.publisher_current_goal_pose = self.create_publisher(
            CurrentGoalPose, 'current_goal_pose', 10)

        self.send_to_rviz = self.get_parameter('send_to_rviz')

        #print('sendtorvizvalue = ', self.send_to_rviz.value)
        if (self.send_to_rviz.value):
            print('going to rviz2')
            # prepare to publish 'route_points_to_rviz' topic
            self.publisher_marker = self.create_publisher(
                Marker, 'route_points_to_rviz', 10)
            timer_period = 3.0  # seconds
            self.timer = self.create_timer(timer_period, self.timer_callback)

            # prepare to publish the 'closest_point' topic for rviz
            self.publisher_closest_point = self.create_publisher(
                PoseStamped, 'closest_point_to_rviz', 10)

            # prepare to publish the 'look_ahead_pose' topic for rviz
            self.publisher_look_ahead_pose = self.create_publisher(
                PoseStamped, 'look_ahead_pose_to_rviz', 10)

            # prepare to publish the 'vehicle_pose_to_rviz' topic for rviz
            self.publisher_vehicle_pose = self.create_publisher(PoseStamped, 'vehicle_pose_to_rviz', 10)

            # create the line strip data to send to rviz
            self.line_strip = Marker()

            # Initialize the transform broadcaster
            self.br = TransformBroadcaster(self)

        # define variables
        self.speed = 0.0            # meters/second
        self.look_ahead_dist = 8.0  # meters
        self.g_xUTM = 0.0           # meters     
        self.g_yUTM = 0.0           # meters

        self.num_route_segments = 0
        self.route_segments = []
        self.look_ahead_pose = uf_nav.route_pose_class()
        self.my_closest_pt = np.array([0.0, 0.0, 0.0])
        self.current_seg_num = 0
        self.look_ahead_seg_num = 0
        self.stop_flag = False
        self.already_sent = False
        self.ready_to_process = False

        self.have_look_ahead_specs = False
        self.have_vehicle_pose = False

        self.want_loop = False

    def send_request(self):  # no data is sent in the request
        self.future = self.cli.call_async(self.req)

    def timer_callback(self):
        self.publisher_marker.publish(self.line_strip)
        self.already_sent = True

    def look_ahead_specs_callback(self, msg):
        self.have_look_ahead_specs = True
        self.speed = msg.speed
        self.look_ahead_dist = msg.look_ahead_dist

    def vehicle_pose_callback(self, msg):
        if not self.ready_to_process:
            return
        
        self.have_vehicle_pose = True
        self.g_xUTM = msg.pose.position.x
        self.g_yUTM = msg.pose.position.y

        vehicle_pt = np.array([self.g_xUTM, self.g_yUTM, 0.0])

        # print('look_ahead_dist = ', look_ahead_dist,' vehicle_pt = ', vehicle_pt[0], ', ', vehicle_pt[1], ', ', vehicle_pt[2], \
        #      ', current_seg_num = ', self.current_seg_num, ', lenth of route segs = ', len(self.route_segments))

        ans = uf_nav.get_look_ahead_point(
            self.look_ahead_dist, vehicle_pt, self.route_segments, self.current_seg_num)

        self.look_ahead_pose = ans[0]
        self.my_closest_pt = ans[1]
        self.current_seg_num = ans[2]
        self.look_ahead_seg_num = ans[3]
        self.stop_flag = ans[4]


        # publish the current_goal_pose topic message
        out_msg = CurrentGoalPose()
        out_msg.current_goal_pose.pose.position.x = self.look_ahead_pose.pt[0]
        out_msg.current_goal_pose.pose.position.y = self.look_ahead_pose.pt[1]
        out_msg.current_goal_pose.pose.position.z = self.look_ahead_pose.pt[2]
        out_msg.current_goal_pose.pose.orientation.w = math.cos(
            self.look_ahead_pose.heading_rad/2.0)
        out_msg.current_goal_pose.pose.orientation.x = 0.0
        out_msg.current_goal_pose.pose.orientation.y = 0.0
        out_msg.current_goal_pose.pose.orientation.z = math.sin(
            self.look_ahead_pose.heading_rad/2.0)
        
        out_msg.state = int(self.route_segments[self.current_seg_num].state)
        #if(self.route_segments[self.current_seg_num].state == myState.END_PLUS_ONE.value and not self.want_loop):
        if(self.want_loop == False and self.current_seg_num == self.num_route_segments-1):
            out_msg.speed = 0.0
        else:
            out_msg.speed = self.speed

        #if(self.have_look_ahead_specs):
        self.publisher_current_goal_pose.publish(out_msg)

        if (self.send_to_rviz.value):
            # publish the 'closet_point' topic to rviz
            out_msg2 = PoseStamped()
            out_msg2.header.frame_id = 'my_frame'
            out_msg2.header.stamp = self.get_clock().now().to_msg()
            out_msg2.pose.position.x = self.my_closest_pt[0]
            out_msg2.pose.position.y = self.my_closest_pt[1]
            out_msg2.pose.position.z = self.my_closest_pt[2]
            out_msg2.pose.orientation.w = 1.0
            out_msg2.pose.orientation.x = 0.0
            out_msg2.pose.orientation.y = 0.0
            out_msg2.pose.orientation.z = 0.0
            self.publisher_closest_point.publish(out_msg2)

            # publish the 'look_ahead_pose' topic to rviz
            out_msg3 = PoseStamped()
            out_msg3.header.frame_id = 'my_frame'
            out_msg3.header.stamp = self.get_clock().now().to_msg()
            out_msg3.pose.position.x = self.look_ahead_pose.pt[0]
            out_msg3.pose.position.y = self.look_ahead_pose.pt[1]
            out_msg3.pose.position.z = self.look_ahead_pose.pt[2]
            out_msg3.pose.orientation.w = math.cos(
                self.look_ahead_pose.heading_rad/2.0)
            out_msg3.pose.orientation.x = 0.0
            out_msg3.pose.orientation.y = 0.0
            out_msg3.pose.orientation.z = math.sin(
                self.look_ahead_pose.heading_rad/2.0)
            self.publisher_look_ahead_pose.publish(out_msg3)

            # publish the 'vehicle_pose_to_rviz' topic to rviz
            out_msg4 = PoseStamped()
            out_msg4.header.frame_id = 'my_frame'
            out_msg4.header.stamp = self.get_clock().now().to_msg()
            out_msg4.pose.position.x = msg.pose.position.x
            out_msg4.pose.position.y = msg.pose.position.y
            out_msg4.pose.position.z = msg.pose.position.z
            out_msg4.pose.orientation.w = msg.pose.orientation.w
            out_msg4.pose.orientation.x = 0.0
            out_msg4.pose.orientation.y = 0.0
            out_msg4.pose.orientation.z = msg.pose.orientation.z
            self.publisher_vehicle_pose.publish(out_msg4)

            # send out the tf2 for the vehicle pose
            t = TransformStamped()

            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = 'my_frame'
            t.child_frame_id = 'vehicle_frame'

            t.transform.translation.x = msg.pose.position.x
            t.transform.translation.y = msg.pose.position.y
            t.transform.translation.z = msg.pose.position.z

            t.transform.rotation.x = msg.pose.orientation.x
            t.transform.rotation.y = msg.pose.orientation.y
            t.transform.rotation.z = msg.pose.orientation.z
            t.transform.rotation.w = msg.pose.orientation.w

            # Send the transformation
            self.br.sendTransform(t)

    def create_line_strip(self):
        self.line_strip.header.frame_id = 'my_frame'
        self.line_strip.header.stamp = self.get_clock().now().to_msg()
        self.line_strip.ns = 'complete_route'
        self.line_strip.pose.orientation.w = 1.0
        self.line_strip.id = 1
        self.line_strip.type = self.line_strip.LINE_STRIP

        self.line_strip.scale.x = 0.25
        self.line_strip.color.r = 1.0
        self.line_strip.color.g = 0.0
        self.line_strip.color.b = 0.0
        self.line_strip.color.a = 1.0

        if (not self.already_sent):
            for i in range(len(self.route_segments)):
                for j in range(50):
                    p = Point()
                    pt = uf_nav.get_point_on_route(self.route_segments[i], j/50.)
                    p.x = pt[0]
                    p.y = pt[1]
                    p.z = pt[2]

                    self.line_strip.points.append(p)
                    #print('p i=',i, ', j=', j, '\t', p.x, ', ', p.y, ', ', p.z)


def main(args=None):
    rclpy.init(args=args)

    goal_pose_creator = GoalPoseCreator()
    goal_pose_creator.send_request()

    while rclpy.ok():
        rclpy.spin_once(goal_pose_creator)

        if goal_pose_creator.future.done():
            try:
                response = goal_pose_creator.future.result()
            except Exception as e:
                goal_pose_creator.get_logger().warn(
                    'Service call failed %r' % (e,))
            else:
                num_poses = response.num_route_poses
                last_pose = response.mypose[-1]
                goal_pose_creator.get_logger().info(
                    f"Goal Pose Creator received {num_poses} poses. " + \
                    f"Last pt = {last_pose.position.x}, {last_pose.position.y}, {last_pose.position.z}")

                # create the route_poses array
                for i, pose in enumerate(response.mypose):
                    ptx = pose.position.x
                    pty = pose.position.y
                    ptz = response.mypose[i].position.z
                    qw = response.mypose[i].orientation.w
                    qx = response.mypose[i].orientation.x
                    qy = response.mypose[i].orientation.y
                    qz = response.mypose[i].orientation.z
                    mystate = response.state[i]
                    myheadingrad = 2.0*math.atan2(qz, qw)
                    myw1 = 1.0
                    myw2 = 1.0
                    route_poses.append(uf_nav.route_pose_class(
                        np.array([ptx, pty, ptz]), myheadingrad, mystate, myw1, myw2))
                    print(route_poses[i].pt[0], route_poses[i].pt[1], route_poses[i].pt[2], route_poses[i].heading_rad*180.0/math.pi, route_poses[i].state, route_poses[i].w1_for_subsequent_segment, route_poses[i].w2_for_subsequent_segment)

                # create the route_segments array
                goal_pose_creator.want_loop = response.want_loop
                goal_pose_creator.route_segments = uf_nav.create_route_segments(route_poses, goal_pose_creator.want_loop)

                goal_pose_creator.get_logger().info('Goal Pose creator made %d route segments.' %
                                                 len(goal_pose_creator.route_segments))

                # print_out_route_segments for debugging
                #fp = open("/home/cimar/dev_ws_uf/src/uf_nav/my_data/output/route_segments.txt", "w")
                #print('p0x, p0y, p0z, p1x, p1y, p1z, p2x, p2y, p2z, p3x, p3y, p3z, w1, w2, length, state', file=fp)
                # for i in range(len(carrot_creator.route_segments)):
                #    print(carrot_creator.route_segments[i].p0[0], ',', carrot_creator.route_segments[i].p0[1], ',', carrot_creator.route_segments[i].p0[2], ',',\
                #          carrot_creator.route_segments[i].p1[0], ',', carrot_creator.route_segments[i].p1[1], ',', carrot_creator.route_segments[i].p1[2], ',',\
                #          carrot_creator.route_segments[i].p2[0], ',', carrot_creator.route_segments[i].p2[1], ',', carrot_creator.route_segments[i].p2[2], ',',\
                #          carrot_creator.route_segments[i].p3[0], ',', carrot_creator.route_segments[i].p3[1], ',', carrot_creator.route_segments[i].p3[2], ',',\
                #          carrot_creator.route_segments[i].w1,    ',', carrot_creator.route_segments[i].w2,    ',', carrot_creator.route_segments[i].length, ',',\
                #          carrot_creator.route_segments[i].state, file=fp)

                # fp.close()

                goal_pose_creator.num_route_segments = num_poses

                # create the line strip data to send to rviz
                if (goal_pose_creator.send_to_rviz.value):
                    goal_pose_creator.create_line_strip()
                goal_pose_creator.ready_to_process = True
                goal_pose_creator.get_logger().info('Ready to proceed.')

            break

    rclpy.spin(goal_pose_creator)

    goal_pose_creator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()