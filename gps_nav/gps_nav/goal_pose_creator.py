import math
import numpy as np

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from gps_nav_interfaces.srv import GetRoutePoses
from gps_nav_interfaces.msg import CurrentGoalPose, LookAheadSpecs

import gps_nav.uf_support.route_support as uf_nav

route_poses = []

class GoalPoseCreator(Node):

    def __init__(self):
        super().__init__('carrot_creator')
        self.cli = self.create_client(GetRoutePoses, 'get_route_poses')

        self.declare_parameter("distBetweenPoints", 0.05)
        self.dist_between_pts = self.get_parameter("distBetweenPoints").value

        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = GetRoutePoses.Request()
          
        # subscribe to 'vehicle_pose' topic
        self.subscription_vehicle_pose = self.create_subscription(
            PoseStamped, 'vehicle_pose', self.vehicle_pose_callback, 1)

        # subscribe to 'look_ahead_specs' topic
        self.subscription_look_ahead_specs = self.create_subscription(
            LookAheadSpecs, 'look_ahead_specs', self.look_ahead_specs_callback, 10)

        # prepare to publish 'current_goal_pose' topic
        self.publisher_current_goal_pose = self.create_publisher(
            CurrentGoalPose, 'current_goal_pose', 10)

        # define variables
        self.speed = 0.0            # meters/second
        self.look_ahead_dist = 8.0  # meters
        self.g_xUTM = 0.0           # meters     
        self.g_yUTM = 0.0           # meters

        self.num_route_segments = 0
        self.route_segments = []
        self.look_ahead_pose = uf_nav.route_pose_class()
        self.closest_pose = uf_nav.route_pose_class()
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

        ans = uf_nav.get_look_ahead_point_v2(
            self.look_ahead_dist, vehicle_pt, self.route_segments, self.current_seg_num)

        self.look_ahead_pose = ans[0]
        self.closest_pose = ans[1]
        self.current_seg_num = ans[2]
        self.look_ahead_seg_num = ans[3]
        self.stop_flag = ans[4]

        # publish the current_goal_pose topic message
        out_msg = CurrentGoalPose()
        out_msg.current_goal_pose.header.frame_id = 'utm'
        out_msg.current_goal_pose.header.stamp = self.get_clock().now().to_msg()
        out_msg.current_goal_pose.pose.position.x = self.look_ahead_pose.pt[0]
        out_msg.current_goal_pose.pose.position.y = self.look_ahead_pose.pt[1]
        out_msg.current_goal_pose.pose.position.z = self.look_ahead_pose.pt[2]
        out_msg.current_goal_pose.pose.orientation.w = math.cos(
            self.look_ahead_pose.heading_rad/2.0)
        out_msg.current_goal_pose.pose.orientation.x = 0.0
        out_msg.current_goal_pose.pose.orientation.y = 0.0
        out_msg.current_goal_pose.pose.orientation.z = math.sin(
            self.look_ahead_pose.heading_rad/2.0)

        out_msg.closest_pose.header.frame_id = 'utm'
        out_msg.closest_pose.header.stamp = out_msg.current_goal_pose.header.stamp
        out_msg.closest_pose.pose.position.x = self.closest_pose.pt[0]
        out_msg.closest_pose.pose.position.y = self.closest_pose.pt[1]
        out_msg.closest_pose.pose.position.z = self.closest_pose.pt[2]

        heading_at_closest_rad = uf_nav.get_heading_rad_at_u(self.route_segments[self.current_seg_num], 0.0)
        out_msg.closest_pose.pose.orientation.w = math.cos(heading_at_closest_rad/2.0)
        out_msg.closest_pose.pose.orientation.x = 0.0
        out_msg.closest_pose.pose.orientation.y = 0.0
        out_msg.closest_pose.pose.orientation.z = math.sin(heading_at_closest_rad/2.0)

        out_msg.closest_pose.pose.orientation.w = math.cos(
            self.closest_pose.heading_rad/2.0)
        out_msg.closest_pose.pose.orientation.x = 0.0
        out_msg.closest_pose.pose.orientation.y = 0.0
        out_msg.closest_pose.pose.orientation.z = math.sin(
            self.closest_pose.heading_rad/2.0)

        if(self.want_loop == False and self.current_seg_num == self.num_route_segments-1):
            out_msg.speed = 0.0
        else:
            out_msg.speed = self.speed

        self.publisher_current_goal_pose.publish(out_msg)


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
                last_pose = response.mypose[num_poses-1]
                goal_pose_creator.get_logger().info(
                    f"Goal Pose Creator received {num_poses} poses. ")

                # create the route_poses array
                for i in range(num_poses):
                    ptx = response.mypose[i].position.x
                    pty = response.mypose[i].position.y
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
                    
                # create the route_segments array
                goal_pose_creator.want_loop = response.want_loop
                goal_pose_creator.route_segments = uf_nav.create_route_segments(route_poses, goal_pose_creator.want_loop, goal_pose_creator.dist_between_pts)

                goal_pose_creator.get_logger().info('Goal Pose creator made %d route segments.' %
                                                 len(goal_pose_creator.route_segments))

                goal_pose_creator.num_route_segments = num_poses

                # now add the heading values at each u value for each segment
                for seg in goal_pose_creator.route_segments:
                    for i in np.arange(len(seg.pt_info)):
                        seg.pt_info[i,3] = seg.get_heading_rad(seg.pt_info[i,0])
                    
                goal_pose_creator.ready_to_process = True
                goal_pose_creator.get_logger().info('Ready to proceed.')

                #outfile = open('equally_spaced.csv', 'w')
                #for seg in goal_pose_creator.route_segments:
                #    for j in np.arange(len(seg.pt_info)):
                #        print(f'{seg.pt_info[j,0]}, {seg.pt_info[j,1]}, {seg.pt_info[j,2]}, {seg.pt_info[j,3]}, {seg.pt_info[j,4]}', file = outfile)
                #outfile.close()

            break

    rclpy.spin(goal_pose_creator)

    goal_pose_creator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()