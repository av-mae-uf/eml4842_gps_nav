# Python Imports
import math
import numpy as np

# ROS2 Imports
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Point, TransformStamped
from visualization_msgs.msg import Marker, MarkerArray

# Custom Imports
from gps_nav_interfaces.srv import GetRoutePoses

import gps_nav.uf_support.route_support as route_supp

class RoutePoseVisualizer(Node):

    def __init__(self):
        super().__init__('route_pose_visualizer')
        
        # Create service client from map provider
        self.cli = self.create_client(GetRoutePoses, 'get_route_poses')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

        self.req = GetRoutePoses.Request()

        # Create a latching publisher for route_pose_viz
        latching_qos = QoSProfile(depth=1, 
        	durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL)
        self.publisher_ = self.create_publisher(MarkerArray, 'route_pose_viz', qos_profile=latching_qos)

        self.tf_static_broadcaster = StaticTransformBroadcaster(self)

        # Variables
        self.route = []
        self.route_requested = False

    def create_line_strip(self):

        route_markers = MarkerArray()
        id = 0
        stamp = self.get_clock().now().to_msg()
        for route_segment in self.route:
            segment_marker = Marker()
            segment_marker.header.stamp = stamp
            segment_marker.header.frame_id = "utm_local"
            segment_marker.type = 4 # LINE_STRIP
            segment_marker.color = ColorRGBA(r=0.88, g=0.53, b=0.25, a=1.0)
            segment_marker.scale.x = 0.2
            segment_marker.id = id
            id = id + 1

            for i in range(21):
                pt = route_supp.get_point_on_route(route_segment, i/20.0)
                segment_marker.points.append(Point(x=pt[0], y=pt[1]))

            route_markers.markers.append(segment_marker)

        self.publisher_.publish(route_markers)

    def send_request(self):
        self.route_requested = True
        self.future = self.cli.call_async(self.req)

    def send_transform(self, transformation):
        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'utm'
        t.child_frame_id = 'utm_local'

        # Transform only moves the origin's coordinates. No rotation.
        t.transform.translation.x = float(transformation.position.x)
        t.transform.translation.y = float(transformation.position.y)
        t.transform.translation.z = 0.0

        self.tf_static_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)

    node = RoutePoseVisualizer()
    node.send_request()

    route_poses = []
    while rclpy.ok():
        rclpy.spin_once(node)
        if node.future.done() and node.route_requested:
            try:
                response = node.future.result()
            except Exception as e:
                node.get_logger().warn('Service call failed %r' % (e,))
            else:
                num_poses = response.num_route_poses
                easting_offset, northing_offset = response.mypose[0].position.x, response.mypose[0].position.y

                # Create list of route_pose
                for i in range(num_poses):
                    ptx = response.mypose[i].position.x - easting_offset
                    pty = response.mypose[i].position.y - northing_offset
                    ptz = response.mypose[i].position.z
                    qw = response.mypose[i].orientation.w
                    qx = response.mypose[i].orientation.x
                    qy = response.mypose[i].orientation.y
                    qz = response.mypose[i].orientation.z
                    state = response.state[i]
                    heading_rad = 2.0*math.atan2(qz, qw)
                    route_poses.append(route_supp.route_pose_class(np.array([ptx, pty, ptz]), heading_rad, state))

                # create the route_segments list
                
                node.route = route_supp.create_route_segments(route_poses, response.want_loop, -1.0) # The -1.0 will prevent the extra calcs for pts

                node.send_transform(response.mypose[0])

                # create the line strip data to send to rviz
                node.get_logger().info('Publishing route visualization.')
                node.create_line_strip()
                node.route_requested = False

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

