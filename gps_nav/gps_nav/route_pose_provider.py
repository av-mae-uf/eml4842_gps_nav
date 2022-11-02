import math
import numpy as np

import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

from gps_nav.uf_support.route_support import route_pose_class

from gps_nav_interfaces.srv import GetRoutePoses 


class RoutePoseProvider(Node):

    def __init__(self):
        super().__init__('route_pose_generator')
        self.srv = self.create_service(GetRoutePoses, 'get_route_poses', self.get_route_poses_callback)        # CHANGE
        
        self.declare_parameter('pose_filename', 'pose_list.txt')  # default file name
        self.declare_parameter('want_loop', False)   # the default condition is false ; not to loop
        self.declare_parameter('state_defs', '{0:\'OFF\', 1:\'ON\', 2:\'OUTSIDE\', 3:\'ENTRY_EXTENSION_PT\', 4:\'EXIT_EXTENSION_PT\', 5:\'EXIT_TURN_PT\', 6:\'START\', 7:\'END\', 8:\'UTURN_PT1\', 9:\'UTURN_PT2\', 10:\'UTURN_PT3\', 11:\'CORNER\', 12:\'END_EXTENSION\'}')

        self.want_loop = self.get_parameter('want_loop').value
        self.pose_filename = self.get_parameter('pose_filename').value
        self.state_defs = self.get_parameter('state_defs').value

        self.read_in_route = False
        self.route_poses = []

    def get_route_poses_callback(self, request, response):
        self.get_logger().info("Incoming request for route.")

        if(self.read_in_route == False):
            self.get_logger().warn(f"Route has been requested before file has been read!")
            return

        num_poses = len(self.route_poses)
  
        if num_poses > 300:
            self.get_logger().warn(f"Route contains over 300 poses. Unable to complete request")
            return

        response.num_route_poses = num_poses
        response.want_loop = self.want_loop   # will be sending the value of the parameter (0 or 1)
        response.state_dictionary = self.state_defs
        response.offset_x = 0.0
        response.offset_y = 0.0

        for i, route_pose in enumerate(self.route_poses):
            response.mypose[i].position.x = route_pose.pt[0]
            response.mypose[i].position.y = route_pose.pt[1]
            response.mypose[i].position.z = route_pose.pt[2]
            response.mypose[i].orientation.w = math.cos(route_pose.heading_rad / 2.0)
            response.mypose[i].orientation.x = 0.0
            response.mypose[i].orientation.y = 0.0
            response.mypose[i].orientation.z = math.sin(route_pose.heading_rad / 2.0)
            response.state[i] = route_pose.state
            
        return response


def main(args=None):

    rclpy.init(args=args)

    route_pose_provider = RoutePoseProvider()

    if route_pose_provider.pose_filename == 'pose_list.txt':
        filename = get_package_share_directory('gps_nav') + '/data/' + route_pose_provider.pose_filename
    else:
        filename = route_pose_provider.pose_filename

    with open(filename, "r") as fp:
        route_pose_provider.get_logger().info(f"Opened the file: {filename}")
        route_poses = fp.readlines()

    if '#' in route_poses[0]:
        route_pose_provider.get_logger().info(f"Comment line found. Ignored.")
        route_poses.pop(0)

    for route_pose in route_poses:
        # Cast each comma seperated value as a float, then unpack the list
        easting, northing, heading, state = [float(val) for val in route_pose.split(',')]
        heading_rad = heading * math.pi/180.0

        route_pose_provider.route_poses.append(route_pose_class(np.array([easting, northing, 0.0]), heading_rad, int(state)))
    
    route_pose_provider.read_in_route = True
                    
    rclpy.spin(route_pose_provider)

    rclpy.shutdown()

if __name__ == '__main__':
    main()
