from ament_index_python.packages import get_package_share_directory  # added so the pose_list.txt file can be found

from gps_nav_interfaces.srv import GetRoutePoses 

import rclpy
from rclpy.node import Node

import math
import numpy as np

from gps_nav.uf_nav_support import *
from gps_nav.uf_support import *

#route_pose = []

class RoutePoseProvider(Node):

    def __init__(self):
        super().__init__('route_pose_generator')
        self.srv = self.create_service(GetRoutePoses, 'get_route_poses', self.get_route_poses_callback)        # CHANGE
        
        self.declare_parameter('pose_filename', 'pose_list.txt')  # default file name
        self.declare_parameter('want_loop', False)   # the default condition is false ; not to loop
        self.declare_parameter('state_defs', '{0:\'OFF\', 1:\'ON\', 2:\'OUTSIDE\', 3:\'ENTRY_EXTENSION_PT\', 4:\'EXIT_EXTENSION_PT\', 5:\'EXIT_TURN_PT\', 6:\'START\', 7:\'END\', 8:\'UTURN_PT1\', 9:\'UTURN_PT2\', 10:\'UTURN_PT3\', 11:\'CORNER\', 12:\'END_EXTENSION\'}')

        self.want_loop = self.get_parameter('want_loop')
        self.pose_filename = self.get_parameter('pose_filename')
        self.state_defs = self.get_parameter('state_defs')

        self.D2R = math.pi/180.0
        self.R2D = 180.0/math.pi

        self.route_pose = []

    def get_route_poses_callback(self, request, response):
        self.get_logger().info('Incoming request for route points.\n' )

        num_poses = len(self.route_pose)
        response.num_route_poses = num_poses
        self.get_logger().info('num_poses = %d\n' % num_poses )

        response.want_loop = self.want_loop.value   # will be sending the value of the parameter (0 or 1)

        response.state_dictionary = self.state_defs.value

        response.offset_x = 0.0
        response.offset_y = 0.0

        for i in range(num_poses):
            response.mypose[i].position.x = self.route_pose[i].pt[0]
            response.mypose[i].position.y = self.route_pose[i].pt[1]
            response.mypose[i].position.z = self.route_pose[i].pt[2]
            response.mypose[i].orientation.w = math.cos(self.route_pose[i].heading_rad/2.0)
            response.mypose[i].orientation.x = 0.0
            response.mypose[i].orientation.y = 0.0
            response.mypose[i].orientation.z = math.sin(self.route_pose[i].heading_rad/2.0)
            response.state[i] = self.route_pose[i].state
            

        return response


def main(args=None):

    rclpy.init(args=args)

    cnt = 0

    route_pose_provider = RoutePoseProvider()

    package_share_directory = get_package_share_directory('gps_nav')
    my_filename = package_share_directory + '/my_data/' + route_pose_provider.pose_filename.value

    if (FileCheck(my_filename) == 0):
        route_pose_provider.get_logger().info('The file ' + my_filename + ' did not exist.\n' )
    else:
        route_pose_provider.get_logger().info('Will open file  ' + my_filename + '\n' )
        fp = open(my_filename, "r")
        for x in fp:
            if (x[0] == '#' or x[0] == '\n'):
                continue
            else:
                cnt = cnt + 1
                a = x.split(',')
                
                myptx = float(a[0])
                mypty = float(a[1])
                myptz = 0.0
                myheadingrad = float(a[2])*route_pose_provider.D2R
                myw1 = 1.0
                myw2 = 1.0
                mystate = int(a[3])
                route_pose_provider.route_pose.append(route_pose_class(np.array([myptx, mypty, myptz]), myheadingrad, mystate, myw1, myw2))
                    
        fp.close()
        

    rclpy.spin(route_pose_provider)

    rclpy.shutdown()

if __name__ == '__main__':
    main()
