# Call this program as follows:
#         py process_pose_list.py  input_file.txt  output_file.txt
#
# This program opens the file 'input_file.txt' which has a list of
# poses that will define a route.  Each line of the input file has:
#    Easting (m), Northing (m) ,angle (deg), state
# where the values are separated by a comma.
# The file 'output_file.txt' will have path data on each line as:
#     path_seg_number, u, x (m), y (m), heading (deg), radius of curvature (m), flag_to_identify_given_pose
# where the values are separated by a comma.

from operator import truediv
import sys
import math
import numpy as np
import csv

from gps_nav.uf_nav_support import *

route_poses = []
route_segments = []

def main():
    num_arg = len(sys.argv)
    if (num_arg < 3):
        print('Not enough arguments provided:')
        print('\tUseage:  convert_to_UTM.py  input_file.txt  output_file.txt')
        exit()
    else:
        fp = 0
        fp_out = 0
        in_filename = sys.argv[1]
        out_filename = sys.argv[2]
    
    try:
        fp = open(in_filename, 'r')
        csvreader = csv.reader(fp)
        rows = []
        for row in csvreader:
            rows.append(row)
    except IOError:
        print ('Error: The file ' + in_filename + ' does not exist.')
    finally:
        if (fp != 0):
            fp.close()
    try:
        fp_out = open(out_filename, 'w')
        print('# path_seg_number, u, x (m), y (m), heading (deg), radius of curvature (m), flag_to_identify_given_pose', file = fp_out)
        for row in rows:
            if (row[0][0] == '#' or row[0][0] == '\n'):
                continue
            else:
                myptx =  float(row[0])
                mypty = float(row[1])
                myptz = 0.0
                myheadingrad = float(row[2])*math.pi/180.0
                mystate = 1
                myw1 = 1.0
                myw2 = 1.0

                route_poses.append(route_pose_class(np.array([myptx, mypty, myptz]), myheadingrad, mystate, myw1, myw2))
                #print('Got row number ', row, file = fp_out)
        
        want_loop = False
        route_segments = create_route_segments(route_poses, want_loop)
        print('Num of route segments = ', len(route_segments))

        cnt = 0
        num_on_seg = 20
        for seg in route_segments:
            print('\tseg num = ', cnt)
            cnt = cnt +1

            for i in range(0, num_on_seg):
                if i==0:
                    pose_point = 1
                else:
                    pose_point = 0

                u = i * (1.0/num_on_seg)
                pt = get_point_on_route(seg, u)
                heading_deg = get_heading_rad_at_u(seg, u) * 180.0/math.pi
                radius_of_curvature = get_radius_at_u(seg, u)
                print(cnt, ', ', u, ', ', pt[0], ', ', pt[1], ', ', heading_deg, ', ', radius_of_curvature, ', ', pose_point, file = fp_out)

    except IOError:
        print('Error: The output file ' + out_filename + ' could not be created.')
    finally:
        if (fp_out !=0):
            fp_out.close()

if __name__ == '__main__':
    main()
