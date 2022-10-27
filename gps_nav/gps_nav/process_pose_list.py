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
import matplotlib.pyplot as plt

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

        xvals = []
        yvals = []
        xp_vals = []
        yp_vals = []
        heading_vals_deg = []
        curvature_vals = []

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
                xvals.append(pt[0])
                yvals.append(pt[1])
                heading_vals_deg.append(heading_deg)
                curvature_vals.append(1.0/radius_of_curvature)
                if pose_point == 1:
                    xp_vals.append(pt[0])
                    yp_vals.append(pt[1])

        fig, ax = plt.subplots()
        ax.axis('equal')
        ax.plot(xvals, yvals)
        ax.scatter(xp_vals, yp_vals, color='red')

        fig2, ax2 = plt.subplots()
        ax2.set_title('heading, degrees')
        ax2.plot(range(0,len(heading_vals_deg)),heading_vals_deg )

        fig3, ax3 = plt.subplots()
        ax3.set_title('curvature, m^-1')
        ax3.plot(range(len(curvature_vals)), curvature_vals)
        
        plt.show()

    except IOError:
        print('Error: The output file ' + out_filename + ' could not be created.')
    finally:
        if (fp_out !=0):
            fp_out.close()

if __name__ == '__main__':
    main()
