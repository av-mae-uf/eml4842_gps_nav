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

import sys
import math

import numpy as np
import csv
import matplotlib.pyplot as plt

import gps_nav.uf_support.route_support as route_supp


def main():
    num_arg = len(sys.argv)
    if (num_arg < 3):
        print('Not enough arguments provided:')
        print('\tUseage:  process_pose_list.py  input_file.txt  output_file.txt')
        return

    in_filename = sys.argv[1]
    out_filename = sys.argv[2]
    
    rows = []
    with open(in_filename, 'r') as fp:
        csvreader = csv.reader(fp)
        for row in csvreader:
            rows.append(row)

    route_poses = []
    for row in rows:
        if (row[0][0] == '#' or row[0][0] == '\n'):
            continue
        else:
            myptx =  float(row[0])
            mypty = float(row[1])
            myptz = 0.0
            myheadingrad = float(row[2])*math.pi/180.0
            mystate = 1

            route_poses.append(route_supp.route_pose_class(np.array([myptx, mypty, myptz]), myheadingrad, mystate))
        
    route_segments = route_supp.create_route_segments(route_poses, want_loop=False)
    print('Num of route segments = ', len(route_segments))

    xvals = []
    yvals = []
    xp_vals = []
    yp_vals = []
    heading_vals_deg = []
    curvature_vals = []

    cnt = 0
    num_on_seg = 20
    with open(out_filename, 'w') as fp_out:
        print('# path_seg_number, u, x (m), y (m), heading (deg), radius of curvature (m), flag_to_identify_given_pose', file = fp_out)
        for seg in route_segments:
            print('\tseg num = ', cnt)
            cnt = cnt +1

            for i in range(0, num_on_seg+1):
                pose_point = 1 if i == 0 else 0 # Ternary Operator

                u = i * (1.0/num_on_seg)
                pt = route_supp.get_point_on_route(seg, u)
                heading_deg = route_supp.get_heading_rad_at_u(seg, u) * 180.0/math.pi
                radius_of_curvature = route_supp.get_radius_at_u(seg, u)
                print(cnt, ', ', u, ', ', pt[0], ', ', pt[1], ', ', heading_deg, ', ', radius_of_curvature, ', ', pose_point, file = fp_out)
                xvals.append(pt[0])
                yvals.append(pt[1])
                heading_vals_deg.append(heading_deg)
                curvature_vals.append(1.0/radius_of_curvature)
                if pose_point == 1:
                    xp_vals.append(pt[0])
                    yp_vals.append(pt[1])

    plt.figure(num=1, figsize=(12, 6), dpi=90)
    ax = plt.subplot(1, 2, 1)
    ax.axis('equal')
    ax.set_ylabel('Northing (meters)')
    ax.set_xlabel('Easting (meters)')
    ax.plot(xvals, yvals)
    ax.scatter(xp_vals, yp_vals, color='red')

    ax2 = plt.subplot(2, 2, 2)
    ax2.set_title('heading, degrees')
    ax2.plot(range(0,len(heading_vals_deg)),heading_vals_deg )

    ax3 = plt.subplot(2, 2, 4)
    ax3.set_title('curvature, 1/m')
    ax3.plot(range(len(curvature_vals)), curvature_vals)
    
    plt.tight_layout()
    plt.show()


if __name__ == '__main__':
    main()
