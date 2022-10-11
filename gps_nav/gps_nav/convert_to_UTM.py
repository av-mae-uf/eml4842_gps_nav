# Call this program as follows:
#         py convert_to_UTM.py  input_file.txt  output_file.txt
#
# This program opens the file 'input_file.txt' which has latitude and longitude data on each line
# where the values are separated by a comma.  Each latitude/longitude pair is converted to UTM coordinates.
# The file 'output_file.txt' will have the Easting, Northing, Zone Number, and Zone Letter values on each line.

import utm
import sys
import math
import numpy as np
import csv

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
        for row in rows:
            my_lat =  float(row[0])
            my_long = float(row[1])
            utmData = utm.from_latlon(my_lat, my_long)
            my_East  = utmData[0]
            my_North = utmData[1]
            my_Zone_Num = utmData[2]
            my_Zone_Letter = utmData[3]
            print(my_East, ', ', my_North, ', ', my_Zone_Num, ', ', my_Zone_Letter,file = fp_out)
    except IOError:
        print('Error: The output file ' + out_filename + ' could not be created.')
    finally:
        if (fp_out !=0):
            fp_out.close()


if __name__ == '__main__':
    main()
