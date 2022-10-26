import sys
import csv

import utm
import numpy as np


def main():
    """
        Call this program as follows:
             py convert_to_UTM.py  input_file.txt  output_file.txt
    
        This program opens the file 'input_file.txt' which has latitude and longitude data on each line
        where the values are separated by a comma.  Each latitude/longitude pair is converted to UTM coordinates.
        The file 'output_file.txt' will have the Easting, Northing, Zone Number, and Zone Letter values on each line.
    
    """
    num_arg = len(sys.argv)
    if (num_arg < 3):
        print('Not enough arguments provided:')
        print('\tUseage:  convert_to_UTM.py  input_file.txt  output_file.txt')
        return

    in_filename = sys.argv[1]
    out_filename = sys.argv[2]

    with open(in_filename, 'r') as fp:
        csvreader = csv.reader(fp)

    data = []
    for row in csvreader:
        easting, northing, zone_number, zone_letter = utm.from_latlon(latitude=float(row[0]), longitude=float(row[1]))
        data.append([easting, northing, zone_number, zone_letter])

    with open(out_filename, 'w') as fp_out:
        for point in data:
            print(f"{point[0]}, {point[1]}, {point[2]}, {point[3]}", file = fp_out)


if __name__ == '__main__':
    main()
