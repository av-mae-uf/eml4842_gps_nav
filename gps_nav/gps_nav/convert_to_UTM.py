import sys

import utm


def main():
    """
        Call this program as follows:
             python3 convert_to_UTM.py  input_file.txt  output_file.txt
    
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
        latlon_data = fp.readlines()

    data = []
    for row in latlon_data:
        lat, long = [float(val) for val in row.split(',')]
        easting, northing, zone_number, zone_letter = utm.from_latlon(latitude=lat, longitude=long)
        data.append([easting, northing, zone_number, zone_letter])

    with open(out_filename, 'w') as fp_out:
        for point in data:
            print(f"{point[0]:.2f}, {point[1]:.2f}, {point[2]}, {point[3]}", file = fp_out)


if __name__ == '__main__':
    main()
