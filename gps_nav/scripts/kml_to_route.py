import sys
import math
import xml.etree.ElementTree as ET
import re

import utm


def main():
    """
        Call this program as follows:
             python3 kml_to_route.py  example_file.kml  output_file.txt
    """
    num_arg = len(sys.argv)
    if (num_arg < 3):
        print('Not enough arguments provided:')
        print('\tUseage:  convert_to_UTM.py  input_file.txt  output_file.txt')
        return

    in_filename = sys.argv[1]
    out_filename = sys.argv[2]

    if '.kml' not in in_filename:
        print("Input file must be a KML file.")
        return
    
    # Load the KML and retrieve the element that contains the latitiude and longitude data
    root = ET.parse(in_filename).getroot()
    xPath = ".//kml:Document/kml:Placemark/kml:LineString/kml:coordinates"
    namespace = {'kml': 'http://www.opengis.net/kml/2.2'}
    coordinates = root.find(xPath, namespace)
    # Strip out new lines, tabs, and spaces
    coord_string = re.sub('[\s+]', '', coordinates.text)

    data = []
    for longlat_pair in coord_string.split(',0'):
        if longlat_pair != '':
            long, lat = [float(val) for val in longlat_pair.split(',')]

            easting, northing, zone_number, zone_letter = utm.from_latlon(latitude=lat, longitude=long)
            data.append([easting, northing, zone_number, zone_letter])

    output_data = []
    for i, point in enumerate(data):
        # Because points are given in pairs to provide heading. Only want to look at correspodsing pairs.
        if i % 2 == 0:
            heading = math.atan2(data[i+1][1] - point[1], data[i+1][0] - point[0]) * 180 / math.pi
            output_data.append([point[0], point[1], heading, 1])
    
    with open(out_filename, 'w') as fp:
        for pose in output_data:
            print(f"{pose[0]:.2f}, {pose[1]:.2f}, {pose[2]:.2f}, {pose[3]}", file = fp)


if __name__ == '__main__':
    main()
