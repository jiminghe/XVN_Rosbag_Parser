#!/usr/bin/env python
import rosbag
import rospy
import numpy as np
import csv
import glob
import os
import argparse
from datetime import datetime

# Constants for WGS84
wgs84_a = 6378137.0
wgs84_b = 6356752.314245
wgs84_e_2 = 6.69437999014e-3
wgs84_a_2 = wgs84_a * wgs84_a
wgs84_b_2 = wgs84_b * wgs84_b
wgs84_e_prime_2 = wgs84_a_2 / wgs84_b_2 - 1

def ECEF_To_LLA(ecef):
    x, y, z = ecef
    x_2 = x * x
    y_2 = y * y
    z_2 = z * z
    r_2 = x_2 + y_2
    r = np.sqrt(r_2)
    F = 54.0 * wgs84_b_2 * z_2
    G = r_2 + (1 - wgs84_e_2) * z_2 - wgs84_e_2 * (wgs84_a_2 - wgs84_b_2)
    c = wgs84_e_2 * wgs84_e_2 * F * r_2 / (G * G * G)
    s = np.cbrt(1 + c + np.sqrt(c * c + 2 * c))
    P = F / (3.0 * (s + 1.0 + 1.0 / s) * (s + 1.0 + 1.0 / s) * G * G)
    Q = np.sqrt(1 + 2 * wgs84_e_2 * wgs84_e_2 * P)
    r0 = -P * wgs84_e_2 * r / (1 + Q) + np.sqrt(0.5 * wgs84_a_2 * (1.0 + 1.0 / Q) - (P * (1 - wgs84_e_2) * z_2 / (Q + Q * Q)) - 0.5 * P * r_2)
    t1 = r - wgs84_e_2 * r0
    t1_2 = t1 * t1
    U = np.sqrt(t1_2 + z_2)
    V = np.sqrt(t1_2 + (1 - wgs84_e_2) * z_2)
    a_V = wgs84_a * V
    z0 = wgs84_b_2 * z / a_V
    h = U * (1 - wgs84_b_2 / a_V)
    lat = np.arctan2((z + wgs84_e_prime_2 * z0), r)
    lon = np.arctan2(y, x)

    # Convert radians to degrees
    lat = np.degrees(lat)
    lon = np.degrees(lon)

    return np.array([lat, lon, h])


def parse_arguments():
    parser = argparse.ArgumentParser(description='Convert ROS bag files to CSV.')
    parser.add_argument('path', nargs='?', default='.', help='Directory containing the bag files (defaults to current directory)')
    return parser.parse_args()

def main():
    args = parse_arguments()

    # Initialize the ROS node
    rospy.init_node('convert_rosbag_to_csv', anonymous=True)

    # Get the list of all .bag files in the specified directory
    bag_files = glob.glob(os.path.join(args.path, '*.bag'))
    
    # Create the sub-folder if it doesn't exist
    csv_folder_path = os.path.join(args.path, 'csv_export')
    if not os.path.exists(csv_folder_path):
        os.makedirs(csv_folder_path)

    # Loop through each bag file and process it
    for bag_file_path in bag_files:
        # Extract the base filename without extension
        base_filename = os.path.basename(os.path.splitext(bag_file_path)[0])
        # Create a corresponding CSV filename within the sub-folder
        output_csv_path = os.path.join(csv_folder_path, base_filename + '.csv')

        # Open the rosbag file and corresponding CSV file
        with rosbag.Bag(bag_file_path, 'r') as bag, open(output_csv_path, 'w', newline='') as csvfile:
            # Prepare CSV writer
            csv_writer = csv.writer(csvfile)
            csv_writer.writerow(["Timestamp", "Latitude", "Longitude", "Altitude"])

            # Read messages from the bag
            for topic, msg, t in bag.read_messages(topics=["/fusion_optim/odometry"]):
                # Extract ECEF coordinates
                ecef_coords = (msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z)
                
                # Convert ECEF to LLA
                lla_coords = ECEF_To_LLA(ecef_coords)
                
                # Combine secs and nsecs to generate timestamp in seconds with 9 decimals
                timestamp = msg.header.stamp.secs + msg.header.stamp.nsecs * 1e-9
                
                # Write to CSV
                csv_writer.writerow([timestamp] + lla_coords.tolist())

        print(f"Data from {bag_file_path} has been written to {output_csv_path}")

if __name__ == '__main__':
    main()
