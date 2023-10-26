#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan, PointCloud2
import sensor_msgs.point_cloud2 as pc2
import math

class LidarParser:
    def __init__(self):
        rospy.init_node('lidar_parser', anonymous=True)
        rospy.Subscriber("/velodyne_points", PointCloud2, self.callback)
        self.scan_pub = rospy.Publisher('/scan', LaserScan, queue_size=1)
        rospy.spin()

    def callback(self, data):
        scan_msg = LaserScan()
        scan_msg.header = data.header
        scan_msg.angle_min = -math.pi
        scan_msg.angle_max = math.pi
        scan_msg.angle_increment = 0.0174532925199  # Set the angle increment (1 degree in radians)
        scan_msg.time_increment = 0.0
        scan_msg.scan_time = 0.0
        scan_msg.range_min = 0.0
        scan_msg.range_max = 100.0

        ranges = []  # Create an empty list to store the range data

        for angle in range(0, 360):  # Assuming 360 degrees LiDAR data
            # Calculate the range from the (x, y, z) point
            range_val = self.get_range_at_angle(data, math.radians(angle))
            ranges.append(range_val)

        scan_msg.ranges = ranges
        self.scan_pub.publish(scan_msg)

    def get_range_at_angle(self, point_cloud, angle):
        for p in pc2.read_points(point_cloud, field_names=("x", "y", "z"), skip_nans=True):
            point_angle = math.atan2(p[1], p[0])  # Calculate the angle of the point
            if math.isclose(point_angle, angle, abs_tol=0.0174532925199):
                return math.sqrt(p[0] ** 2 + p[1] ** 2 + p[2] ** 2)
        return float('inf')

if __name__ == '__main__':
    try:
        lidar_parser = LidarParser()
    except rospy.ROSInterruptException:
        pass
