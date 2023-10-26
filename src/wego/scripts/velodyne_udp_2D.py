import rospy
import std_msgs.msg
from sensor_msgs.msg import LaserScan
import socket
import struct

localIP = "127.0.0.1"
localPort = 2368
bufferSize = 1024

# Initialize ROS node
rospy.init_node('velodyne_publisher', anonymous=True)

# Create a publisher for the /scan topic
scan_pub = rospy.Publisher('/scan', LaserScan, queue_size=10)

# Define the LaserScan message
header = std_msgs.msg.Header()
header.stamp = rospy.Time.now()
header.frame_id = "lidar3D"  # Use the provided target_frame

angle_min = -1.6  # Use the provided minimum angle
angle_max = 1.6  # Use the provided maximum angle
angle_increment = 0.0087  # Use the provided angle increment
time_increment = 0.0  # Use the provided time increment
scan_time = 0.0666667  # Use the provided scan time
range_min = 0.1  # Use the provided minimum range
range_max = 200.0  # Use the provided maximum range

# Create empty lists for the LaserScan data
ranges = []
intensities = []

UDPServerSocket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
UDPServerSocket.bind((localIP, localPort))

while not rospy.is_shutdown():
    bytesAddressPair = UDPServerSocket.recvfrom(bufferSize)
    message = bytesAddressPair[0]

    # Parse the message into X, Y data
    point_data = []  # A list of (x, y) points

    # Parse message and extract point data
    for i in range(0, len(message), 8):  # Assuming 8 bytes per point
        x, y = struct.unpack('ff', message[i:i+8])
        point_data.append((x, y))

    # Populate the LaserScan data and ensure range values are within a valid range
    valid_ranges = []
    for x, y in point_data:
        range_val = (x ** 2 + y ** 2) ** 0.5
        if range_min <= range_val <= range_max:
            valid_ranges.append(range_val)
        else:
            valid_ranges.append(range_max)  # Set to the maximum valid range

    # Update the LaserScan message
    scan_msg = LaserScan(
        header=header,
        angle_min=angle_min,
        angle_max=angle_max,
        angle_increment=angle_increment,
        time_increment=time_increment,
        scan_time=scan_time,
        range_min=range_min,
        range_max=range_max,
        ranges=valid_ranges,
        intensities=intensities
    )

    # Publish the LaserScan message
    scan_pub.publish(scan_msg)

# Close the socket if the program is interrupted
UDPServerSocket.close()
