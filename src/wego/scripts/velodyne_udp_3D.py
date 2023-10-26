import rospy
import std_msgs.msg
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
import socket
import struct

localIP = "127.0.0.1"
localPort = 2368
bufferSize = 1024

# Initialize ROS node
rospy.init_node('velodyne_publisher', anonymous=True)

# Create a publisher for the /lidar3D topic
lidar_pub = rospy.Publisher('/lidar3D', PointCloud2, queue_size=10)

# Define the PointCloud2 message
header = std_msgs.msg.Header()
header.stamp = rospy.Time.now()
header.frame_id = "base_link"  # Adjust the frame_id as needed

fields = [
    PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    # Add more fields as needed for additional point attributes
]

# Create an empty list for the point data
point_data = []

cloud_msg = pc2.create_cloud(header, fields, point_data)

UDPServerSocket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
UDPServerSocket.bind((localIP, localPort))

# Initialize a buffer to accumulate incoming data
data_buffer = b''

while not rospy.is_shutdown():
    bytesAddressPair = UDPServerSocket.recvfrom(bufferSize)
    message = bytesAddressPair[0]

    # Add the received message to the data buffer
    data_buffer += message

    # Process complete points
    while len(data_buffer) >= 12:
        x, y, z = struct.unpack('fff', data_buffer[:12])
        point_data.append((x, y, z))
        data_buffer = data_buffer[12:]  # Remove processed data

    # Update the PointCloud2 message
    cloud_msg = pc2.create_cloud(header, fields, point_data)

    # Publish the PointCloud2 message
    lidar_pub.publish(cloud_msg)

# Close the socket if the program is interrupted
UDPServerSocket.close()
