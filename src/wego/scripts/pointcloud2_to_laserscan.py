import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, LaserScan
import math

def pointcloud_to_laserscan(pointcloud_msg, laser_frame_id, min_height, max_height):
    # Laser scan parameters
    angle_min = -math.pi
    angle_max = math.pi
    angle_increment = 0.005
    range_min = 0.0
    range_max = 130.0

    # Create a LaserScan message
    laserscan_msg = LaserScan()
    laserscan_msg.header = pointcloud_msg.header
    laserscan_msg.header.frame_id = laser_frame_id
    laserscan_msg.angle_min = angle_min
    laserscan_msg.angle_max = angle_max
    laserscan_msg.angle_increment = angle_increment
    laserscan_msg.range_min = range_min
    laserscan_msg.range_max = range_max

    # Process the point cloud data
    pc_data = pc2.read_points(pointcloud_msg, field_names=("x", "y", "z"), skip_nans=True)
    
    ranges = [float('inf')] * int((angle_max - angle_min) / angle_increment)

    for x, y, z in pc_data:
        if min_height <= z <= max_height:
            # Calculate the angle of the point
            angle = math.atan2(y, x)
            if angle < angle_max and angle > angle_min:
                index = int((angle - angle_min) / angle_increment)
                range_val = (x ** 2 + y ** 2) ** 0.5
                if range_val < ranges[index]:
                    ranges[index] = range_val

    laserscan_msg.ranges = ranges

    return laserscan_msg

def pointcloud_callback(msg):
    laserscan_msg = pointcloud_to_laserscan(msg, laser_frame_id, min_height, max_height)
    pub.publish(laserscan_msg)

if __name__ == '__main__':
    rospy.init_node('pointcloud_to_laserscan_node')

    # Input PointCloud2 topic and LaserScan topic
    input_topic = '/velodyne_points'
    output_topic = '/scan'  # New LaserScan topic
    laser_frame_id = 'base_link'  # Replace with your desired frame ID
    min_height = 0.1  # Set your desired min height
    max_height = 1.0  # Set your desired max height

    pub = rospy.Publisher(output_topic, LaserScan, queue_size=10)
    sub = rospy.Subscriber(input_topic, PointCloud2, pointcloud_callback)

    rospy.spin()
