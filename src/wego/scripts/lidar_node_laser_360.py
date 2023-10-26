#!/usr/bin/env python3
  
import rospy
from sensor_msgs.msg import LaserScan, PointCloud2
import sensor_msgs.point_cloud2 as pc2
from geometry_msgs.msg import Point32

class lidarPaser:
    def __init__(self):
        rospy.init_node('lidar', anonymous=True)
    
        rospy.Subscriber("/velodyne_points", PointCloud2, self.callback)
        self.scan_pub = rospy.Publisher('/scan', LaserScan, queue_size=1)
        
        
        rospy.spin()

    def callback(self, data):
        scan_msg = LaserScan()
        scan_msg.header = data.header
        scan_msg.angle_min = -3.14159265359  # 최소 각도 (라디안)
        scan_msg.angle_max = 3.14159265359   # 최대 각도 (라디안)
        scan_msg.angle_increment = 0.00872664626  # 각도 증가량 (약 0.5도)
        scan_msg.time_increment = 0.0
        scan_msg.scan_time = 0.1  # 10Hz 스캔 속도를 가정
        scan_msg.range_min = 0.0   # 최소 거리 (미터)
        scan_msg.range_max = 100.0  # 최대 거리 (미터)

        scan_msg.ranges = []
        for p in pc2.read_points(data, field_names=("x", "y", "z"), skip_nans=True):
            # 각 점의 거리 계산
            distance = (p[0] ** 2 + p[1] ** 2 + p[2] ** 2) ** 0.5
            scan_msg.ranges.append(distance)

        scan_msg.header.frame_id=data.header.frame_id
        scan_msg.header.stamp=data.header.stamp
        self.scan_pub.publish(scan_msg)
        # print("point num : {}".format(len(pc1_msg.points)))
        
        

if __name__ == '__main__':
    try:
        lidar = lidarPaser()
    except rospy.ROSInterruptException:
        pass