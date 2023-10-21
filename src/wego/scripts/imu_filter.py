#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# import rospy
# from sensor_msgs.msg import Imu
# from geometry_msgs.msg import Vector3

# class RoundedIMU:
#     def __init__(self):
#         rospy.init_node('rounded_imu_publisher')
        
#         # Create a publisher for the rounded IMU values
#         self.rounded_imu_pub = rospy.Publisher('/imu', Imu, queue_size=10)
        
#         # Subscribe to the original IMU topic
#         rospy.Subscriber("/imu", Imu, self.imu_callback)

#     def imu_callback(self, imu_msg):
#         # Round the values to two decimal places
#         imu_msg.orientation.x = round(imu_msg.orientation.x, 2)
#         imu_msg.orientation.y = round(imu_msg.orientation.y, 2)
#         imu_msg.orientation.z = round(imu_msg.orientation.z, 2)
#         imu_msg.orientation.w = round(imu_msg.orientation.w, 2)
#         imu_msg.angular_velocity.x = round(imu_msg.angular_velocity.x, 2)
#         imu_msg.angular_velocity.y = round(imu_msg.angular_velocity.y, 2)
#         imu_msg.angular_velocity.z = round(imu_msg.angular_velocity.z, 2)
#         imu_msg.linear_acceleration.x = round(imu_msg.linear_acceleration.x, 2)
#         imu_msg.linear_acceleration.y = round(imu_msg.linear_acceleration.y, 2)
#         imu_msg.linear_acceleration.z = round(imu_msg.linear_acceleration.z, 2)

#         # Publish the rounded IMU message
#         self.rounded_imu_pub.publish(imu_msg)

# if __name__ == '__main__':
#     try:
#         rounded_imu_node = RoundedIMU()
#         rospy.spin()
#     except rospy.ROSInterruptException:
#         pass



import rospy
from sensor_msgs.msg import Imu

class RoundedIMU:
    def __init__(self):
        rospy.init_node('rounded_imu_publisher')
        
        # Create a publisher for the rounded IMU values
        self.rounded_imu_pub = rospy.Publisher('/rounded_imu', Imu, queue_size=50)
        
        # Subscribe to the original IMU topic
        rospy.Subscriber("/imu", Imu, self.imu_callback)

    def imu_callback(self, imu_msg):
        # Create a new IMU message
        rounded_imu_msg = Imu()

        # Copy the header from the original message
        rounded_imu_msg.header = imu_msg.header

        # Round the orientation, angular velocity, and linear acceleration values
        rounded_imu_msg.orientation.x = round(imu_msg.orientation.x, 2)
        rounded_imu_msg.orientation.y = round(imu_msg.orientation.y, 2)
        rounded_imu_msg.orientation.z = round(imu_msg.orientation.z, 2)
        rounded_imu_msg.orientation.w = round(imu_msg.orientation.w, 2)

        rounded_imu_msg.angular_velocity.x = round(imu_msg.angular_velocity.x, 2)
        rounded_imu_msg.angular_velocity.y = round(imu_msg.angular_velocity.y, 2)
        rounded_imu_msg.angular_velocity.z = round(imu_msg.angular_velocity.z, 2)

        rounded_imu_msg.linear_acceleration.x = round(imu_msg.linear_acceleration.x, 2)
        rounded_imu_msg.linear_acceleration.y = round(imu_msg.linear_acceleration.y, 2)
        rounded_imu_msg.linear_acceleration.z = round(imu_msg.linear_acceleration.z, 2)

        # Publish the rounded IMU message
        self.rounded_imu_pub.publish(rounded_imu_msg)

if __name__ == '__main__':
    try:
        rounded_imu_node = RoundedIMU()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass