#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import sys
import rospy
import rospkg
from morai_msgs.msg  import CtrlCmd, EgoVehicleStatus
from sensor_msgs.msg import LaserScan
from math import *
import numpy as np
import matplotlib.pyplot as plt

class FollowGap():
    def __init__(self):
        rospy.init_node('follow_the_gap', anonymous=True)
        self.ctrl_pub = rospy.Publisher('/ctrl_cmd',CtrlCmd, queue_size=1) ## Vehicl Control
        self.scan_sub = rospy.Subscriber("/scan", LaserScan, self.lidar_callback, queue_size = 1)
        self.ego_sub = rospy.Subscriber("/Ego_topic",EgoVehicleStatus, self.statusCB, queue_size = 1)
        self.ctrl_msg= CtrlCmd()

    def lidar_callback(self,data):
        x = []
        y = []
        safty_bubble_idx = []
        mid_point = []
        rb = 1
        ranges = list(data.ranges)
        limit_distance = 50
        step = 15
        
        del ranges[:224]
        del ranges[480:]
        scan_ranges = [round(data,3) for data in ranges]        

        min_range_point_idx = ranges.index(min(ranges))

        

        angle_min = data.angle_min  # 첫 번째 스캔 데이터의 각도
        angle_increment = data.angle_increment  # 각 스캔 데이터 사이의 각도 간격

        for i, range_val in enumerate(scan_ranges):
            if not isnan(range_val):
                # Calculate the angle for this range value
                angle = angle_min + i * angle_increment

                # Convert polar coordinates to Cartesian coordinates
                x_val = range_val * cos(angle)
                y_val = range_val * sin(angle)

                x.append(x_val)
                y.append(y_val)
        
        min_point = (x[min_range_point_idx], y[min_range_point_idx])
        # print("============after 0==============")
        for idx, val in enumerate(zip(x, y)):
            if (hypot(val[0] - min_point[0], val[1] - min_point[1]) < rb):
                scan_ranges[idx] = 0

        print(scan_ranges)
        print("=====//=====================")

        sublists = self.find_sublists_with_threshold(scan_ranges, limit_distance, step)
        print(sublists)



        for i, (start, end) in enumerate(sublists):
            print(f"부분 {i + 1}: 시작 인덱스 {start}, 끝 인덱스 {end}")
            length = (end - start)/2
            mid_point.append(int(start + length))

        # print(mid_point[-1])
        velocity = 50 / 3.6
        self.ctrl_msg.steering = -(224 - mid_point[0]) * 90 / 224 * 0.01

        # speed control method
        if velocity > self.velocity:
            self.ctrl_msg.accel = 1
            self.ctrl_msg.brake = 0
        else: # velocity <= self.velocity:
            self.ctrl_msg.accel = 0
            self.ctrl_msg.brake = 0
            if self.velocity > velocity + 5:
                self.ctrl_msg.accel = 0
                self.ctrl_msg.brake = 1

        # # Anti-Lock Braking System 
        # if (self.ctrl_msg.brake == 1) and (self.wheel_angle > 0.5):
        #     if abs_flag:
        #         self.ctrl_msg.brake = 0
        #         abs_flag = False
        #     else:
        #         self.ctrl_msg.brake = 1
        #         abs_flag = True



        self.ctrl_pub.publish(self.ctrl_msg)

        



    def find_sublists_with_threshold(self, lst, threshold, n):
        sublists = []
        current_count = 0
        start = None

        for i, value in enumerate(lst):
            if value >= threshold:
                if start is None:
                    start = i
                current_count += 1
            else:
                if current_count >= n:
                    sublists.append((start, i - 1))
                start = None
                current_count = 0

        # Check if the last sublist extends to the end of the list
        if current_count >= n:
            sublists.append((start, len(lst) - 1))

        return sublists
    
    def statusCB(self, data):
        self.velocity = data.velocity.x
        self.wheel_angle = data.wheel_angle
        self.is_status = True
        
    





if __name__ == "__main__":

    follow_gap = FollowGap()
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass