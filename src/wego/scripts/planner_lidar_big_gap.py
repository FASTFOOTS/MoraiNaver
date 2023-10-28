#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import sys,os
import rospy
import rospkg
import numpy as np
from nav_msgs.msg import Path,Odometry
from std_msgs.msg import Float64,Int16,Float32MultiArray
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import PoseStamped,Point,Twist
from sensor_msgs.msg import LaserScan
from utils import *
import tf
from math import cos,sin,sqrt,pow,atan2,pi, isinf, isnan
from morai_msgs.msg import GPSMessage, EgoVehicleStatus, CtrlCmd
import pyproj
from sensor_msgs.msg import Imu, CompressedImage, Image
from cv_bridge import CvBridge
import cv2

class ego_status:
    def __init__(self):
        self.position = Vector3()
        self.heading = 0.0
        self.velocity = Vector3()


class gen_planner():
    def __init__(self):
        self.bridge = CvBridge()
        rospy.init_node('gen_planner', anonymous=True)

        arg = rospy.myargv(argv=sys.argv)
        self.path_name=arg[1]
        self.x_offset=float(arg[2])
        self.y_offset=float(arg[3])

        self.scan_angle = np.linspace(0,360,723)
        self.min_angle = 0.0
        self.min_dist = 0.0

        self.steering = 0
        self.steering_angle = [0,0]

        self.object_info_msg=[[1,1000,1000,0]]
        self.obstacle_detected = False

        self.current_waypoint = 0
        

        abs_flag = True
        
        path_reader=pathReader('wego') ## 경로 파일의 위치

        #publisher
        global_path_pub= rospy.Publisher('/global_path',Path, queue_size=1) ## global_path publisher
        local_path_pub= rospy.Publisher('/local_path',Path, queue_size=1) ## local_path publisher
        ctrl_pub = rospy.Publisher('/ctrl_cmd',CtrlCmd, queue_size=1) ## Vehicl Control
        odom_pub = rospy.Publisher('odom',Odometry, queue_size=1)
        # obj_pub = rospy.Publisher('/semantic_obstacle/compressed', CompressedImage, queue_size=1)
        ctrl_msg= CtrlCmd()
        self.status_msg = ego_status()

        

        ########################  lattice   ########################
        for i in range(1,22):            
            globals()['lattice_path_{}_pub'.format(i)]=rospy.Publisher('lattice_path_{}'.format(i),Path,queue_size=1)  

        ########################  lattice   ########################

    
        #subscriber
        rospy.Subscriber("/gps", GPSMessage, self.gpsCB)
        self.imu_sub = rospy.Subscriber("/imu", Imu, self.imuCB, queue_size=1)
        self.ego_sub = rospy.Subscriber("/Ego_topic",EgoVehicleStatus, self.statusCB, queue_size = 1)
        self.scan_sub = rospy.Subscriber("/scan", LaserScan, self.scanCB, queue_size = 1)


        #def
        self.is_status = False
        self.is_gps = False
        self.is_imu = False

        #class
        
        pure_pursuit=purePursuit() ## purePursuit import
        self.proj_UTM = pyproj.Proj(proj='utm', zone=52, ellps='WGS84', preserve_units=False)

        self.global_path = path_reader.read_txt(self.path_name)

        normal_velocity_30 = 30/3.6 # km/h -> m/s
        vel_planner_30=velocityPlanning(normal_velocity_30,0.15) ## 속도 계획
        vel_profile_30=vel_planner_30.curveBasedVelocity(self.global_path,100)

        normal_velocity_10 = 10/3.6 # km/h -> m/s
        vel_planner_10=velocityPlanning(normal_velocity_10,0.15) ## 속도 계획
        vel_profile_10=vel_planner_10.curveBasedVelocity(self.global_path,100)

        normal_velocity_50 = 50/3.6 # km/h -> m/s
        vel_planner_50=velocityPlanning(normal_velocity_50,0.15) ## 속도 계획
        vel_profile_50=vel_planner_50.curveBasedVelocity(self.global_path,100)

        normal_velocity_70 = 70/3.6 # km/h -> m/s
        vel_planner_70=velocityPlanning(normal_velocity_70,0.15) ## 속도 계획
        vel_profile_70=vel_planner_70.curveBasedVelocity(self.global_path,100)

        normal_velocity_90 = 90/3.6 # km/h -> m/s
        vel_planner_90=velocityPlanning(normal_velocity_90,0.15) ## 속도 계획
        vel_profile_90=vel_planner_90.curveBasedVelocity(self.global_path,100)

        normal_velocity_100 = 100/3.6 # km/h -> m/s
        vel_planner_100=velocityPlanning(normal_velocity_100,0.15) ## 속도 계획
        vel_profile_100=vel_planner_100.curveBasedVelocity(self.global_path,100)
        

        

        lattice_current_lane=3

    
        #time var
        count=0
        rate = rospy.Rate(50) # 30hz

        while not rospy.is_shutdown():
            # print(self.is_status , self.is_imu ,self.is_gps)
            if self.is_status == True and self.is_imu == True and self.is_gps == True: # and self.is_obj == True:
                self.getScoutStatus()
                ## global_path와 차량의 status_msg를 이용해 현제 waypoint와 local_path를 생성
                local_path,self.current_waypoint=findLocalPath(self.global_path,self.status_msg) 
 
                ########################  lattice  ########################
                vehicle_status=[self.status_msg.position.x,self.status_msg.position.y,(self.status_msg.heading)/180*pi,self.status_msg.velocity.x]
                # print(f"obj : {self.object_info_msg}")
                lattice_path,selected_lane=latticePlanner(local_path,self.object_info_msg,vehicle_status,lattice_current_lane)
                lattice_current_lane=selected_lane
                # print(f"lattice_current_lane : {lattice_current_lane}")
                # print(f"selected_lane : {selected_lane}")
                                
                if selected_lane != -1: 
                    local_path=lattice_path[selected_lane]                
                
                if len(lattice_path)==21:                  
                    for i in range(1,22):
                        globals()['lattice_path_{}_pub'.format(i)].publish(lattice_path[i-1])
                ########################  lattice  ########################
# (630< self.current_waypoint < 700) or \

                pure_pursuit.getPath(local_path) ## pure_pursuit 알고리즘에 Local path 적용
                pure_pursuit.getEgoStatus(self.status_msg) ## pure_pursuit 알고리즘에 차량의 status 적용
                if (0< self.current_waypoint < 130) or \
                    (408< self.current_waypoint < 450) or \
                    (1375< self.current_waypoint < 1446) or \
                    (1640< self.current_waypoint < 1730) or \
                    (1862< self.current_waypoint < 1920) or \
                    (2120< self.current_waypoint < 2191) or \
                    (3000< self.current_waypoint < 3076) or \
                    (3877< self.current_waypoint < 3911) or \
                    (4610< self.current_waypoint < 4643):
                    ctrl_msg.steering=self.steering
                    # ctrl_msg.steering=-pure_pursuit.steering_angle()
                    # print("lattice planner")
                else:
                    # print("follow the gap")
                    ctrl_msg.steering=self.steering
                target_velocity = vel_profile_70[self.current_waypoint]
                # if abs(self.wheel_angle) > 4.5:
                #     if abs(self.wheel_angle) > 8.0:
                #         target_velocity = vel_profile_50[self.current_waypoint]
                #     else:
                #         target_velocity = vel_profile_50[self.current_waypoint]
                # else:
                #     target_velocity = vel_profile_50[self.current_waypoint]
                #     if self.obstacle_detected:
                #         target_velocity = vel_profile_70[self.current_waypoint]
                    
                #     if abs(ctrl_msg.steering) > 4.5:
                #         target_velocity = vel_profile_50[self.current_waypoint]

                #     if abs(ctrl_msg.steering) > 8.0:
                #         target_velocity = vel_profile_50[self.current_waypoint]
                        
                if self.velocity > 90:
                    if ctrl_msg.steering > 3:
                        ctrl_msg.steering = 3
                    if ctrl_msg.steering < -3:
                        ctrl_msg.steering = -3
                                        
                # speed control method
                if target_velocity > self.velocity:
                    ctrl_msg.accel = 1
                    ctrl_msg.brake = 0
                else: # target_velocity <= self.velocity:
                    ctrl_msg.accel = 0
                    ctrl_msg.brake = 0
                    if self.velocity > target_velocity + 5:
                        ctrl_msg.accel = 0
                        ctrl_msg.brake = 1

                # Anti-Lock Braking System 
                if (ctrl_msg.brake == 1) and (self.wheel_angle > 0.5):
                    if abs_flag:
                        ctrl_msg.brake = 0
                        abs_flag = False
                    else:
                        ctrl_msg.brake = 1
                        abs_flag = True

                local_path_pub.publish(local_path) ## Local Path 출력
                ctrl_pub.publish(ctrl_msg) ## Vehicl Control 출력
                odom_pub.publish(self.makeOdomMsg())
                # self.print_info()
            
                if count==30 : ## global path 출력
                    global_path_pub.publish(self.global_path)
                    count=0
                count+=1
                rate.sleep()

    def getScoutStatus(self): ## Vehicl Status Subscriber 
        self.status_msg.position.x = self.xy_zone[0] - self.x_offset
        self.status_msg.position.y = self.xy_zone[1] - self.y_offset
        self.status_msg.heading = self.euler_data[2] * 180 / pi
        self.status_msg.velocity.x = self.velocity
        br = tf.TransformBroadcaster()
        br.sendTransform((self.status_msg.position.x, self.status_msg.position.y, self.status_msg.position.z),
                        tf.transformations.quaternion_from_euler(0, 0, (self.status_msg.heading)/180*pi),
                        rospy.Time.now(),
                        "base_link",
                        "odom")
        self.is_status=True

    def statusCB(self, data):
        self.velocity = data.velocity.x
        self.wheel_angle = data.wheel_angle
        self.is_status = True

    def gpsCB(self, data):
        self.xy_zone = self.proj_UTM(data.longitude, data.latitude)
        self.is_gps = True

    def imuCB(self, data):
        self.euler_data = tf.transformations.euler_from_quaternion((data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w))
        self.is_imu = True
    
    def scanCB(self, data):
        limit_distance = 50
        step = 15
        x = []
        y = []
        filtered_x = []
        filtered_y = []
        filtered_x2 = []
        filtered_y2 = []
        index_list = []
        index_list2 = []
        mid_point = []
        length_list = []
        
        scan_ranges = [round(data,5) for data in data.ranges]
        del scan_ranges[:224]
        del scan_ranges[480:]
        
        print("===========================")
        print(scan_ranges)
        sublists = self.find_sublists_with_threshold(scan_ranges, limit_distance, step)
        if sublists:
            # print("step 50")
            pass
        else: # under 15 bundle more than 50 
            sublists = self.find_sublists_with_threshold(scan_ranges, limit_distance, 1) # step = 1
            if sublists:
                # print("step 1")
                pass
            else: # doesn't have more than 50
                # print("no step")
                one_point_index = scan_ranges.index(max(scan_ranges))
                sublists = [(one_point_index, one_point_index+1)] # max value of scan_ranges
        # print(sublists)
        for i, (start, end) in enumerate(sublists):
            # print(f"부분 {i + 1}: 시작 인덱스 {start}, 끝 인덱스 {end}")
            length = (end - start)/2
            mid_point.append(int(start + length))
            length_list.append(length)

        # selected_index = round(len(mid_point)/2) - 1
        selected_index = length_list.index(max(length_list))
        if len(mid_point) > 1:
            self.steering_angle[1] = -(224 - mid_point[selected_index]) * 90 / 224 * 0.0075
        else:
            self.steering_angle[1] = 0

        # if self.steering_angle[1] - self.steering_angle[0] > 1:
        #     self.steering_angle[1] += 1
        # if self.steering_angle[1] - self.steering_angle[0]  < -1:
        #     self.steering_angle[1] -= 1
        
        self.steering = self.steering_angle[1]
        # else:
        #     # print("lattice!")/
        #     self.obstacle_detected = False

        self.is_obj=True

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

    def makeOdomMsg(self):
        odom=Odometry()
        odom.header.frame_id='map'
        odom.child_frame_id='base_link'

        quaternion = tf.transformations.quaternion_from_euler(0, 0, np.deg2rad(self.status_msg.heading))

        odom.pose.pose.position.x=self.status_msg.position.x
        odom.pose.pose.position.y=self.status_msg.position.y
        odom.pose.pose.position.z=self.status_msg.position.z
        odom.pose.pose.orientation.x=quaternion[0]
        odom.pose.pose.orientation.y=quaternion[1]
        odom.pose.pose.orientation.z=quaternion[2]
        odom.pose.pose.orientation.w=quaternion[3]


        return odom

    def print_info(self):

        os.system('clear')
        # print('--------------------status-------------------------')
        # print('position :{0} ,{1}, {2}'.format(self.status_msg.position.x,self.status_msg.position.y,self.status_msg.position.z))
        # print('velocity :{} km/h'.format(self.status_msg.velocity.x))
        # print('heading :{} deg'.format(self.status_msg.heading))

        # print('--------------------object-------------------------')
        # print('object num :{}'.format(self.object_num))
        # for i in range(0,self.object_num) :
        #     print('{0} : type = {1}, x = {2}, y = {3}, z = {4} '.format(i,self.object_info_msg[0][i],self.object_info_msg[1][i],self.object_info_msg[2][i],self.object_info_msg[3][i]))

        print('--------------------localization-------------------------')
        print('all waypoint size: {} '.format(len(self.global_path.poses)))
        print('current waypoint : {} '.format(self.current_waypoint))

 
if __name__ == '__main__':
    try:
        kcity_pathtracking=gen_planner()
    except rospy.ROSInterruptException:
        pass