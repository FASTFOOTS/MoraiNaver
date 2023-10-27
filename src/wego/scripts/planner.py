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
        self.image_sub = rospy.Subscriber("/segmantic_image_jpeg/compressed", CompressedImage, self.img_CB)
        self.object_info_timer = rospy.Timer(rospy.Duration(0.15), self.timerCB)

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
        rate = rospy.Rate(10) # 30hz

        while not rospy.is_shutdown():
            # print(self.is_status , self.is_imu ,self.is_gps)
            if self.is_status == True and self.is_imu == True and self.is_gps == True and self.is_obj == True:
                self.getScoutStatus()
                ## global_path와 차량의 status_msg를 이용해 현제 waypoint와 local_path를 생성
                local_path,self.current_waypoint=findLocalPath(self.global_path,self.status_msg) 
                
                ## 장애물의 숫자와 Type 위치 속도 (object_num, object type, object pose_x, object pose_y, object velocity)
                # self.vo.get_object(self.object_num,self.object_info_msg[0],self.object_info_msg[1],self.object_info_msg[2],self.object_info_msg[3])
                # global_obj,local_obj=self.vo.calc_vaild_obj([self.status_msg.position.x,self.status_msg.position.y,(self.status_msg.heading)/180*pi])

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


                pure_pursuit.getPath(local_path) ## pure_pursuit 알고리즘에 Local path 적용
                pure_pursuit.getEgoStatus(self.status_msg) ## pure_pursuit 알고리즘에 차량의 status 적용
                
                ctrl_msg.steering=-pure_pursuit.steering_angle()
                
                if abs(self.wheel_angle) > 4.5:
                    if abs(self.wheel_angle) > 8.0:
                        target_velocity = vel_profile_50[self.current_waypoint]
                    else:
                        target_velocity = vel_profile_50[self.current_waypoint]
                else:
                    if self.obstacle_detected:
                        target_velocity = vel_profile_50[self.current_waypoint]
                    else:
                        if (0 < self.current_waypoint < 240):
                            target_velocity = vel_profile_50[self.current_waypoint]
                        elif(650 < self.current_waypoint <  778) or \
                        (1330 < self.current_waypoint < 1450) or \
                        (2056 < self.current_waypoint < 2150) or \
                        (2780 < self.current_waypoint < 2921) or \
                        (3530 < self.current_waypoint < 3630) or \
                        (4200 < self.current_waypoint < 4300) or \
                        (5090 < self.current_waypoint < 5160) or \
                        (5650 < self.current_waypoint < 5990):
                            target_velocity = vel_profile_50[self.current_waypoint]
                        elif (778 < self.current_waypoint < 1300) or \
                            (2150 < self.current_waypoint < 2699) or \
                            (2921 < self.current_waypoint < 3300) or \
                            (3800 < self.current_waypoint < 4190) or \
                            (5160 < self.current_waypoint < 5600):
                                target_velocity = vel_profile_70[self.current_waypoint]
                        else:
                            target_velocity = vel_profile_50[self.current_waypoint]
                    
                    if abs(ctrl_msg.steering) > 4.5:
                        target_velocity = vel_profile_50[self.current_waypoint]

                    if abs(ctrl_msg.steering) > 8.0:
                        target_velocity = vel_profile_50[self.current_waypoint]
                        
                # print(f"target_velocity : {round(target_velocity * 3.6, 0)}")

                # if self.velocity > 25 and ctrl_msg.steering > 3.5:
                #     target_velocity = vel_profile_10[self.current_waypoint]
                
                # target_velocity = vel_profile[self.current_waypoint]

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
                self.print_info()
            
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
        min_dist = float("inf")
        min_idx = 0
        x = []
        y = []
        filtered_x = []
        filtered_y = []
        filtered_x2 = []
        filtered_y2 = []
        index_list = []
        index_list2 = []
        list1 = []
        scan_ranges = [round(data,1) for data in data.ranges]

        angle_min = data.angle_min  # 첫 번째 스캔 데이터의 각도
        angle_increment = data.angle_increment  # 각 스캔 데이터 사이의 각도 간격

        for i, range_val in enumerate(scan_ranges):
            if not isinf(range_val) and not isnan(range_val):
                # Calculate the angle for this range value
                angle = angle_min + i * angle_increment

                # Convert polar coordinates to Cartesian coordinates
                x_val = range_val * cos(angle)
                y_val = range_val * sin(angle)

                x.append(x_val)
                y.append(y_val)
        
        for index, data in enumerate(y):
            if abs(data) < 2.5:
                filtered_y.append(data)
                index_list.append(index)
        for i in index_list:
            filtered_x.append(x[i])

        for index, data in enumerate(filtered_x):
            if 1 < data < 10:
                filtered_x2.append(data)
                index_list2.append(index)
        for i in index_list2:
            filtered_y2.append(filtered_y[i])
        
        self.object_info_msg=[[1,1000,1000,0]]
        self.obstacle_detected = False
        
        if not ((5681 < self.current_waypoint < 5915) and (1400 < self.current_waypoint < 2150) and (4257 < self.current_waypoint < 5200)):
            # self.object_info_msg=[[1,min_dist_x + self.status_msg.position.x,min_dist_y + self.status_msg.position.y,0]]/
            # self.object_info_msg = [1,closest_x, closest_y,0]
            for i in range(len(filtered_x2)):
                list1.append([1, filtered_x2[i], filtered_y2[i], 0])
            self.object_info_msg = list1


            self.obstacle_detected = True

        self.is_obj=True

    def detect_color(self, img):
        # Convert to HSV color space
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        # Define range of yellow color in HSV
        yellow_lower = np.array([18, 100, 100])
        yellow_upper = np.array([32, 255, 255])

        # Threshold the HSV image to get only yellow colors
        yellow_mask = cv2.inRange(hsv, yellow_lower, yellow_upper)
        yellow_color = cv2.bitwise_and(img, img, mask=yellow_mask)
        return yellow_color

    def img_binary(self, blend_line):
        bin = cv2.cvtColor(blend_line, cv2.COLOR_BGR2GRAY)
        binary_line = np.zeros_like(bin)
        binary_line[bin >127] = 1
        return binary_line
    
    def detect_obj(self, bin_img):
        bottom_half_y = bin_img.shape[0] * 2 / 3
        histogram = np.sum(bin_img[int(bottom_half_y) :, :], axis=0)
        histogram[histogram < 25] = 0
        print(f"histogram : {histogram}")

        obj_base = np.argmax(histogram)
        print(f"obj_base : {obj_base}")

        return obj_base

    def img_CB(self, data):
        img = self.bridge.compressed_imgmsg_to_cv2(data)
        yellow_obj = self.detect_color(img)
        bin_img = self.img_binary(yellow_obj)
        detected_obj = self.detect_obj(bin_img)
        obj_y_value = (320 - detected_obj) / 20
        
        if 200 < detected_obj < 500:
            self.object_info_msg.append([1,5,obj_y_value,0])
            self.obstacle_detected = True
            print(self.object_info_msg)
        else:
            print("noting detected by semantic camera")

    def timerCB(self, event):
        pass

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