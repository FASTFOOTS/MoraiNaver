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
from math import cos,sin,sqrt,pow,atan2,pi
from morai_msgs.msg import GPSMessage, EgoVehicleStatus, CtrlCmd
import pyproj
from sensor_msgs.msg import Imu


class ego_status:
    def __init__(self):
        self.position = Vector3()
        self.heading = 0.0
        self.velocity = Vector3()


class gen_planner():
    def __init__(self):
        rospy.init_node('gen_planner', anonymous=True)

        arg = rospy.myargv(argv=sys.argv)
        self.path_name=arg[1]
        self.x_offset=float(arg[2])
        self.y_offset=float(arg[3])

        self.scan_angle = np.linspace(0,360,723)
        self.object_info_msg=[[1,1000,1000,0]]
        self.obstacle_detected = False
        self.min_angle = 0.0
        self.min_dist = 0.0
        
        path_reader=pathReader('wego') ## 경로 파일의 위치

        #publisher
        global_path_pub= rospy.Publisher('/global_path',Path, queue_size=1) ## global_path publisher
        local_path_pub= rospy.Publisher('/local_path',Path, queue_size=1) ## local_path publisher
        ctrl_pub = rospy.Publisher('/ctrl_cmd',CtrlCmd, queue_size=1) ## Vehicl Control
        ctrl_msg= CtrlCmd()
        odom_pub = rospy.Publisher('odom',Odometry, queue_size=1)
        self.status_msg = ego_status()
        

        ########################  lattice   ########################
        for i in range(1,8):            
            globals()['lattice_path_{}_pub'.format(i)]=rospy.Publisher('lattice_path_{}'.format(i),Path,queue_size=1)  

        ########################  lattice   ########################

    
        #subscriber
        rospy.Subscriber("/gps", GPSMessage, self.gpsCB)
        self.image_sub = rospy.Subscriber("/imu", Imu, self.imuCB, queue_size=10)
        self.ego_sub = rospy.Subscriber("/Ego_topic",EgoVehicleStatus, self.statusCB, queue_size = 10)
        self.scan_sub = rospy.Subscriber("/scan", LaserScan, self.scanCB, queue_size = 10)

        #def
        self.is_status = False
        self.is_gps = False
        self.is_imu = False

        #class
        
        pure_pursuit=purePursuit() ## purePursuit import
        self.proj_UTM = pyproj.Proj(proj='utm', zone=52, ellps='WGS84', preserve_units=False)

        self.global_path = path_reader.read_txt(self.path_name)

        # normal_velocity_30 = 30/3.6 # km/h -> m/s
        # vel_planner_30=velocityPlanning(normal_velocity_30,0.15) ## 속도 계획
        # vel_profile_30=vel_planner_30.curveBasedVelocity(self.global_path,100)

        normal_velocity_50 = 50/3.6 # km/h -> m/s
        vel_planner_50=velocityPlanning(normal_velocity_50,0.15) ## 속도 계획
        vel_profile_50=vel_planner_50.curveBasedVelocity(self.global_path,100)

        # normal_velocity_70 = 70/3.6 # km/h -> m/s
        # vel_planner_70=velocityPlanning(normal_velocity_70,0.15) ## 속도 계획
        # vel_profile_70=vel_planner_70.curveBasedVelocity(self.global_path,100)

        # normal_velocity_90 = 90/3.6 # km/h -> m/s
        # vel_planner_90=velocityPlanning(normal_velocity_90,0.15) ## 속도 계획
        # vel_profile_90=vel_planner_90.curveBasedVelocity(self.global_path,100)

        # normal_velocity_120 = 120/3.6 # km/h -> m/s
        # vel_planner_120=velocityPlanning(normal_velocity_120,0.15) ## 속도 계획
        # vel_profile_120=vel_planner_120.curveBasedVelocity(self.global_path,100)

        

        lattice_current_lane=3

    
        #time var
        count=0
        rate = rospy.Rate(30) # 30hz

        while not rospy.is_shutdown():
            # print(self.is_status , self.is_imu ,self.is_gps)
            if self.is_status == True and self.is_imu == True and self.is_gps == True: # and self.is_obj == True:
                self.getScoutStatus()
                ## global_path와 차량의 status_msg를 이용해 현제 waypoint와 local_path를 생성
                local_path,self.current_waypoint=findLocalPath(self.global_path,self.status_msg) 
                
                ## 장애물의 숫자와 Type 위치 속도 (object_num, object type, object pose_x, object pose_y, object velocity)
                # self.vo.get_object(self.object_num,self.object_info_msg[0],self.object_info_msg[1],self.object_info_msg[2],self.object_info_msg[3])
                # global_obj,local_obj=self.vo.calc_vaild_obj([self.status_msg.position.x,self.status_msg.position.y,(self.status_msg.heading)/180*pi])

                ########################  lattice  ########################
                vehicle_status=[self.status_msg.position.x,self.status_msg.position.y,(self.status_msg.heading)/180*pi,self.status_msg.velocity.x]
                lattice_path,selected_lane=latticePlanner(local_path,self.object_info_msg,vehicle_status,lattice_current_lane)
                lattice_current_lane=selected_lane
                # print(f"lattice_current_lane : {lattice_current_lane}")
                # print(f"selected_lane : {selected_lane}")
                                
                if selected_lane != -1: 
                    local_path=lattice_path[selected_lane]                
                
                if len(lattice_path)==7:                  
                    for i in range(1,8):
                        globals()['lattice_path_{}_pub'.format(i)].publish(lattice_path[i-1])
                ########################  lattice  ########################


                pure_pursuit.getPath(local_path) ## pure_pursuit 알고리즘에 Local path 적용
                pure_pursuit.getEgoStatus(self.status_msg) ## pure_pursuit 알고리즘에 차량의 status 적용
                
                ctrl_msg.steering=-pure_pursuit.steering_angle()

                if abs(ctrl_msg.steering) > 4.5:
                    target_velocity = vel_profile_50[self.current_waypoint]
                else:
                    if self.obstacle_detected:
                        target_velocity = vel_profile_50[self.current_waypoint]
                    else:
                        if (0 < self.current_waypoint < 240) or \
                        (650 < self.current_waypoint <  778) or \
                        (1330 < self.current_waypoint < 1450) or \
                        (2056 < self.current_waypoint < 2150) or \
                        (2780 < self.current_waypoint < 2921) or \
                        (3530 < self.current_waypoint < 3630) or \
                        (4200 < self.current_waypoint < 4300) or \
                        (5090 < self.current_waypoint < 5160) or \
                        (5650 < self.current_waypoint < 5990):
                            target_velocity = vel_profile_50[self.current_waypoint]
                        elif (778 < self.current_waypoint < 1300) or \
                            (2150 < self.current_waypoint < 2700) or \
                            (2921 < self.current_waypoint < 3300) or \
                            (3800 < self.current_waypoint < 4190) or \
                            (5160 < self.current_waypoint < 5600):
                                target_velocity = vel_profile_50[self.current_waypoint]
                        else:
                            target_velocity = vel_profile_50[self.current_waypoint]

                if self.velocity > 50 and self.ctrl_msg.steering > 3.5:
                    target_velocity = vel_profile_30[self.current_waypoint]
                
                # target_velocity = vel_profile[self.current_waypoint]


                if target_velocity > self.velocity:
                    ctrl_msg.accel = 1
                    ctrl_msg.brake = 0
                else: # target_velocity <= self.velocity:
                    ctrl_msg.accel = 0
                    ctrl_msg.brake = 0
                    if self.velocity > target_velocity + 1:
                        ctrl_msg.accel = 0
                        ctrl_msg.brake = 1

                

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
        self.is_status = True

    def gpsCB(self, data):
        self.xy_zone = self.proj_UTM(data.longitude, data.latitude)
        self.is_gps = True

    def imuCB(self, data):
        self.euler_data = tf.transformations.euler_from_quaternion((data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w))
        self.is_imu = True
    


    def scanCB(self, data):
        min_dist = 1000
        min_idx = 0
        scan_ranges = [round(data,1) for data in data.ranges]

        for idx, val in enumerate(scan_ranges): 
            if min_dist > val:
                min_dist = val
                min_idx = idx

        angle_min = data.angle_min  # 첫 번째 스캔 데이터의 각도
        angle_increment = data.angle_increment  # 각 스캔 데이터 사이의 각도 간격
        min_angle = round(angle_min + min_idx * angle_increment,2)  # 최소 거리를 갖는 스캔 데이터의 각도

        min_dist_x = round(min_dist * cos(min_angle),2)  # x 좌표 계산
        min_dist_y = round(min_dist * sin(min_angle),2)  # y 좌표 계산
        print(f"obj x,y : ({min_dist_x},{min_dist_y})")

        if min_dist_x < 15 and abs(min_dist_y) < 2.5:
            self.object_info_msg=[[1,min_dist_x,min_dist_y,0]]
            self.object_num = 1
            self.obstacle_detected = True
        else:
            self.object_info_msg=[[1,1000,1000,0]]
            self.object_num = 0
            self.obstacle_detected = False

        self.is_obj=True

            

            

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