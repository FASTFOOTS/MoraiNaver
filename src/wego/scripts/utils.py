#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import rospkg
from nav_msgs.msg import Path,Odometry
from geometry_msgs.msg import PoseStamped,Point
from std_msgs.msg import Float64,Int16,Float32MultiArray
import numpy as np
from math import cos,sin,sqrt,pow,atan2,pi
import tf


class pathReader :
    def __init__(self,pkg_name):
        rospack=rospkg.RosPack()
        self.file_path=rospack.get_path(pkg_name)

    def read_txt(self,file_name):
        full_file_name=self.file_path+"/path/"+file_name
        openFile = open(full_file_name, 'r')
        out_path=Path()
        
        out_path.header.frame_id='map'
        line=openFile.readlines()
        for i in line :
            tmp=i.split()
            read_pose=PoseStamped()
            read_pose.pose.position.x=float(tmp[0])
            read_pose.pose.position.y=float(tmp[1])
            read_pose.pose.position.z=float(tmp[2])
            read_pose.pose.orientation.x=0
            read_pose.pose.orientation.y=0
            read_pose.pose.orientation.z=0
            read_pose.pose.orientation.w=1
            out_path.poses.append(read_pose)
        
        openFile.close()
        return out_path
      

def findLocalPath(ref_path,status_msg):
    out_path=Path()
    current_x=status_msg.position.x
    current_y=status_msg.position.y
    current_waypoint=0
    min_dis=float('inf')

    for i in range(len(ref_path.poses)) :
        dx=current_x - ref_path.poses[i].pose.position.x
        dy=current_y - ref_path.poses[i].pose.position.y
        dis=sqrt(dx*dx + dy*dy)
        if dis < min_dis :
            min_dis=dis
            current_waypoint=i

    if current_waypoint+50 > len(ref_path.poses) :
        last_local_waypoint= len(ref_path.poses)
    else :
        last_local_waypoint=current_waypoint+50

    out_path.header.frame_id='map'
    for i in range(current_waypoint,last_local_waypoint) :
        tmp_pose=PoseStamped()
        tmp_pose.pose.position.x=ref_path.poses[i].pose.position.x
        tmp_pose.pose.position.y=ref_path.poses[i].pose.position.y
        tmp_pose.pose.position.z=ref_path.poses[i].pose.position.z
        tmp_pose.pose.orientation.x=0
        tmp_pose.pose.orientation.y=0
        tmp_pose.pose.orientation.z=0
        tmp_pose.pose.orientation.w=1
        out_path.poses.append(tmp_pose)

    return out_path,current_waypoint

class velocityPlanning :
    def __init__(self,car_max_speed,road_friction):
        self.car_max_speed=car_max_speed
        self.road_friction=road_friction
 
    def curveBasedVelocity(self,global_path,point_num):
        out_vel_plan=[]
        for i in range(0,point_num):
            out_vel_plan.append(self.car_max_speed)

        for i in range(point_num,len(global_path.poses)-point_num):
            x_list=[]
            y_list=[]
            for box in  range(-point_num,point_num):
                x=global_path.poses[i+box].pose.position.x
                y=global_path.poses[i+box].pose.position.y
                x_list.append([-2*x,-2*y,1])
                y_list.append(-(x*x)-(y*y))
            
            x_matrix=np.array(x_list)
            y_matrix=np.array(y_list)
            x_trans=x_matrix.T
            

            a_matrix=np.linalg.inv(x_trans.dot(x_matrix)).dot(x_trans).dot(y_matrix)
            a=a_matrix[0]
            b=a_matrix[1]
            c=a_matrix[2]
            r=sqrt(a*a+b*b-c)
            v_max=sqrt(r*9.8*self.road_friction)  #0.7
            if v_max>self.car_max_speed :
                v_max=self.car_max_speed
            out_vel_plan.append(v_max)

        for i in range(len(global_path.poses)-point_num,len(global_path.poses)):
            out_vel_plan.append(self.car_max_speed)
        
        return out_vel_plan

class PotentialField:
    def __init__(self, gain_att, gain_rep):
        self.gain_att = gain_att
        self.gain_rep = gain_rep

    def get_arguments(self, obstacles, goal):
        self.obstacles = obstacles
        self.goal = goal

    def calculate_force(self, current_pose):
        force_att = (
            self.gain_att * (self.goal[0] - current_pose[0]),
            self.gain_att * (self.goal[1] - current_pose[1])
        )
        force_rep = np.zeros_like(current_pose)

        for obstacle in self.obstacles:
            distance = np.linalg.norm((current_pose[0] - obstacle[0], current_pose[1] - obstacle[1]))
            if distance < 5.0:
                force_rep += (
                    self.gain_rep * (1.0 / distance - 1.0 / 5.0) * (1.0 / distance ** 2) * (current_pose[0] - obstacle[0]),
                    self.gain_rep * (1.0 / distance - 1.0 / 5.0) * (1.0 / distance ** 2) * (current_pose[1] - obstacle[1])
                )

        total_force = (
            force_att[0] + force_rep[0],
            force_att[1] + force_rep[1]
        )
        return total_force
    def convert_force_to_controls(self, force):
        # Constants for your car's dynamics
        max_velocity = 90.0  # Maximum linear velocity (m/s)
        max_steering_angle = 50.0  # Maximum steering angle (degrees)
        car_length = 1.0  # Length of the car (meters)
        steering_ratio = 11.0  # Steering ratio

        # Calculate desired linear velocity based on the force
        desired_velocity = np.linalg.norm(force)

        # Ensure linear velocity does not exceed the maximum
        if desired_velocity > max_velocity:
            desired_velocity = max_velocity

        # Calculate desired steering angle based on the force
        desired_steering_angle = np.arctan2(2.0 * car_length * np.sin(steering_ratio) * force[1], car_length)

        # Convert desired steering angle from radians to degrees
        desired_steering_angle_degrees = np.degrees(desired_steering_angle)

        # Ensure steering angle does not exceed the maximum
        if desired_steering_angle_degrees > max_steering_angle:
            desired_steering_angle_degrees = max_steering_angle
        elif desired_steering_angle_degrees < -max_steering_angle:
            desired_steering_angle_degrees = -max_steering_angle
        desired_steering_angle_degrees /= 100
        desired_velocity *= 1

        return desired_velocity, desired_steering_angle_degrees


class pidController : ## 속도 제어를 위한 PID 적용 ##
    def __init__(self):
        self.p_gain=1.0
        self.i_gain=0.0
        self.d_gain=0.5
        self.controlTime=0.033
        self.prev_error=0
        self.i_control=0


    def pid(self,target_vel,current_vel):
        error= target_vel-current_vel
        
        p_control=self.p_gain*error
        self.i_control+=self.i_gain*error*self.controlTime
        d_control=self.d_gain*(error-self.prev_error)/self.controlTime

        output=p_control+self.i_control+d_control
        self.prev_error=error
        return output

