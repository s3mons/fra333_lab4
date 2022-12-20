#!/usr/bin/python3
import math
from re import S
import numpy as np
from numpy import ones, zeros
import rclpy
import json
import sys, os, yaml
from rclpy.node import Node
from builtin_interfaces.msg import Duration
from trajectory_msgs.msg import JointTrajectory , JointTrajectoryPoint
from romeona_interfaces.srv import EnableTrack
from sensor_msgs.msg import Imu , JointState
from std_msgs.msg import Bool,Float64MultiArray

class Tracker(Node):

    def __init__(self):
        super().__init__('tracker')
        
        # service for test
        self.Is_Enable = self.create_service(EnableTrack,'enable',self.is_enable_callback)

        # subscription
        self.joint_sub = self.create_subscription(JointState,'/joint_states',self.joint_states_callback,10)
        self.ref_joint_sub = self.create_subscription(JointTrajectoryPoint,'/reference/joint_states',self.Ref_joint_states_callback,10)
        self.final_point = self.create_subscription(Bool,'/enableTracker',self.finished_callback,10)


        # create publisher and publishtopic to gazebo
        publish_topic = "/velocity_controllers/commands"
        self.Joint_state_pub = self.create_publisher(Float64MultiArray,publish_topic,10)

        # timer
        self.timer_period = 0.1
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        # # user variables
        self.last_point = False
        self.enable = True
        self.q = np.array([0.0,0.0,0.0])
        self.qr = np.array([0.0,0.0,0.0])
        self.qr_dot = np.array([0.0,0.0,0.0])
        self.intigral = np.array([0.0,0.0,0.0])


    def joint_states_callback(self,msg):
        self.q = np.array(msg.position)
    
    def Ref_joint_states_callback(self,msg):
        self.qr = np.array(msg.positions)
        self.qr_dot = np.array(msg.velocities)

    def finished_callback(self, msg):
        self.last_point = msg.data

    def is_enable_callback(self,request,response):
        self.enable_state = request.enable.data
        # self.goal_positions[0] = round(request.g[0],2)
        # self.goal_positions[1] = round(request.g[1],2)
        # self.goal_positions[2] = round(request.g[2],2)
        
        # return responsee
        pass

    
    def timer_callback(self):
        Kp = 0.5
        Ki = 0.01
        if self.enable == True and self.last_point == False:
            self.intigral += self.qr - self.q
            u = self.qr_dot + np.dot(Kp,(self.qr-self.q))+np.dot(Ki,self.intigral)
            response = u.tolist()
            Trajectory_Tracker = Float64MultiArray()
            Trajectory_Tracker.data = response
            self.Joint_state_pub.publish(Trajectory_Tracker)
            
        else:
            response = [ 0.0 , 0.0 , 0.0 ]
            print('finish')
            pass
       
 
def main(args=None):
    rclpy.init(args=args)
    joint_trajectory_object = Tracker()
    try:
        while rclpy.ok():
            rclpy.spin_once(joint_trajectory_object)
    except KeyboardInterrupt:
        print('repeater stopped cleanly')
    except BaseException:
        print('exception in repeater:', file=sys.stderr)
        raise
    finally:
        joint_trajectory_object.destroy_node()
        rclpy.shutdown() 


if __name__ == '__main__':
    main()
