#!/usr/bin/python3
import math
from re import S
import numpy as np
from numpy import ones, zeros
import rclpy
import sys, os, yaml
from rclpy.node import Node
from builtin_interfaces.msg import Duration
from romeona_control.Jacobian import pos_inverse_kinematics, vel_inverse_kinematics
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory , JointTrajectoryPoint
from std_msgs.msg import Int32, Float64MultiArray, Float64, Bool

class Scheduler(Node):

    def __init__(self):
        super().__init__('scheduler')

        # create publisher
        self.p_i = self.create_publisher(Float64MultiArray,'/p_i',10)
        self.p_f = self.create_publisher(Float64MultiArray,'/p_f',10)
        self.T   = self.create_publisher(Float64,'/T',10)
        self.state_complete = self.create_publisher(Bool,'/enableTracker',10)



        # create subscription
        self.hasReached = self.create_subscription(Bool,"/hasReached",self.reached_callback,10)


        # timer
        timer_period = 0.1
        # self.timer = self.create_timer(timer_period, self.timer_callback)

        # variable
        self.reach = False
        self.point_idx = 1
        self.via_point = [[0.5,0.,0.],[0.55,0.,0.], [0.55,0.35,0.],[0.525,0.,0.],[0.525,0.35,0.]]
        # self.initial_pos = [0.0, 0.1, 0.2]
        # self.final_pos = [0.0, 0.2, 0.3]
        self.time = 5.0
        self.wdw()

    def reached_callback(self, msg):
        if msg.data == True:
            self.point_idx += 1 
            self.wdw()
            

    def wdw(self):  
        last_p = Bool()
        last_p.data = False
        pos_i = Float64MultiArray()
        pos_f = Float64MultiArray()
        t = Float64()      

        if self.point_idx == len(self.via_point)-1:
            last_p.data = True
            pos_i.data = self.via_point[-1]
            pos_f.data = [0.669,0.,0.29]
            t.data = self.time
            
        else:
            pos_i.data = self.via_point[self.point_idx-1]
            pos_f.data = self.via_point[self.point_idx]
            t.data = self.time
        print(self.point_idx)
             
        self.p_i.publish(pos_i)
        self.p_f.publish(pos_f)
        self.T.publish(t)     
        self.state_complete.publish(last_p)

        # if self.reach == True:
        

    # def timer_callback(self):
    #     # via_p = [[1,2,3],[1,2,3],[1,2,3]]
        

        



    
 
def main(args=None):
    rclpy.init(args=args)
    joint_trajectory_tracker = Scheduler()
    try:
        while rclpy.ok():
            rclpy.spin_once(joint_trajectory_tracker)
    except KeyboardInterrupt:
        print('repeater stopped cleanly')
    except BaseException:
        print('exception in repeater:', file=sys.stderr)
        raise
    finally:
        joint_trajectory_tracker.destroy_node()
        rclpy.shutdown() 


if __name__ == '__main__':
    main()