#!/usr/bin/python3
import math
from re import S
import numpy as np
from numpy import ones, zeros
import rclpy
import sys, os, yaml
from rclpy.node import Node
from builtin_interfaces.msg import Duration
from romeona_interfaces import enableTracker
from romeona_control.Jacobian import pos_inverse_kinematics, vel_inverse_kinematics
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory , JointTrajectoryPoint
from std_msgs.msg import Int32, Float64MultiArray, Float64, Bool
from romeona_control.Jacobian import forward_kin

class Proximity_detector(Node):

    def __init__(self):
        super().__init__('proximity_detector')

        # subscript joint state from GZ
        self.joint_state = self.create_subscription(JointState,'/joint_state',self.joint_state_callback,10)

        # subscript pos_ref
        self.position_referance = self.create_subscription(Float64MultiArray,'/final_position',self.final_pos_callback,10)


        # create publisher
        self.reach = self.create_publisher(Bool,"/hasReached", 10)


        # timer
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # variable
        self.threshold = 0.1
        self.ang_pos = [0.0, 0.0, 0.0]
        self.pos_ref = [0.0, 0.0, 0.0]

    def joint_state_callback(self, msg):
        self.ang_pos = msg.position

    def final_pos_callback(self, msg):
        self.pos_ref = msg.data

        
    def timer_callback(self):
        arrive = Bool()
        R,P,R_e,p_e,H0_e = forward_kin(self.ang_pos)
        diff = np.subtract(np.array(self.pos_ref),np.array(p_e[0]))
        if abs(diff[0]) <= self.threshold and abs(diff[1]) <= self.threshold and abs(diff[2]) <= self.threshold:  
            arrive = True   
        else:
            arrive = False
        self.reach.publish(arrive)

    
 
def main(args=None):
    rclpy.init(args=args)
    joint_trajectory_tracker = Proximity_detector()
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