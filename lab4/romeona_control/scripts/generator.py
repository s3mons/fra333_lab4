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
from std_msgs.msg import Int32, Float64MultiArray, Float64

class Generator(Node):

    def __init__(self):
        super().__init__('generator')

        # subscription from GZ
        self.p_i = self.create_subscription(Float64MultiArray,'/p_i',self.initial_position_callback,10)
        self.p_f = self.create_subscription(Float64MultiArray,'/p_f',self.final_position_callback,10)
        self.T   = self.create_subscription(Float64,'/T',self.time_callback,10)


        # create publisher
        publish_topic = "/velocity_controllers/command"
        # self.pos_publisher = self.create_publisher(Float64MultiArray,publish_topic, 10)
        self.vel_command_publisher = self.create_publisher(JointTrajectoryPoint,publish_topic, 10)


        # timer
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # variable
        # self.initial_pos = [0.0, 0.0, 0.0]
        # self.final_pos = [0.0, 0.0, 0.0]
        # self.time = 2
    

    def initial_position_callback(self, msg):
        self.initial_pos = msg.data

    def final_position_callback(self, msg):
        self.final_pos = msg.data

    def time_callback(self, msg):
        self.time = msg.data
        
    def timer_callback(self):
        i = 0
        f = 1
        t = self.time
        a0 = i
        a1 = 0
        a2 = 0
        a3 = 10/(t**3)
        a4 = -15/(t**4)
        a5 = 6/(t**5)

        s = a0 + a1*t + a2*(t**2) + a3*(t**3) + a4*(t**4) + a5*(t**5)
        s_dot = a1 + 2*a2*t + 3*a3*(t**2) + 4*a4*(t**3) + 5*a5*(t**4)

        p_r = (1-s)*np.array(self.initial_pos) + s*np.array(self.final_pos)
        v_r = s_dot*(self.final_pos-self.initial_pos)

        q_r = pos_inverse_kinematics(p_r)
        q_r_dot = vel_inverse_kinematics(q_r,v_r)

        ref_jointstate = JointTrajectoryPoint()
        ref_jointstate.positions = q_r
        ref_jointstate.velocities = q_r_dot    
        self.vel_command_publisher.publish(ref_jointstate)

    
 
def main(args=None):
    rclpy.init(args=args)
    joint_trajectory_tracker = Generator()
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