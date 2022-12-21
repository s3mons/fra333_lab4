#!/usr/bin/python3
import rclpy
import sys
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool , Float64MultiArray
from visualization_msgs.msg import MarkerArray , Marker
from geometry_msgs.msg import Point
from romeona_control.Jacobian import *
import numpy as np

class marker_node(Node):
    def __init__(self):
        super().__init__("marker")
        self.Marker_pub = self.create_publisher(Marker,"/visualization_marker",10)
        self.q_sub = self.create_subscription(JointState,"/joint_states" ,self.q_callback, 10)
        self.Marker = Marker()
        self.Marker.header.frame_id = "/world"
        self.Marker.type = 8
        # self.Marker.type = 4
        self.Marker.scale.x = 0.01
        self.Marker.scale.y = 0.01
        self.Marker.scale.z = 0.01

        self.Marker.color.r = 220/255 
        self.Marker.color.g = 50/255
        self.Marker.color.b = 0.0
        self.Marker.color.a = 1.0

        self.timer = 0.1
        self.timer = self.create_timer(self.timer,self.timer_callback)

        self.position = np.array([0.,0.,0.])

    def q_callback(self,msg):
        Rotation,Position,R_e,p_e,H0_e = forward_kin(msg.position)
        self.position = p_e[0]

    def timer_callback(self):

        now = self.get_clock().now()
        self.Marker.header.stamp = now.to_msg()
        pos = Point()
        pos.x = self.position[0]
        pos.y = self.position[1]
        pos.z = self.position[2]
        if(pos.z < 0.145):
            self.Marker.points.append(pos)
            self.Marker_pub.publish(self.Marker)
        

        



def main(args=None):
    rclpy.init(args=args)
    joint_trajectory_tracker = marker_node()
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
