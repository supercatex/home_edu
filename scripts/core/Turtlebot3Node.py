#!/usr/bin/env python3
# -*- encoding: utf-8 -*-
import os, sys
sys.path.append(os.path.join(os.path.dirname(__file__), ".."))
from core import *
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion
import numpy as np


class Turtlebot3Node(Node):
    def __init__(self, node_name):
        super(Turtlebot3Node, self).__init__(node_name)

        self.imu = None

        self.pub = Pub("/cmd_vel", Twist)
        self.sub = Sub("/imu", Imu, self.imu_callback, 1, True)
        

    def imu_callback(self, msg: Imu):
        self.imu = msg

    def move(self, forward_speed: float = 0, turn_speed: float = 0):
        msg = Twist()
        msg.linear.x = forward_speed
        msg.angular.z = turn_speed
        self.pub.publish(msg)
    
    def turn_to(self, angle: float, speed: float):
        max_speed = 1.82
        limit_time = 8
        start_time = rospy.get_time()
        while True:
            q = [
                self.imu.orientation.x,
                self.imu.orientation.y,
                self.imu.orientation.z,
                self.imu.orientation.w
            ]
            roll, pitch, yaw = euler_from_quaternion(q)
            e = angle - yaw
            if yaw < 0 and angle > 0:
                cw = np.pi + yaw + np.pi - angle
                aw = -yaw + angle
                if cw < aw:
                    e = -cw
            elif yaw > 0 and angle < 0:
                cw = yaw - angle
                aw = np.pi - yaw + np.pi + angle
                if aw < cw:
                    e = aw
            if abs(e) < 0.01 or rospy.get_time() - start_time > limit_time:
                break
            self.move(0.0, max_speed * speed * e)
            rospy.Rate(20).sleep()
        self.move(0.0, 0.0)
    
    def turn(self, angle: float):
        q = [
            self.imu.orientation.x,
            self.imu.orientation.y,
            self.imu.orientation.z,
            self.imu.orientation.w
        ]
        roll, pitch, yaw = euler_from_quaternion(q)
        target = yaw + angle
        if target > np.pi:
            target = target - np.pi * 2
        elif target < -np.pi:
            target = target + np.pi * 2
        self.turn_to(target, 0.5)


if __name__ == "__main__":
    tb3 = Turtlebot3Node("tb3").spin()
