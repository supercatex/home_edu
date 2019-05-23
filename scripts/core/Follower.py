#!/usr/bin/env python
import rospy
from Astra import Astra
from Kobuki import Kobuki
import cv2 as cv
import numpy as np


class Follower(object):
    
    def __init__(self, topic_name="camera", target_distance=600, PID_1=(1.0, 0.0, 0.0), PID_2=(1.0, 0.0, 0.0)):
        self.camera = Astra(topic_name)
        self.chassis = Kobuki()
        self.target_distance = target_distance

        self.PID_1 = PID_1
        self.iter_error_1 = 0.0
        self.last_error_1 = 0.0

        self.PID_2 = PID_2
        self.iter_error_2 = 0.0
        self.last_error_2 = 0.0
        
    def next_step(self):
        distance = 0
        offset_z = 0
        x = 0
        z = 0
        
        depth_image = self.camera.depth_image
        indices = np.nonzero(depth_image)
        if len(indices[0]) > 0:
            distance = np.min(depth_image[indices])
            n = np.where(depth_image == distance)
            offset_z = n[1][0]
        
        if distance == 0 or distance > 800:
            return x, z
        
        error = distance - self.target_distance
        if abs(error) < 10:
            error = 0
            self.iter_error_1 = 0
        delta_error = error - self.last_error_1
        x = self.PID_1[0] * error + self.PID_1[1] * self.iter_error_1 + self.PID_1[2] * delta_error
        self.iter_error_1 += error
        self.last_error_1 = error
        x = max(min(x, 0.5), -0.5)

        error = 320 - offset_z
        if abs(error) < 50:
            error = 0
            self.iter_error_2 = 0
        delta_error = error - self.last_error_2
        z = self.PID_2[0] * error + self.PID_2[1] * self.iter_error_2 + self.PID_2[2] * delta_error
        self.iter_error_2 += error
        self.last_error_2 = error
        z = max(min(z, 1), -1)

        return x, z


if __name__ == "__main__":
    rospy.init_node("home_edu_follower")
    rate = rospy.Rate(20)
    
    f = Follower(PID_1=(0.001, 0.00001, 0.001), PID_2=(0.0015, 0.00003, 0.01))
    while not rospy.is_shutdown():
        x, z = f.next_step()
        print(x, z)
        f.chassis.move(x, z)
        rate.sleep()
