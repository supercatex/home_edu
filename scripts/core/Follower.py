#!/usr/bin/env python
import rospy
from Astra import Astra
from Kobuki import Kobuki
import cv2 as cv
import numpy as np


class Follower(object):
    
    def __init__(self, topic_name="camera", target_distance=600, PID=(1.0, 0.0, 0.0)):
        self.camera = Astra(topic_name)
        self.chassis = Kobuki()
        self.target_distance = target_distance
        self.PID = PID
        self.iter_error = 0.0
        self.last_error = 0.0
        
    def next_step(self):
        distance = 0
        x = 0
        z = 0
        
        depth_image = self.camera.depth_image
        indices = np.nonzero(depth_image)
        if len(indices[0]) > 0:
            distance = np.min(depth_image[indices])
        
        if distance == 0 or distance > 1500:
            return x, z
        
        error = distance - self.target_distance
        delta_error = error - self.last_error
        x = self.PID[0] * error + self.PID[1] * self.iter_error + self.PID[2] * delta_error
        self.iter_error += error
        self.last_error = error
        
        return x, z


if __name__ == "__main__":
    rospy.init_node("home_edu_follower")
    rate = rospy.Rate(20)
    
    f = Follower(PID=(0.002, 0.00001, 0.001))
    while not rospy.is_shutdown():
        x, z = f.next_step()
        print(x, f.iter_error)
        f.chassis.move(x, z)
        rate.sleep()
