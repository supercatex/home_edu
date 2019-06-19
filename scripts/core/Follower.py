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
        offset_y = 0
        x = 0
        z = 0

        gx, gy = np.meshgrid(np.linspace(-1, 1, 640), np.linspace(-1, 1, 480))
        d = np.sqrt(gx * gx + gy * gy)
        sigma, mu = 0.5, 0.0
        g = np.exp(-((d - mu) ** 2 / (2.0 * sigma ** 2)))
        print(np.min(g), np.max(g))
        cv.imshow("g", g)
        
        depth_image = self.camera.depth_image
        d = depth_image - (300 * g)

        # indices = np.nonzero(depth_image)
        indices = np.where(d > 1)
        if len(indices[0]) > 0:
            distance = np.min(d[indices])
            n = np.where(d == distance)
            offset_z = n[1][0]
            offset_y = n[0][0]
            distance = depth_image[offset_y, offset_z]
        
        if distance == 0 or distance > 1500:
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
        print(error)

        error = 320 - offset_z
        if abs(error) < 10:
            error = 0
            self.iter_error_2 = 0
        delta_error = error - self.last_error_2
        z = self.PID_2[0] * error + self.PID_2[1] * self.iter_error_2 + self.PID_2[2] * delta_error
        self.iter_error_2 += error
        self.last_error_2 = error
        z = max(min(z, 1), -1)
        print(error)

        frame = self.camera.rgb_image
        cv.circle(frame, (offset_z, offset_y), 10, (0, 0, 255), -1)
        cv.imshow("frame", frame)
        cv.waitKey(1)

        return x, z


if __name__ == "__main__":
    rospy.init_node("home_edu_follower")
    rate = rospy.Rate(20)
    
    f = Follower(
        topic_name="top_camera",
        target_distance=800,
        PID_1=(0.001, 0.0, 0.0),
        PID_2=(0.01, 0.0, 0.0)
    )
    while not rospy.is_shutdown():
        x, z = f.next_step()
        print(x, z)
        f.chassis.move(x, z)
        rate.sleep()
