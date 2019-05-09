#!/usr/bin/env python
import cv2 as cv
import math
import rospy
from Astra import Astra
from Manipulator import Manipulator


class RobotArm(object):
    HEIGHT = 50.5
    MANIPULATOR_ORIGIN_X = 18.0
    MANIPULATOR_ORIGIN_Y = 10.0
    
    FOV_H = 60.0
    FOV_V = 49.5
    FOV_D = 73.0
    
    IMAGE_W = 640
    IMAGE_H = 480
    
    def __init__(self, name="camera"):
        self.camera = Astra(name)
        self.manipulator = Manipulator()
        self.manipulator.exec_servos_pos(0, 10, 0, 0)
        print("RobotArm is ready.")
    
    def calc_manipulator_pos(self, image, depth, x, y):
        d = depth[y, x] / 10
        lh = d * math.tan(RobotArm.FOV_V * math.pi / 180 / 2)
        ly = float(y - RobotArm.IMAGE_H / 2) / (RobotArm.IMAGE_H / 2) * lh
        mx = d - RobotArm.MANIPULATOR_ORIGIN_X
        my = RobotArm.HEIGHT - RobotArm.MANIPULATOR_ORIGIN_Y - ly
        print(mx, my)
        self.manipulator.exec_servos_pos(mx, my, 0, 0)
    
    def mouse_callback(self, event, x, y, flags, param):
        if event == cv.EVENT_LBUTTONDOWN:
            depth = self.camera.depth_image[y, x]
            print(self.camera.depth_image.shape)
            print("x=%d, y=%d, distance=%d" % (x, y, depth))
            
            self.calc_manipulator_pos(self.camera.rgb_image, self.camera.depth_image, x, y)


if __name__ == "__main__":
    rospy.init_node("home_edu_robotarm", anonymous=True)
    rate = rospy.Rate(20)
    
    arm = RobotArm()
    cv.namedWindow("image")
    cv.setMouseCallback("image", arm.mouse_callback)
    while not rospy.is_shutdown():
        cv.imshow("image", arm.camera.rgb_image)
        if cv.waitKey(1) == 27:
            break
        rate.sleep()
    cv.destroyAllWindows()
