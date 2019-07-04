from Kobuki import Kobuki as kobuki
from Astra import Astra as astra
import cv2 as cv
import rospy
import numpy as np
from math import sqrt

class PH_Follow_me(object):
    def __init__(self):
        # Kps for turning and forward
        self.p1 = 1.0 / 900.0
        self.turn_p = -(1.0 / 150.0)
        
        # The nearest distance for the robot between the operator
        self.horizan = 595
        
        # Set the forward and turn speed
        self.forward_speed = 0
        self.turn_speed = 0
        
        # Set this two variable for finding the most center point
        self.most_center_point = (0, 0)
        self.most_center_dis = 0
        self.center_point = (240, 320)
        
        # The center point
        self.center_x = self.center_point[1]
        
        # size of the image
        self.size = (480, 640)
        
        # forward speed and turn speed
        self.forward_speed = 0
        self.turn_speed = 0
    
    def calc_kp(self, x1, y1, kp):
        error = (x1 - y1)
        return error * kp
    
    def calc_ph(self, x, y, center_point):
        # This function will only works if you have imported the maths library
        return sqrt((center_point[1] - x) ** 2 + (center_point[0] - y) ** 2)  # center_point format: (y, x)
    
    # CAUTION!!! The image is DEPTH image!!!
    def follow(self, depth, flag):
        if flag:
            frame = depth[(480 / 5):(480 / 5 * 3), (640 / 5):(640 / 5 * 3)].copy()
            nonzeros = np.nonzero(frame)
            if len(nonzeros[0]) > 0:
                if len(nonzeros[0]) > 0:
                    val = np.min(frame[nonzeros])
                    n = np.where(np.logical_and(frame <= val + 30, frame > 0))
                    # print(n)
                    
                    self.most_center_point = (0, 0)
                    self.most_center_dis = 0
                    for locations in range(len(n[0])):
                        x, y = n[1][locations], n[0][locations]
                        Dist = self.calc_ph(x, y, self.center_point)
                        if Dist < 250:
                            if self.most_center_point == (0, 0) and self.most_center_dis == 0:
                                self.most_center_point = x, y
                                self.most_center_dis = Dist
                            if Dist < self.most_center_dis:
                                self.most_center_point = x, y
                                self.most_center_dis = Dist
                        else:
                            pass
                    
                    minLoc = (self.most_center_point[0] + (640 / 5), self.most_center_point[1])
                    
                    # error = (val - horizan)
                    
                    notzero = len(nonzeros[0])
                    
                    if val < 1450 or (len(frame) - notzero > notzero):
                        self.forward_speed = self.calc_kp(val, self.horizan, self.p1)
                        
                        if not minLoc[0] == 0:
                            self.turn_speed = self.calc_kp(minLoc[0], self.center_x, self.turn_p)
                    else:
                        self.forward_speed = 0
                        self.turn_speed = 0
            if self.forward_speed < 0.7:
                return self.forward_speed, self.turn_speed
        return 0, 0
