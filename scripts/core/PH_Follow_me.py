from Kobuki import Kobuki as kobuki
from Astra import Astra as astra
import cv2 as cv
import rospy
import numpy as np
from math import sqrt


class ph_follow(object):
    def __init__(self):
        # Kps for turning and forward
        self.p1 = 1.0 / 1400.0
        self.p2 = -(1.0 / 1400.0)
        self.turn_p = -(1.0 / 320.0)

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
        self.center_x = center_point[1]

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
    def follow(self, frame, flag):
		if flag:
			zeros = np.nonzero(frame)
			if len(zeros[0]) > 0:
				val = np.min(frame[zeros])
				n = np.where(np.logical_and(frame <= val, frame > 0))

				self.most_center_point = (0, 0)
				self.most_center_dis = 0

				for locations in range(len(n[0])):
					x, y = n[1][locations], n[0][locations]
					Dist = self.calc_ph(x, y, center_point)
					if not Dist > 250:
						if self.most_center_point == (0, 0) and self.most_center_dis == 0:
							self.most_center_point = x, y
							self.most_center_dis = Dist
						if Dist < self.most_center_dis:
							self.most_center_point = x, y
							self.most_center_dis = Dist

				minLoc = (self.most_center_point[0], self.most_center_point[1])

				error = (val - self.horizan)

				if not val > 1370:
					if error > 0:
						self.forward_speed = self.calc_kp(val, horizan, p1)

					else:
						self.forward_speed = self.calc_kp(val, horizan, p2)

					if not minLoc[0] == 0:
						self.turn_speed = self.calc_kp(minLoc[0], center_x, turn_p)
				else:
					self.forward_speed = 0
					self.turn_speed = 0

			return self.forward_speed, self.turn_speed
		else:
			return 0, 0
