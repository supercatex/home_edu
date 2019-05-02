#!/usr/bin/env python
import rospy
from Dynamixel import Dynamixel


class Arm(object):

	def __init__(self):
		self.servos = []
		self.servos.append(Dynamixel("waist_controller"))
		self.servos.append(Dynamixel("shoulder_controller"))
		self.servos.append(Dynamixel("elbow_controller"))
		self.servos.append(Dynamixel("wrist_controller"))
		self.servos.append(Dynamixel("hand_controller"))


if __name__ == "__main__":
	rospy.init_node("home_edu_arm", anonymous=True)
	arm = Arm()
	for servo in arm.servos:
		servo.set_radian(0)