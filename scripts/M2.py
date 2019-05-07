#!/usr/bin/env python
import rospy
from Manipulator import Manipulator
import math

class M2(Manipulator):
	HAND_LENGTH = 12.6

	def __init__(self):
		super(M2, self).__init__()
		print("M2")
	
	def exec_servos_pos(self, x, y, z, w):
		a = math.cos(-w * math.pi / 180) * M2.HAND_LENGTH
		b = math.sin(-w * math.pi / 180) * M2.HAND_LENGTH

		r = math.sqrt(x * x + y * y + z * z)

		if r > M2.ARM_LENGTH * 1 and r < M2.ARM_LENGTH * 2 + 12.6:
			if x < 12 and y < -10:
				print("not avaliable")
			else:
				x = x - a
				y = y - b
				super(M2, self).exec_servos_pos(x, y, z, w)
		else:
		 	print("not in range")
		


if __name__ == "__main__":
	rospy.init_node("home_edu_arm", anonymous=True)
	m2 = M2()
	m2.reset()
	print("ok")
	m2.exec_servos_pos(10, 5, 0, 0)
	m2.wait()