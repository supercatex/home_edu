#!/usr/bin/env python
import rospy
from .core import Manipulator
import math

class M2(Manipulator):
	ARM_LENGTH = 10.5
	HAND_LENGTH = 12.6

	def __init__(self):
		super(M2, self).__init__()
		print("M2")
	
	def reset(self):
		self.exec_servos_pos(0, Manipulator.ARM_LENGTH * 2 + 12.6, 0, -90)
		self.close()
		self.wait()
		print("Manipulator is ready.")
		
	def exec_servos_pos(self, x, y, z, w):
		a = math.cos(w * math.pi / 180) * M2.HAND_LENGTH
		b = math.sin(w * math.pi / 180) * M2.HAND_LENGTH

		r = math.sqrt(x * x + y * y + z * z)
		print(a, b)
		x = x - a
		y = y + b

		if r >= M2.ARM_LENGTH * 1.5 and r <= M2.ARM_LENGTH * 2 + 12.6:
			print("run")
			print(x, y, z, w)
			super(M2, self).exec_servos_pos(x, y, z, w)
		else:
		 	print("not in range")
		


if __name__ == "__main__":
	rospy.init_node("home_edu_arm", anonymous=True)
	m2 = M2()
	m2.reset()
	m2.exec_servos_pos(10, 32, 0, -80)
	m2.wait()
