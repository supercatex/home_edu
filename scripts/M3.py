#!/usr/bin/env python
import rospy
from Manipulator import Manipulator
import math
import numpy as np

class M3(Manipulator):
	l1 = 10.5
	l2 = 10.5
	l3 = 12.6
	MAX_SPEED = 1
	def __init__ (self):
		super(M3, self).__init__()
		print('M3')
		
	def reset(self):
		self.run(0, M3.l1 + M3.l2 + M3.l3, 0, -90)
		self.close()
		self.wait()
		print("Manipulator is ready.")

	def run(self, x, y, z, w):
		a = math.cos(w * math.pi / 180) * M3.l3
		b = math.sin(w * math.pi / 180) * M3.l3

		r = math.sqrt(x * x + y * y + z * z)
		x = x - a
		y = y + b
		print(x, y, z, w)
		if r >= M3.l1 * 1.5 and r <= M3.l1 * 2 + 12.6:
			print("run")
			self.exec_servos_pos(x, y, z, w)
		else:
		 	print("not in range")

 	def exec_servos_pos(self, x, y, z, w):
		x = float(x)
		y = float(y)
		z = float(z)
		w = float(w)
		if math.sqrt(x * x + y * y + z * z) > M3.l1 * 2:
			print("Cannot move to point(%.2f, %.2f, %.2f)" % (x, y, z))
			return
		
		if self.target_x == x and self.target_y == y and self.target_z == z and self.target_w == w:
			print("Same poisition point(%.2f, %.2f, %.2f)" % (x, y, z))
			return
		alpha = [0, 0, 0, 0]
		x = float(x)
		y = float(y)
		z = float(z)
		w = float(w)
		pi = math.pi
		x = math.sqrt(x**2 + z**2)
		r = w * pi / 180
		D = x**2 + y**2
		a = math.atan2(y, x)
		c = a + r
		l4 = math.sqrt(M3.l3**2 + D - (2 * M3.l3 * math.sqrt(D) * math.cos(c)))
		f = math.acos((M3.l1**2 + l4**2 - M3.l2**2) / (2 * l4 * M3.l1))
		e = math.acos((D + l4**2 - M3.l3**2) / (2 * math.sqrt(D) * l4))
		shoulder_angle = float(90 - (a * 180 / pi) - (f * 180 / pi) - (e * 180 / pi))
		g = math.acos((M3.l1**2 + M3.l2**2 - l4**2) / (2 * M3.l1 * M3.l2))
		elbow_angle = float(180 - (g * 180 / pi))
		h = 180 - (g * 180 / pi) - (f * 180 / pi)
		d = 180 - (e * 180 / pi) - (c * 180 / pi)
		wrist_angle = float(180 - h - d)
		waist_angle = float((math.atan2(z, x)) * 180 / pi)
		alpha[0] = waist_angle * pi / 180
		alpha[1] = shoulder_angle * pi / 180
		alpha[2] = elbow_angle * pi / 180
		alpha[3] = wrist_angle * pi / 180
		delta_list = []
		for i in range(len(alpha)):
			delta_list.append(abs(alpha[i] - self.servos[i].state.current_pos))
		max_delta = np.max(delta_list)
		
		for i in range(len(alpha)):
			self.servos[i].set_speed(M3.MAX_SPEED * delta_list[i] / max_delta)
			self.servos[i].set_radian(alpha[i])

		print(x, y, z, w)
		print(alpha)

if __name__ == "__main__":
	rospy.init_node("home_edu_arm", anonymous=True)
	m3 = M3()
	m3.reset()
	m3.run(10, 20, 0, 0)
	m3.wait()

