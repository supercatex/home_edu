#!/usr/bin/env python

from random import randint
import rospy
import time 

rospy.init_node("pcms_test", anonymous=True)

rate = rospy.Rate(20)

while True:
	random_num = randint(90, 100)
	print(random_num)
	if random_num > 90:
		last_time = time.time()
		if time.time() - last_time >= 5:
			break
				
	else:
		pass