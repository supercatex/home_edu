#!/usr/bin/env python
import rospy
from .ROS_Topic import ROS_Topic_Kobuki as T
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
import time


'''
roslaunch turtlebot_bringup minimal.launch
'''

class Kobuki(object):

	def __init__(self, name="mobile_base"):
		self.topic = T(name)
		self.imu = None
		
		self.publisher = rospy.Publisher(
			self.topic.velocity(),
			Twist,
			queue_size = 1
		)
		
		self.imu_subscriber = rospy.Subscriber(
			self.topic.imu(),
			Imu,
			self.imu_callback,
			queue_size = 1
		)
		
		while self.imu is None: pass
		print("Kobuki (%s) is OK." % self.topic.name)
	
	def action(self, move, turn):
		twist = Twist()
		twist.linear.x = move
		twist.linear.y = 0
		twist.linear.z = 0
		twist.angular.x = 0
		twist.angular.y = 0
		twist.angular.z = turn
		self.publisher.publish(twist)
	
	def imu_callback(self, data):
		self.imu = data


if __name__ == "__main__":
	rospy.init_node("home_edu_kobuki", anonymous=True)
	rate = rospy.Rate(20)
	chassis = Kobuki()

	last_time = time.time()

	while not rospy.is_shutdown():
		if chassis.imu.orientation.z >= 0:
			chassis.action(0, 0.3)

		if chassis.imu.orientation.z <= 0:
			chassis.action(0, -0.3)

		if abs(chassis.imu.orientation.z) > 0.99:
			break

		last_time = time.time() - last_time
		rate.sleep()
		print(chassis.imu.orientation)

		print("Ok, used time: {}".format(round(last_time, 2)))
