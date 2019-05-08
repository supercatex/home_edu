#!/usr/bin/env python
import rospy
from ROS_Topic import ROS_Topic_Kobuki as T
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
		self.is_ready = False
		
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
		self.is_ready = True
		print("Kobuki (%s) is OK." % self.topic.name)
	
	def move(self, forward_speed, turn_speed):
		twist = Twist()
		twist.linear.x = forward_speed
		twist.linear.y = 0
		twist.linear.z = 0
		twist.angular.x = 0
		twist.angular.y = 0
		twist.angular.z = turn_speed
		self.publisher.publish(twist)

	def clockwise_to(self, target_z, turn_speed=0.3):
		while self.is_ready:
			current_z = self.imu.orientation.z
			if abs(current_z - target_z) < 0.001:
				break
			self.move(0, -abs(turn_speed))

	def anticlockwise_to(self, target_z, turn_speed=0.3):
		while self.is_ready:
			current_z = self.imu.orientation.z
			if abs(current_z - target_z) < 0.001:
				break
			self.move(0, abs(turn_speed))
	
	def imu_callback(self, data):
		self.imu = data


if __name__ == "__main__":
	rospy.init_node("home_edu_kobuki", anonymous=True)
	rate = rospy.Rate(20)
	chassis = Kobuki()

	chassis.anticlockwise_to(0.6)
