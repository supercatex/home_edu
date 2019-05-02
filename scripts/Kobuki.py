#!/usr/bin/env python
import rospy
from libs import ROS_Topic as T
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu


class Kobuki(object):

	def __init__(self, name="mobile_base"):
		self.topic = T.Kobuki(name)
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
	
	while chassis.imu.orientation.z < 0.9:
		print(chassis.imu.orientation)
		chassis.action(0, 0.3)
		rate.sleep()
	print(chassis.imu.orientation)
