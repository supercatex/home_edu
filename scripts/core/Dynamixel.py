#!/usr/bin/env python
import rospy
from .ROS_Topic import ROS_Topic_Dynamixel as T
from std_msgs.msg import Float64
from dynamixel_controllers.srv import SetSpeed
from dynamixel_msgs.msg import JointState
import math
import time


'''
roslaunch rchomeedu_arm arm.launch
'''

class Dynamixel(object):

	def __init__(self, name="tilt_controller"):
		self.topic = T(name)
		self.state = None
		
		self.publisher = rospy.Publisher(
			self.topic.command(),
			Float64,
			queue_size = 1,
			latch = True
		)
		
		self.state_subscriber = rospy.Subscriber(
			self.topic.state(),
			JointState,
			self.state_callback
		)
		
		rospy.wait_for_service(self.topic.set_speed())
		self.set_speed = rospy.ServiceProxy(
			self.topic.set_speed(),
			SetSpeed
		)
		
		while self.state is None: pass
		print("Dynamixel (%s) is OK." % self.topic.name)
	
	def set_radian(self, radian):
		self.publisher.publish(radian)
	
	def set_speed(self, speed):
		self.set_speed(speed)
	
	def state_callback(self, data):
		self.state = data


if __name__ == "__main__":
	rospy.init_node("home_edu_dynamixel", anonymous=True)
	rate = rospy.Rate(20)
	servo = Dynamixel("waist_controller")
	servo.set_speed(0.3)
	if servo.state.current_pos > 1:
		servo.set_radian(0)
	else:
		servo.set_radian(math.pi / 4)
	
	time.sleep(1)
	while servo.state.is_moving:
		print(servo.state)
		rate.sleep()

