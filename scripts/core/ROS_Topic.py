#! /usr/bin/env python


class ROS_Topic_Astra(object):

	def __init__(self, name="camera"):
		self.name = name
	
	def rgb_image_raw(self):
		return "/%s/rgb/image_raw" % (self.name)
	
	def depth_image_raw(self):
		return "/%s/depth/image_raw" % (self.name)


class ROS_Topic_Dynamixel(object):

	def __init__(self, name="tilt_controller"):
		self.name = name
	
	def command(self):
		return "/%s/command" % (self.name)
	
	def set_speed(self):
		return "/%s/set_speed" % (self.name)
	
	def state(self):
		return "/%s/state" % (self.name)


class ROS_Topic_Kobuki(object):

	def __init__(self, name="mobile_base"):
		self.name = name
	
	def velocity(self):
		return "/%s/commands/velocity" % (self.name)
	
	def imu(self):
		return "/%s/sensors/imu_data" % (self.name)
