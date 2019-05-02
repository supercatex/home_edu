#!/usr/bin/env python
import rospy
import roslib
import cv2 as cv
import numpy as np
import ROS_Topic as T
from cv_bridge import CvBridge
from sensor_msgs.msg import Image


class Camera(object):

	def __init__(self, name):
		self.topic = T.Astra(name)
		self.bridge = CvBridge()
		self.rgb_image = None
		self.depth_image = None
		
		self.rgb_subscriber = rospy.Subscriber(
			self.topic.rgb_image_raw(), 
			Image, 
			self.rgb_callback, 
			queue_size = 1
		)
		
		self.depth_subscriber = rospy.Subscriber(
			self.topic.depth_image_raw(), 
			Image, 
			self.depth_callback, 
			queue_size = 1
		)
	
	def rgb_callback(self, msg):
		self.rgb_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
	
	def depth_callback(self, msg):
		self.depth_image = self.bridge.imgmsg_to_cv2(msg, "passthrough")
		self.depth_image = np.array(self.depth_image, dtype=np.float32)
	
	def mouse_callback(self, event, x, y, flags, param):
		if self.depth_image is None:
			return
		
		if event == cv.EVENT_LBUTTONDOWN:
			val = self.depth_image[y, x]
			print("x=%d, y=%d, distance=%d" % (x, y, val))


if __name__ == "__main__":
	rospy.init_node("home_edu_camera", anonymous=True)
	try:
		c = Camera("cam1")
		cv.namedWindow("image")
		cv.setMouseCallback("image", c.mouse_callback)
		while not rospy.is_shutdown():
			if c.rgb_image is None or c.depth_image is None:
				continue
			cv.imshow("image", c.rgb_image)
			if cv.waitKey(1) == 27:
				break
	except Exception as e:
		print(e)
	finally:
		cv.destroyAllWindows()
