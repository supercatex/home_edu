#!/usr/bin/env python
import rospy
from core import Speech2Text as Speech2Text
from std_msgs.msg import String

s = Speech2Text()
s.ambient_noise()

_isListen = 'false'
def callback(data):
	global _isListen
	_isListen = data.data
	print(data.data)

rospy.init_node("home_edu_PCMS_Second_mission_listen")

my_publisher = rospy.Publisher("/home_edu_Listen/msg", String, queue_size=1)
my_subscriber = rospy.Subscriber("/home_edu_Listen/situation", String, callback, queue_size=1)


while True:
	if _isListen == "true":
		msg = s.listen()
		print(msg)
		my_publisher.publish(msg)
	else:
		pass
	