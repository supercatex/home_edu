#!/usr/bin/env python
import rospy
from core import Speech2Text as Speech2Text
from std_msg.msg import String 

s = Speech2Text()

rospy.init_node("home_edu_PCMS_Second_mission_listen")

my_publisher = rospy.Publisher("/home_edu_Listen/msg", String, queue_size=1)

while True:
    my_publisher.publish(s)