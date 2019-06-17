#!/usr/bin/env python
import rospy
from std_msgs.msg import String


rospy.init_node("home_edu_Kinda")

my_publisher = rospy.Publisher("/home_edu_Kinda/msg", String, queue_size=1)

while True:
    my_publisher.publish("Hello")
