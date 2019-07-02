#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from core import Speaker as speaker


def callback(data):
    global _msg
    _msg = data.data


rospy.init_node("home_edu_mission3", anonymous=True)

rate = rospy.Rate(20)

_listen_controller = rospy.Publisher("/home_edu_Listen/situation", String, queue_size=1)
_listen_msg = rospy.Subscriber("/home_edu_Listen/msg", String, callback, queue_size=1)

p = speaker()

_msg = ""

while not rospy.is_shutdown():
    p.say("Please tell me your order")
    _listen_controller.publish('true')
    p.say("Our guest's order is {}".format(_msg))
    rate.sleep()
