#!/usr/bin/env python

import rospy
from std_msgs.msg import String


def callback(data):
    global _signal_key
    _signal_key = data.data


_signal_key = 0
rospy.init_node("home_edu_client_4", anonymous=True)

rospy.Subscriber(
    '/home_edu/facial_key',
    String,
    callback,
    queue_size=1
)

_mission4_sig_pub = rospy.Publisher(
    '/home_edu/mission4_signal',
    String,
    queue_size=1
)

while not rospy.is_shutdown():
    if _signal_key == "32":
        _mission4_sig_pub.publish("1")
    elif _signal_key == "-1":
        _mission4_sig_pub.publish('0')
    elif _signal_key == "113":
        _mission4_sig_pub.publish("-1")
    else:
        pass
