#!/usr/bin/env python

import rospy
from core import Speech2Text
from std_msgs.msg import Int8, String
from core import Speaker
import main


def callback(data):
    global _start_order
    _start_order = bool(data.data)


rospy.init_node("home_edu_order program", anonymous=True)

s = Speech2Text()

rospy.Subscriber("/home_edu/order", Int8, callback, queue_size=1)
_main_publisher = rospy.Publisher('/home_edu/order_msg', String, queue_size=1)

_start_order = False

_order_msg = ""

kdata = main.load_data("./mission3.txt")

while not rospy.is_shutdown():
    if _start_order:
        msg = s.listen()
        _order_msg = main.answer_question_from_data(msg, kdata)['question']
        _main_publisher.publish(_order_msg)
    else:
        continue
