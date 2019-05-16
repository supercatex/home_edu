#!/usr/bin/env python

import cv2 as cv
import time
import rospy
from std_msgs.msg import String


_is_talk = False
_last_change_time = time.time()
_last_change_image_name = ""
_image = cv.imread("./libs/emoji_smile.png", cv.IMREAD_COLOR)


def callback(data):
    global _image, _is_talk, _last_change_time, _last_change_image_name
    _image = cv.imread("./libs/emoji_%s.png" % data.data, cv.IMREAD_COLOR)
    if data.data == "talk":
        _is_talk = True
        _last_change_time = time.time()
        _last_change_image_name = "talk"
    else:
        _is_talk = False
        _last_change_image_name = "smile"


rospy.init_node("home_edu_emoji_viewer")
status_sub = rospy.Subscriber(
    "/home_edu/emoji_type",
    String,
    callback
)

while not rospy.is_shutdown():

    print(_is_talk, _last_change_image_name, _last_change_time)
    if _is_talk and time.time() - _last_change_time > 0.01:
        print("IN")
        if _last_change_image_name == "talk":
            _last_change_image_name = "smile"
        else:
            _last_change_image_name = "talk"
        img = cv.imread("./libs/emoji_%s.png" % (_last_change_image_name), cv.IMREAD_COLOR)
        cv.imshow("frame", img)
        _last_change_time = time.time()
    else:
        cv.imshow('frame', _image)

    cv.waitKey(1)
