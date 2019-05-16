#!/usr/bin/env python

import rospy
from core.Speaker import Speaker as speaker
import time
from std_msgs.msg import String

rospy.init_node("home_edu_emoji_speaker")

face_pub = rospy.Publisher(
    "/home_edu/emoji_type",
    String,
    queue_size=1
)

p = speaker()
while not rospy.is_shutdown():
    text = raw_input("What do you want me to say? ")
    face_pub.publish('talk')
    p.say(text)
    face_pub.publish("smile")
