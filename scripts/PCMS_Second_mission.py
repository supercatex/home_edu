#!/usr/bin/env python

import rospy
import time
#import FollowMe as follow
from core import Manipulator as mani
from core import Kobuki as kobuki
from core import Speaker as speaker
from core import Speech2Text as speech2text

m = mani()
k = kobuki()
s = speaker()
t = speech2text()

if __name__ == '__main__':
	rospy.init_node("home_edu_arm", anonymous=True)
	time.sleep(1)
	s.say("hello, I'm your assistant")
	time.sleep(1)
	s.say("please stand in front of me")
	s.say("please say follow me for following you")
