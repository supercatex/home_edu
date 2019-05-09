#! /usr/bin/env python
# -*- encoding: UTF-8 -*-
import os
import time
import rospy
import atexit
from turtlebot_msgs.srv import SetFollowState
import thread
from threading import Thread


class follower_control(object):
	self.command = 'r'
    def __init__(self, command):
		self.command = command
        rospy.init_node("follower_control")
        self.keyboard_control()

    def control_follow(self, msg):
        rospy.wait_for_service('/turtlebot_follower/change_state')
        change_state = rospy.ServiceProxy('/turtlebot_follower/change_state', SetFollowState)
        response = change_state(msg)
        print "(state changed)"

    def keyboard_control(self):
        print('Follower Control: Start(f) Stop(s) Quit(x)')
        while self.command != 'c':
            try:
                if self.command == 'f':
                    self.control_follow(1)
                elif self.command == 's':
                    self.control_follow(0)
                elif self.command == 'q':
                    break
                else:
                    print("Invalid command!")
            except EOFError:
                print "Error!!!"



if __name__ == "__main__":
    follower_control(f)