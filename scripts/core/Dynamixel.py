#!/usr/bin/env python
import time

import math
import rospy
from ROS_Topic import ROS_Topic_Dynamixel as T
from dynamixel_controllers.srv import SetSpeed
from dynamixel_msgs.msg import JointState
from std_msgs.msg import Float64

'''
# Default ROS command.
roslaunch home_edu arm.launch
'''


class Dynamixel(object):

    # Constructor:
    def __init__(self, topic_name="tilt_controller"):

        # Dynamixel ROS topic object.
        self.topic = T(topic_name)

        # Dynamixel information.
        self.state = None

        # Dynamixel command publisher.
        self.publisher = rospy.Publisher(
            self.topic.command(),
            Float64,
            queue_size=1,
            latch=True
        )

        # Dynamixel state subscriber.
        self.state_subscriber = rospy.Subscriber(
            self.topic.state(),
            JointState,
            self.state_callback
        )

        # Dynamixel speed service.
        rospy.wait_for_service(self.topic.set_speed())
        self.set_speed = rospy.ServiceProxy(
            self.topic.set_speed(),
            SetSpeed
        )

        rospy.loginfo("Waiting %s topic callback..." % self.topic.name)
        rospy.wait_for_message(self.topic.state(), JointState)
        rospy.loginfo("Dynamixel (%s) is OK." % self.topic.name)

    # Change Dynamixel angle.
    def set_radian(self, radian):
        self.publisher.publish(radian)

    # Change Dynamixel moving speed.
    def set_speed(self, speed):
        self.set_speed(speed)

    # Dynamixel ROS state topic callback.
    def state_callback(self, data):
        self.state = data


# How to use?
if __name__ == "__main__":
    rospy.init_node("home_edu_dynamixel", anonymous=True)
    rate = rospy.Rate(20)
    
    # 1. Create a Dynamixel object.
    servo = Dynamixel("waist_controller")
    
    # 2. Set default speed.
    servo.set_speed(0.3)
    
    # 3. Set angle.
    if servo.state.current_pos >= 0.01:
        servo.set_radian(0)
    else:
        servo.set_radian(math.pi / 4)
    
    # 4. Set a delay to wait state change to is moving.
    time.sleep(0.2)
    
    # 5. Display state information on the screen.
    while servo.state.is_moving:
        rospy.loginfo(servo.state)
        rate.sleep()
