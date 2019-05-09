#!/usr/bin/env python
import time

import rospy
from ROS_Topic import ROS_Topic_Kobuki as T
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu

'''
# Default ROS command
roslaunch turtlebot_bringup minimal.launch
'''


class Kobuki(object):
    
    # Constructor:
    def __init__(self, topic_name="mobile_base"):
        
        # Kobuki ROS topic object.
        self.topic = T(topic_name)
        
        # IMU data.
        self.imu = None
        
        # Enable flag.
        self.is_ready = False
        
        # Kobuki velocity publisher.
        self.publisher = rospy.Publisher(
            self.topic.velocity(),
            Twist,
            queue_size=1
        )
        
        # Kobuki IMU subscriber.
        self.imu_subscriber = rospy.Subscriber(
            self.topic.imu(),
            Imu,
            self.imu_callback,
            queue_size=1
        )
        
        # Waiting callback...
        rospy.loginfo("Waiting Kobuki IMU topic callback...")
        rospy.wait_for_message(self.topic.imu(), Imu)
        self.is_ready = True
        print("Kobuki (%s) is OK." % self.topic.name)
    
    # Move action.
    def move(self, forward_speed, turn_speed):
        twist = Twist()
        twist.linear.x = forward_speed
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = turn_speed
        self.publisher.publish(twist)
    
    # Left rotation.
    def clockwise_to(self, target_z, turn_speed=0.3):
        while self.is_ready:
            current_z = self.imu.orientation.z
            if abs(current_z - target_z) < 0.001:
                break
            self.move(0, -abs(turn_speed))
    
    # Right rotation.
    def anticlockwise_to(self, target_z, turn_speed=0.3):
        while self.is_ready:
            current_z = self.imu.orientation.z
            if abs(current_z - target_z) < 0.001:
                break
            self.move(0, abs(turn_speed))
    
    # Kobuki ROS IMU topic callback.
    def imu_callback(self, data):
        self.imu = data


# How to use?
if __name__ == "__main__":

    rospy.init_node("home_edu_kobuki", anonymous=True)
    rate = rospy.Rate(20)
    
    # 1. Create a Kobuki object.
    chassis = Kobuki()
    
    # # 2. Move action.
    # chassis.move(0, 0.3)
    # time.sleep(3)
    #
    # # 3. Right rotation.
    # chassis.anticlockwise_to(0.6)
    while not rospy.is_shutdown():
        z = chassis.imu.orientation.z
        print(chassis.imu.orientation)
        if not abs(z) >= 0.99:
            if z <= 1:
                chassis.move(0, 0.3)

            elif z >= 1:
                chassis.move(0, -0.3)
        else:
            break

