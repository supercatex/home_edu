#!/usr/bin/env python
import rospy
from core import RobotChassis

rospy.init_node("home_edu_deonsire1", anonymous=True)
rate = rospy.Rate(20)

chassis = RobotChassis()

init = [2.67, -0.297, 0.00247]
goal = [[-1.49, 8.48, 0.00247], [7.51, 7.52, -0.00143], [10.6, -3.76, -0.00143]]

chassis.set_initial_pose_in_rviz()

i = 0

success = chassis.move_to(goal[i][0], goal[i][1], goal[i][2])

if success == 1:
    i += 1
    chassis.move_to(goal[i][0], goal[i][1], goal[i][2])

print("END")