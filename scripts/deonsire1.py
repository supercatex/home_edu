#!/usr/bin/env python
import rospy
from core import RobotChassis
from core import Speaker as speaker
from core import Speech2Text as speech2text

rospy.init_node("home_edu_deonsire1", anonymous=True)
rate = rospy.Rate(20)

chassis = RobotChassis()

s = speaker()

t = speech2text()

init = [2.67, -0.297, 0.00247]

goal = [[-1.49, 8.48, 0.00247], [7.51, 7.52, -0.00143], [10.6, -3.76, -0.00143], [2.67, -0.297, 0.00247]]

chassis.set_initial_pose_in_rviz()

i = 0

while True:
    success = chassis.move_to(goal[i][0], goal[i][1], goal[i][2])
    
    s.say("i have arrived to the point")
    
    if success == 1:
        if i == 4:
            i = 0
        else:
            i += 1
    else:
        continue

print("END")