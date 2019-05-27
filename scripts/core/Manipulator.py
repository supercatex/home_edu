#!/usr/bin/env python
import time

import math
import numpy as np
import rospy
from Dynamixel import Dynamixel

'''
# Default ROS command.
roslaunch home_edu arm.launch
'''


class Manipulator(object):
    
    # Two ARM is same length.
    ARM_LENGTH = 10.5
    HAND_LENGTH = 12.5
    MAX_SPEED = 1.0
    
    def __init__(self):
        
        # Initial five servos.
        self.servos = []
        self.servos.append(Dynamixel("waist_controller"))
        self.servos.append(Dynamixel("shoulder_controller"))
        self.servos.append(Dynamixel("elbow_controller"))
        self.servos.append(Dynamixel("wrist_controller"))
        self.servos.append(Dynamixel("hand_controller"))
        
        # Default speed is MAX_SPEED.
        for servo in self.servos:
            servo.set_speed(Manipulator.MAX_SPEED)
        
        # Default pose is all zero, the first run should be call reset().
        self.target_x = 0
        self.target_y = 0
        self.target_z = 0
        self.target_w = 0
    
    # Reset arm to default pose.
    def reset(self):
        self.exec_servos_pos(0, Manipulator.HAND_LENGTH + Manipulator.ARM_LENGTH * 2, 0, -90)
        self.close()
        self.wait()
        print("Manipulator is ready.")
    
    def calc_servos_pos(self, x, y, z, w=0):
        # Calculation.
        alpha = [0, 0, 0, 0]

        alpha[0] = math.atan2(z, x)

        t = 0
        if x != 0:
            t = abs(x) / x
        x = math.sqrt(x * x + z * z) * t
        y = y
        z = 0

        w = float(w) * math.pi / 180
        x = x - Manipulator.HAND_LENGTH * math.cos(w)
        y = y + Manipulator.HAND_LENGTH * math.sin(w)
        print("Goto:", x, y)
    
        L1 = Manipulator.ARM_LENGTH
        L2 = Manipulator.ARM_LENGTH
        L1_2 = L1 * L1
        L2_2 = L2 * L2
        L3_2 = x * x + y * y
    
        theta1 = 0
        if L3_2 != 0:
            theta1 = math.acos((L1_2 + L3_2 - L2_2) / (2 * L1 * math.sqrt(L3_2)))
        alpha[1] = math.pi / 2 - math.atan2(y, x) - theta1

        theta2 = math.acos((L1_2 + L2_2 - L3_2) / (2 * L1 * L2))
        alpha[2] = math.pi - theta2

        theta3 = theta1
        alpha[3] = math.atan2(y, x) - theta3 + w
    
        delta_list = []
        for i in range(len(alpha)):
            delta_list.append(abs(alpha[i] - self.servos[i].state.current_pos))
        max_delta = np.max(delta_list)
        delta_list = delta_list / max_delta
        return alpha, delta_list
    
    # Execute servos position.
    def exec_servos_pos(self, x, y, z, w=0, mode=0):
        
        # Safety checking.
        if math.sqrt(x * x + y * y + z * z) > Manipulator.ARM_LENGTH * 2 + Manipulator.HAND_LENGTH:
            print("Cannot move to point(%.2f, %.2f, %.2f)" % (x, y, z))
            return
        
        if self.target_x == x and self.target_y == y and self.target_z == z and self.target_w == w:
            print("Same poisition point(%.2f, %.2f, %.2f)" % (x, y, z))
            return
        
        try:
            alpha, delta_list = self.calc_servos_pos2(x, y, z, w, mode)
        except Exception as e:
            print(e)
            return

         # Execute.
        for i in range(len(alpha)):
            self.servos[i].set_speed(Manipulator.MAX_SPEED * delta_list[i])
            self.servos[i].set_radian(alpha[i])
        self.target_x = x
        self.target_y = y
        self.target_z = z
        self.target_w = w
    
    def calc_servos_pos2(self, x, y, z, w=0, mode=0):
        
        print("Goto:", x, y, z, w)

        alpha, delta_list = self.calc_servos_pos(x, y, z, w)

        if mode == 0:       # Normal
            pass
        elif mode == 1:     # Rotate first
            delta_list[0] = Manipulator.MAX_SPEED
        elif mode == 2:     # Rotate last
            delta_list[0] = np.min(delta_list)
        
        return alpha, delta_list
    
    # Arm is moving or not.
    def is_moving(self):
        for servo in self.servos:
            if servo.state.is_moving:
                return True
        return False
    
    # Wait until stop.
    def wait(self):
        time.sleep(1)
        while self.is_moving(): pass
    
    # Open the gripper.
    def open(self):
        self.servos[4].set_radian(-math.pi / 180 * 25)
    
    # Close the gripper.
    def close(self, offset=0):
        self.servos[4].set_radian(math.pi / 180 * max(39 - abs(offset), 0))


# How to use?
if __name__ == "__main__":
    rospy.init_node("home_edu_arm", anonymous=True)
    
    # 1. Create a Manipulator object.
    manipulator = Manipulator()
    
    # 2. Reset arm pose.
    manipulator.reset()
    
    # 3. Action.
    manipulator.exec_servos_pos(10, 5, 0, 45)
    manipulator.wait()

    manipulator.exec_servos_pos(10, 8, 10, 0)
    manipulator.wait()

    manipulator.exec_servos_pos(10, 5, 0, 45)
    manipulator.wait()

    manipulator.exec_servos_pos(10, 8, -10, 0)
    manipulator.wait()

    manipulator.exec_servos_pos(15, 0, 0, 90)
    manipulator.open()
    manipulator.wait()

    manipulator.exec_servos_pos(15, -8, 0, 90)
    manipulator.wait()

    manipulator.close()
    manipulator.wait()

    manipulator.exec_servos_pos(0, 30, 0, -90)
    manipulator.wait()

    manipulator.exec_servos_pos(5, 5, 0, 0)
    manipulator.open()
    manipulator.wait()

    manipulator.close()
    manipulator.wait()
