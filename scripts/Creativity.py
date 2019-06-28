#!/usr/bin/env python
import rospy
import time
import cv2 as cv
import numpy as np
#import FollowMe as follow
from std_msgs.msg import String
from core import RobotChassis as chassis
from core import Kobuki as kobuki
from core import Speaker as speaker
from core import Speech2Text as speech2text
from core import Astra as astra
#from core import ServiceController as follow
from turtlebot_msgs.srv import SetFollowState
from core import GenderDetection as Gender
from core import Manipulator as manipulator
from core import PH_Follow_me as PH_Follow_me
from manipulator_track import manipulator_track as manipulator_track
import main

publisher = rospy.Publisher(
    "/home_edu/facial",
    String,
    queue_size=1,
    latch=True
)
def wavehand():
    shoulder.set_radian(45*math.pi/180)
    shoulder.set_radian(-45*math.pi/180)




def listen_callback(data):
    global msg
    msg = data.data
    print(msg)

if __name__ == '__main__':
    kdata = main.load_data("/home/mustar/pcms/src/home_edu/scripts/misison4_key.txt")
    msg = ' '
    rospy.init_node("home_edu_PCMS_Second_mission", anonymous=True)
    rate = rospy.Rate(20)
    s = speaker()
    
    rospy.Subscriber(
        "/home_edu_Listen/msg",
        String,
        listen_callback,
        queue_size=1
    )
    t = speech2text()

    chassis = chassis()

    c = astra("top_camera")
    f = PH_Follow_me()
    m = manipulator()
    k = kobuki()
    obj = manipulator_track("red")
    #m.exec_servos_pos(10, 15, 0, -30)
    print("started")
    s.say("hello, Do you need help", "happy-1")
    _listen_publisher = rospy.Publisher("/home_edu_Listen/situation", String, queue_size=1)

    _listen_publisher.publish("true")
    
    #goal = [[-3.36, 8.17, 0.0025], [1.9, 5.51, -0.00137], [0.00489, -0.0209, -0.00137]]
    
    flag = 0
    '''
    flag1=nika
    flag2=pumi
    flag3=fans
    flag4=basketball shoes
    flag5=running shoes
    flag6=skateboard
    '''
    print(kdata)
    distance = 0
    while True:
        t.ambient_noise(2)
        break

    while True:
        answer = main.answer_question_from_data(msg, kdata)['answer']
        print(answer)
        if answer == "nike":
            flag = 1
            distance = 5
            
        elif answer == "puma":
            flag = 2
            distance = 10
            
        elif answer == "adidas":
            flag = 3
            distance = 15
            
        elif answer == "don't" or answer == "do not":
            stop = 1
            
        elif answer == "basketball":
            flag = 4
            s.say("you can buy basketball equipments at nika")
            
        elif answer == "running":
            flag = 5
            s.say("you can buy basketball equipments at pumi")

        elif answer == "skateboard":
            flag = 6
            s.say("you can buy basketball equipments at fans")
        
		elif answer == "find":
			s.say("Hello, I am turtlebot, who is the server here, where do you want to go? I can bring you there")
			
		elif answer == "nike":
			s.say("ok, follow me") 
			
		elif answer == "if you buy two items in Nike, you will get a 10%off. "
        k.move(0.3, 0)
        time.sleep(distance)

        s.say("arrived, have a nice day")
        k.move(0, 180)
        time.sleep(distance)

        k.move(distance, 0)
        if stop == 1 or arrive = 1:
            break
    _listen_publisher.publish("false")
