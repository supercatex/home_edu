import rospy
import time
import cv2 as cv
import numpy as np
#import FollowMe as follow
from std_msgs.msg import String
from core import Kobuki as kobuki
from core import Speaker as speaker
from core import Speech2Text as speech2text
from core import Astra as astra
#from core import ServiceController as follow
from turtlebot_msgs.srv import SetFollowState
from core import GenderDetection as Gender
from core import Manipulator as manipulator
from manipulator_track import manipulator_track as manipulator_track


def callback(data):
    # return 1 or 0 for true and false
    global flag
    flag = data.data

publisher = rospy.Publisher(
    "/home_edu/facial",
    String,
    queue_size=1,
    latch=True
)

rospy.Subscriber(
    "/home_edu/mission4_signal",
    String,
    callback,
    queue_size=1
)


if __name__ == '__main__':
    i = 0
    flag = False
    rospy.init_node("home_edu_PCMS_creativity_mission", anonymous=True)
    rate = rospy.Rate(20)
    s = speaker(150)
    k = kobuki()

    while True:
        if flag == "1":
            if i == 1:
                k.move(0.4, 0)
                k.clockwise_to(0.6)
                time.sleep(4)
                s.say("Hello, my guest, what can I help?")
                flag = False
                i = i + 1


            elif i == 2:
                k.move(0.4, 0)
                time.sleep(5)
                s.say("I’m a service robot that works in this mall, you can call me server, what can I help?")
                flag = False
                i = i + 1

            elif i == 3:
                s.say("Of course, just follow me!")
                flag = False
                i = i + 1

            elif i == 4:
                k.move(0.4, 0)
                time.sleep(3)
                flag = False
                i = i + 1

            elif i == 5:
                s.say("What kind of shoes do you want to buy?")
                flag = False
                i = i + 1

            elif i == 6:
                s.say("Got it, there are five sneaker shops in this place, they are reebok, champion, Adidas, converse, and puma. I’m going to bring you to reebok first, it’s the nearest. ")
                flag = False
                i = i + 1

            elif i == 7:
                s.say(" By the way, Reebok has a promotion of new sneakers, this is the image of it, you can have a look.")
                flag = False
                i = i + 1

        elif flag == "-1":
            break

        else:
            pass

    print("Oh u have killed me")



