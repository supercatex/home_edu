#!/usr/bin/env python

import rospy
import time
import cv2 as cv
import numpy as np
from std_msgs.msg import String
from core import RobotChassis as chassis
from core import Kobuki as kobuki
from core import Speaker as speaker
from core import Speech2Text as speech2text
from core import Astra as astra
from core import GenderDetection as Gender
from core import Manipulator as manipulator
from core import PH_Follow_me as PH_Follow_me
from manipulator_track import manipulator_track as manipulator_track
import main
from os import system

system("rm -rf /home/mustar/pcms/src/home_edu/scripts/face_images/*")

publisher = rospy.Publisher(
    "/home_edu/facial",
    String,
    queue_size=1,
    latch=True
)


def depth_detect(frame):
    try:
        image = frame[100:540, 100:380].copy()
    
        index = image[np.nonzero(image)]
        
        if index is None:
            return False
        else:
            min = np.min(index)
    
        print(min)
        if min <= 750 and min >= 350:
            return True
        else:
            return False
    
    except Exception:
        print("error")

        
def listen_callback(data):
    global msg
    msg = data.data
    print(msg)


if __name__ == '__main__':
    kdata = main.load_data("/home/mustar/pcms/src/home_edu/scripts/second_keyword.txt")
    msg = ' '
    rospy.init_node("home_edu_PCMS_Second_mission", anonymous=True)
    rate = rospy.Rate(20)
    s = speaker(150, 1.5)
    
    rospy.Subscriber(
        "/home_edu_Listen/msg",
        String,
        listen_callback,
        queue_size=1
    )
    t = speech2text()
    t.ambient_noise()
    chassis = chassis()
    
    c = astra("top_camera")
    f = PH_Follow_me()
    m = manipulator()
    k = kobuki()
    obj = manipulator_track("brown")
    print("started")
    time.sleep(20)
    m.exec_servos_pos(10, 15, 0, -30)
    s.say("hello, I'm your assistant", "happy-1")

    _listen_publisher = rospy.Publisher("/home_edu_Listen/situation", String, queue_size=1)
    
    s.say("Please stand in front of me", "happy-1")
    
    while True:
        # frame = c.rgb_image
        status = depth_detect(c.depth_image)
        if status == True:
            print("ok")
            s.say("ok I have found you")
            break

    s.say("please stand still and say follow me")

    _listen_publisher.publish("true")
    goal = [[-4.89, 2.46, -2.79], [-5.55, -1.73, 1.57], [-2.72, 1.89, -2.79]]
    
    flag = 0
    flag2 = 0
    start_time = time.time()
    print(kdata)
    while True:
        answer = main.answer_question_from_data(msg, kdata)['answer']
        # print("answer:", answer)
        if answer == 'follow':
            flag = 1
            flag2 = 1
            
        elif answer == 'stop':
            k.move(0, 0)
            break
        
        if flag2 == 1:
            _listen_publisher.publish("false")
            _listen_publisher.publish("wait")
            _listen_publisher.publish("true")
            flag2 = 0

        forward_speed, turn_speed = f.follow(c.depth_image, flag==1)
        k.move(forward_speed, turn_speed)
        cv.waitKey(1)
        
    _listen_publisher.publish("false")
    px, py, pz = chassis.get_current_pose()
    print(px, py, pz)
    print('finished append')
    s.say("the robot have stopped, okay this is a futuristic tesla car")
    
    signal = True
    
    s.say("please give me the bag")
    
    m.open()
    
    start_time2 = time.time()
    while not rospy.is_shutdown():
    
        print(obj.area)
        frame, image = c.depth_image, c.rgb_image
    
        image, status, x, y, z, alpha = obj.run(c.rgb_image, c.depth_image)
    
        if status == True:
            cv.imshow("image", image)
            cv.waitKey(1)
        else:
            cv.imshow("image", image)
            cv.waitKey(1)
            continue
        
        if time.time() - start_time2 > 20:
            break
        else:
            pass
        if signal == True:
            if obj.area < 6000 or obj.area is None:
                signal = False
                start_time = time.time()
                cv.imshow("image", image)
                cv.waitKey(1)
                continue
            else:
                m.exec_servos_pos(x, y, z, -60)
                print('mani x, y, z:', x, y, z, -60)
                cv.imshow("image", image)
                cv.waitKey(1)
                continue
        else:
            if obj.area < 6000 or obj.area is None:
                if time.time() - start_time > 3:
                    break
                else:
                    cv.imshow("image", image)
                    cv.waitKey(1)
                    continue
            else:
                signal = True
                cv.imshow("image", image)
                cv.waitKey(1)
                continue
    print("end loop")

    time.sleep(1)

    m.close()

    m.exec_servos_pos(5, 25, 0, -30)
    
    s.say("gripped")
    s.say("please say the location of the bag")
    _listen_publisher.publish("true")

    while True:
        place = main.answer_question_from_data(msg, kdata)
        print(place)
        place = str(place['answer']).lower()
        if place == 'kitchen':
            i = 0
            break
        elif place == 'bedroom':
            i = 1
            break
        elif place == 'living room':
            i = 2
            break
        elif len(place) > 20:
            s.say("please tell me where should i put the bag")

    _listen_publisher.publish("false")
    s.say("you said " + str(place))
    
    s.say("Please stand away from me", "wink")
    time.sleep(1.5)
    print(goal[i][0], goal[i][1], goal[i][2])
    chassis.move_to(goal[i][0], goal[i][1], goal[i][2])
    s.say("arrived to goal")

    m.exec_servos_pos(20, 15, 0, -30)

    m.exec_servos_pos(20, 10, 0, 35)

    m.wait()
    m.open()
    time.sleep(2)

    m.exec_servos_pos(12, 15, 0, -30)
    
    s.say("I have put the bag on the floor", "wink")
    
    s.say("i am now turning around to find you")
    
    
    g = Gender("/home/mustar/pcms/src/home_edu/scripts/libs/deploy_gender.prototxt", "/home/mustar/pcms/src/home_edu/scripts/libs/gender_net.caffemodel")
    
    cam = astra("top_camera")

    while not rospy.is_shutdown():
        frame = cam.rgb_image
        position, frame, face_images = g.detect_face(frame)
        if len(position) == 0:
            print('moving')
            k.move(0, 0.4)
            cv.imshow("frame", frame)
        else:
            p1 = position[0]
            x_val = (p1[2] / 2 + p1[0])
            print(x_val, frame.shape)
            if x_val < 300:
                k.move(0, 0.3)
            elif x_val > 340:
                k.move(0, -0.3)
            else:
                print("ok")
                for i in range(len(face_images)):
                    cv.imwrite("./face_images/face%d.jpg" % i, face_images[i])
                break
            cv.imshow("frame", frame)
        cv.waitKey(1)

    start_time = time.time()
    while True:
        if time.time() - start_time < 2.5:
            k.move(0.2, 0)
        else:
            break
                
    start_time = time.time()
            
    while True:
        time.sleep(1)
        s.say("ok, i have found you")
        s.say("please follow me to the garage and stand behind me", "wink")
        chassis.move_to(px, py, pz)
        s.say("arrived to the garage")
        break
        
    print("end of program")
    s.say("finished task", "wink")
    
