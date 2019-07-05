#!/usr/bin/env python
import rospy
from core import Astra as astra
import cv2 as cv
from core import Kobuki as kobuki
from core import GenderDetection as Gender
import time

if __name__ == '__main__':
    msg = ' '
    rospy.init_node("home_edu_PCMS_Second_mission", anonymous=True)
    rate = rospy.Rate(20)
    cam = astra("top_camera")
    k = kobuki()
    g = Gender("/home/mustar/pcms/src/home_edu/scripts/libs/deploy_gender.prototxt",
               "/home/mustar/pcms/src/home_edu/scripts/libs/gender_net.caffemodel")

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
                # time.sleep(1)
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