#!/usr/bin/env python

import rospy
import roslib
import cv2 as cv
import dlib
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from libs import ROS_Topic as T
from Astra import Astra as astra
from Kobuki import Kobuki as kobuki
from pyttsx3 import init


def speak(*args):
    for arg in args:
        print(arg)
        engine.say(arg)
        engine.runAndWait()


if __name__ == '__main__':
    # face_cascade = cv.CascadeClassifier('/home/mustar/pcms/src/home_edu/scripts/libs/haarcascade_frontalface_default.xml')

    male_count, female_count = 0, 0

    MODEL_MEAN_VALUES = (78.4263377603, 87.7689143744, 114.895847746)
    gender_list = ['Male', 'Female']
    engine = init()

    gender_net = cv.dnn.readNetFromCaffe(
        '/home/mustar/pcms/src/home_edu/scripts/libs/deploy_gender.prototxt',
        '/home/mustar/pcms/src/home_edu/scripts/libs/gender_net.caffemodel'
    )

    rospy.init_node('Home_first_mission', anonymous=True)

    rate = rospy.Rate(20)

    _detector = dlib.get_frontal_face_detector()
    c = astra("cam2")
    # chassis = kobuki()
    cv.namedWindow("image")
    cv.setMouseCallback("image", c.mouse_callback)
    while not rospy.is_shutdown():
        if c.rgb_image is None or c.depth_image is None:
            continue

        frame = c.rgb_image
        gray = cv.cvtColor(c.rgb_image, cv.COLOR_RGB2GRAY)

        dets, _, _ = _detector.run(frame, False)

        face_count = 0
        for i, d in enumerate(dets):
            x1 = d.left()
            y1 = d.top()
            x2 = d.right()
            y2 = d.bottom()
            if x1 < 0 or y1 < 0: continue
            face_count += 1
            # print(y1, y2, x1, x2)
            face_img = frame[y1:y2, x1:x2].copy()
            # print(face_img.shape)
            # cv.imshow("face_img", face_img)
            # cv.waitKey(0)
            face_img = cv.resize(face_img, (227, 227))
            # cv.imshow("face_img", face_img)
            # cv.waitKey(0)
            blob = cv.dnn.blobFromImage(face_img, 1, (227, 227), MODEL_MEAN_VALUES, swapRB=False)
            # print(blob)
            color = (255, 0, 0)

            # Recognize gender
            gender_net.setInput(blob)
            gender_preds = gender_net.forward()
            gender = gender_list[gender_preds[0].argmax()]
            print("Face in picture's gender is {}".format(gender))

            if gender == 'Male':
                color = (255, 0, 0)
                male_count += 1

            elif gender == 'Female':
                female_count += 1
                color = (0, 0, 255)

            cv.rectangle(frame, (x1, y1), (x2, y2), color, 2)

        # faces = face_cascade.detectMultiScale(gray, minSize=(50, 50))
        # for (x, y, w, h) in faces:
        #     cv.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 0), 2)
        #     print("Recognize faces: {}".format(len(faces)))

        cv.imshow('image', frame)
        if cv.waitKey(1) == ord('q'):
            break
        rate.sleep()

    speak("I see {} man in this picture and {} women in this picture".format(str(male_count), str(female_count)))

    cv.destroyAllWindows()
