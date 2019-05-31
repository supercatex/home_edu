#!/usr/bin/env python


import rospy
# import roslib
import cv2 as cv
import dlib
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from core import Astra as astra
from core import Kobuki as kobuki
# from pyttsx3 import init
from core import Speaker as speaker
import data
import time
import speech_recognition as sr

# import os


def ambient_noice():
    global r
    with sr.Microphone() as source:
        r.adjust_for_ambient_noise(source)


def Recognize_gender(image, model_values):
    blob = cv.dnn.blobFromImage(image, 1, (227, 227), model_values, swapRB=False)
    gender_net.setInput(blob)
    gender_preds = gender_net.forward()
    gender = gender_list[gender_preds[0].argmax()]
    return gender


def turn_180_degree(chassis):
    z = chassis.imu.orientation.z
    print(z)
    if not abs(z) >= 0.99:
        if z < 1:
            chassis.move(0, 0.3)

        elif z > 1:
            chassis.move(0, -0.3)
    else:
        return 0


def speech_to_text():
    global r
    with sr.Microphone() as source:
        audio = r.listen(source)
    try:
        text = r.recognize_google(audio, language="en-US")
        P.say("You said, {}".format(text))
        return text
    except Exception as e:
        print(e)
        P.say("Please say one more time")
        return ""


def Answer_question(text):
    try:
        for key in data.question.keys():
            c = key.split(" ")
            text = text.lower()
            found_count = 0
            for i in c:
                i = i.lower()
                if i in text:
                    found_count += 1
            if found_count == len(c):
                return data.question[key]
        return 'Please ask again'

    except Exception as e:
        print(e)
        return "Please ask again"


# def speak(*args):
#     for arg in args:
#         print(arg)
#         engine.say(arg)
#         engine.runAndWait()
#

if __name__ == '__main__':
    rospy.init_node('Home_first_mission', anonymous=True)
    # face_cascade = cv.CascadeClassifier('/home/mustar/pcms/src/home_edu/scripts/libs/haarcascade_frontalface_default.xml')

    # Init the speaker cam and kobuki
    P = speaker()

    # base_path = os.path.abspath("./")
    time.sleep(4)

    P.say("I will start now")
    male_count, female_count = 0, 0

    r = sr.Recognizer()

    MODEL_MEAN_VALUES = (78.4263377603, 87.7689143744, 114.895847746)
    gender_list = ['Male', 'Female']
    # engine = init()

    gender_net = cv.dnn.readNetFromCaffe(
        '/home/mustar/pcms/src/home_edu/scripts/libs/deploy_gender.prototxt',
        '/home/mustar/pcms/src/home_edu/scripts/libs/gender_net.caffemodel'
    )


    rate = rospy.Rate(20)

    _detector = dlib.get_frontal_face_detector()
    image_path = "/home/mustar/pcms/src/home_edu/scripts/temp.jpg"

    c = astra("cam2")
    chassis = kobuki()
    cv.namedWindow("image")
    cv.setMouseCallback("image", c.mouse_callback)

    while True:
        a = turn_180_degree(chassis)
        #a = 0
        if a == 0:
            P.say("Hello, may I have your attention please? I am going to take a photo for you, please ready")
            time.sleep(2)
            P.say("Please look at my top camera. Ready? 3, 2, 1, chieese")
            frame = c.rgb_image
            image = cv.resize(frame, (1280, 960))
            time.sleep(1)
            cv.imwrite("/home/mustar/pcms/src/home_edu/scripts/temp.jpg", image)
            P.say("I've just take a photo of you")
            break

    # if c.rgb_image is None or c.depth_image is None:
    #     continue

    # frame = c.rgb_image
    # cv.imwrite("/home/mustar/pcms/src/home_edu/scripts/temp.jpg", frame)

    image = cv.imread("/home/mustar/pcms/src/home_edu/scripts/temp.jpg", cv.IMREAD_COLOR)
    # cv.imshow('image', image)
    # cv.waitKey(1)
    # gray = cv.cvtColor(c.rgb_image, cv.COLOR_RGB2GRAY)

    dets, _, _ = _detector.run(image, False)
    print(dets)

    face_count = 0
    for i, d in enumerate(dets):
        x1 = d.left()
        y1 = d.top()
        x2 = d.right()
        y2 = d.bottom()
        if x1 < 0 or y1 < 0: continue
        face_count += 1
        # print(y1, y2, x1, x2)
        face_img = image[y1:y2, x1:x2].copy()
        # print(face_img.shape)
        # cv.imshow("face_img", face_img)
        # cv.waitKey(0)
        # cv.imshow("face_img", face_img)
        # cv.waitKey(0)
        # blob = cv.dnn.blobFromImage(face_img, 1, (227, 227), MODEL_MEAN_VALUES, swapRB=False)
        # print(blob)
        color = (255, 0, 0)

        # Recognize gender
        # gender_net.setInput(blob)
        # gender_preds = gender_net.forward()
        # gender = gender_list[gender_preds[0].argmax()]
        gender = Recognize_gender(face_img, MODEL_MEAN_VALUES)
        print("Face in picture's gender is {}".format(gender))

        if gender == 'Male':
            color = (255, 0, 0)
            male_count += 1

        if gender == 'Female':
            female_count += 1
            color = (0, 0, 255)

        cv.rectangle(image, (x1, y1), (x2, y2), color, 2)

    # faces = face_cascade.detectMultiScale(gray, minSize=(50, 50))
    # for (x, y, w, h) in faces:
    #     cv.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 0), 2)
    #     print("Recognize faces: {}".format(len(faces)))

    cv.imshow('image', image)
    cv.waitKey(1)
    # if cv.waitKey(1) == ord('q'):
        # break
    # rate.sleep()

    P.say("I see {} person, {} man, and {} women in this picture".format(str(face_count), str(male_count), str(female_count)))

    cv.destroyAllWindows()

    ambient_noice()

    P.say("I'm ready")
    # Question_count = 0
    # Question part
    while not rospy.is_shutdown():
        P.say("Please ask")

        operator_question = speech_to_text()

        P.say(Answer_question(operator_question))
        #
        # if Question_count == 5:
        #     break
        #
        # Question_count += 1
