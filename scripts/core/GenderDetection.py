#!/usr/bin/env python

import cv2 as cv
import numpy as np
import os
import rospy
import dlib
from Astra import Astra as astra


class GenderDetection(object):
    def __init__(self, gender_lib_prototxt="../libs/deploy_gender.prototxt", gender_lib_model="../libs/gender_net.caffemodel"):
        self.gender_lib_prototxt = os.path.abspath(gender_lib_prototxt)
        self.gender_lib_model = os.path.abspath(gender_lib_model)
        self.MODEL_MEAN_VALUES = (78.4263377603, 87.7689143744, 114.895847746)
        self.gender_list = ['male', 'female']
        self.gender_net = cv.dnn.readNetFromCaffe(
            self.gender_lib_prototxt,
            self.gender_lib_model
        )
        self._detector = dlib.get_frontal_face_detector()

    def detect_face(self, image):
        face_position = list()
        dets, _, _ = self._detector.run(image, False)

        for i, d in enumerate(dets):
            x1 = d.left()
            y1 = d.top()
            x2 = d.right()
            y2 = d.bottom()

            if x1 < 0 or y1 < 0: continue

            face_position.append([x1, y1, x2, y2])

        return face_position

    def predict_gender(self, face_image):
        blob = cv.dnn.blobFromImage(face_image, 1, (277, 277), self.MODEL_MEAN_VALUES, swapRB=False)
        self.gender_net.setInput(blob)
        gender_pred = self.gender_net.forward()
        gender = self.gender_list[gender_pred[0].argmax()]
        return gender


if __name__ == '__main__':
    rospy.init_node("home_edu_gender_detection", anonymous=True)
    rate = rospy.Rate(20)

    c = astra()
    g = GenderDetection()

    while not rospy.is_shutdown():
        frame = c.rgb_image

        face_position = g.detect_face(frame)
        for x1, y1, x2, y2 in face_position:
            cv.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 3)

        cv.imshow('frame', frame)

        if cv.waitKey(1) == ord('q'):
            break

    cap.release()
    cv.destroyAllWindows()
