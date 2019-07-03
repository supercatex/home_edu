#!/usr/bin/env python

import rospy
import cv2 as cv
import dlib
import numpy as np
from core import Astra as astra

rospy.init_node("home_edu_face_predictor", anonymous=True)

_face_detector = dlib.get_frontal_face_detector()

c = astra("top_camera")

while not rospy.is_shutdown():
    frame = c.rgb_image.copy()
    frame = cv.resize(frame, (1280, 960))
    dets, _, _ = _face_detector.run(frame, False)

    min_ycc = np.array([0, 133, 85], np.uint8)
    max_ycc = np.array([255, 170, 125], np.uint8)

    for i, d in enumerate(dets):
        x1 = d.left()
        y1 = d.top()
        x2 = d.right()
        y2 = d.bottom()

        if x1 - (x2 - x1) * 2 < 0:
            x1 = int(2 * x2 / 3)
            x2 = int(3 * x1 / 2)
        
        if x1 + (x2 - x1) * 2 > 1280:
            x1 = int(x2 * 2 - 1280)
            x2 = int((x1 + 1280) / 2)

        if y1 - y1 * 0.1 < 0:
            y1 = int(y1 / 0.1)
        
        if y1 - y1 * 2 < 0:
            y1 = int(y1 / 2)
        
        cv.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
        
        area1 = int((x2 - x1) * (y2 - y1))
        area1 = area1 / 1000
        print("area1", area1)

        cv.rectangle(frame, (x1 - (x2 - x1) * 2, y1 - y1 * 0.1), (x1 + (x2 - x1) * 2, y1 - y1*2), (0, 0, 255), 3)

        frame2 = frame[y1 - y1 * 0.1:y1 - y1*2, x1 - (x2 - x1) * 2: x1 + (x2 - x1) * 2].copy()

    #     ycc = cv.cvtColor(frame2, cv.COLOR_BGR2YCR_CB)
    #
    #     skin = cv.inRange(ycc, min_ycc, max_ycc)
    #
    #     opening = cv.morphologyEx(skin, cv.MORPH_OPEN, np.ones((3, 3), np.uint8), iterations=3)
    #     sure_bg = cv.dilate(opening, np.ones((3, 3), np.uint8), iterations=2)
    #
    #     _, contours, _ = cv.findContours(sure_bg, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_NONE)
    #
    #     for cnt in contours:
    #
    #         area2 = cv.contourArea(cnt)
    #         print("area2", area2)
    #         if area2 < 700:
    #             continue
    #         else:
    #             area2 = area2 / 1000
    #             x, y, w, h = cv.boundingRect(cnt)
    #
    #             cv.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 0), 2)
    #
    #             if abs(area2 - area1) < 60:
    #                 print("found hand")
    #             else:
    #                 pass
    #
    frame = cv.resize(frame, (640, 480))
    cv.imshow('frame', frame)
    if cv.waitKey(1) in [ord('q'), 27]:
        break

cv.destroyAllWindows()
