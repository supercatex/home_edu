#!/usr/bin/env python

import rospy
import cv2 as cv
import dlib
import numpy as np
from core import Astra as astra

rospy.init_node("home_edu_face_predictor", anonymous=True)

_face_detector = dlib.get_frontal_face_detector()

c = astra("top_camera")

min_ycc = np.array([0, 133, 85], np.uint8)
max_ycc = np.array([255, 180, 125], np.uint8)

lower = np.array([0, 10, 160], dtype="uint8")
upper = np.array([30, 100, 255], dtype="uint8")

def find_face(frame):
    dets, _, _ = _face_detector.run(frame, False)

flag = 0
while not rospy.is_shutdown():
    frame = c.rgb_image
    frame = cv.resize(frame, (1280, 960))
    
    for i, d in enumerate(dets):
        x1 = d.left()
        y1 = d.top()
        x2 = d.right()
        y2 = d.bottom()
        
        cv.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
        
        area1 = int((x2 - x1) * (y2 - y1))
        area1 = area1 / 1000
        # print("area1", area1),
        
        cv.rectangle(frame, (x1 - int((x2 - x1) * 1.5), int(y1 * 0.85)),
                     (x2 + int((x2 - x1) * 1.5), y1 - int((y2 - y1) * 2)), (0, 0, 255), 3)
        
        frame2 = frame[y1 - int((y2 - y1) * 1.5):y1, x1 - int((x2 - x1) * 1.5):x2 + int((x2 - x1) * 1.5)].copy()
        
        print('frame2', frame2.shape)
        
        if frame2.shape[0] == 0 or frame2.shape[1] == 0:
            continue
        
        ycc = cv.cvtColor(frame2, cv.COLOR_BGR2HSV)
        
        skin = cv.inRange(ycc, lower, upper)
        
        opening = cv.morphologyEx(skin, cv.MORPH_OPEN, np.ones((3, 3), np.uint8), iterations=3)
        sure_bg = cv.dilate(opening, np.ones((3, 3), np.uint8), iterations=2)
        
        _, contours, _ = cv.findContours(sure_bg, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_NONE)
        
        for cnt in contours:
            
            area2 = cv.contourArea(cnt)
            
            if area2 < 3000:
                continue
            else:
                area2 = area2 / 1000
                x, y, w, h = cv.boundingRect(cnt)
                
                cv.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 0), 2)
                
                mid_x = x + w / 2
                
                mid_y = y + h / 2
                
                mid = [mid_x, mid_y]
                
                if abs(area2 - area1) < 60:
                    flag = 1
                    print("found hand")
                else:
                    pass
    
    flag = 0
    frame = cv.resize(frame, (640, 480))
    cv.imshow('frame', frame)
    if cv.waitKey(1) in [ord('q'), 27]:
        break

cv.destroyAllWindows()
