#!/usr/bin/env python
import rospy
import os
import cv2 as cv
import numpy as np
from std_msgs.msg import String


class FacialDisplay(object):
    
    def __init__(self, path="emoji", name="happy", fullscreen=False):
        self.dir = os.path.dirname(os.path.abspath(__file__))
        self.path = path
        self.name = name
        self.fullscreen = fullscreen
        self.image = None
        self.message = ""
        self.load_image(name, self.message)

        self.subscriber = rospy.Subscriber(
            "/home_edu/facial",
            String,
            self.callback,
            queue_size=1
        )
    
    def load_image(self, name, message):
        self.name = name
        self.message = message
        img = cv.imread(
            "%s/%s/%s.png" % (self.dir, self.path, self.name),
            cv.IMREAD_UNCHANGED
        )
        bg = cv.imread(
            "%s/%s/%s.jpg" % (self.dir, self.path, "1"),
            cv.IMREAD_COLOR
        )
        h, w, c = img.shape
        fw = 960
        fh = 600
        frame = np.zeros((fh, fw, 3), dtype="uint8")
        
        frame = cv.resize(bg, (fw, fh))
        
        ox = int((fw - w) / 2)
        oy = 10
        for i in range(min(c, 3)):
            if c <= 3:
                frame[oy:oy+h, ox:ox+w, i] = img[:, :, i]
            else:
                c1 = frame[oy:oy + h, ox:ox + w, i] * (1 - img[:, :, 3] / 255)
                c2 = img[:, :, i] * (img[:, :, 3] / 255)
                frame[oy:oy + h, ox:ox + w, i] = c1 + c2

        cv.putText(
            frame, "Press 'q' or 'Esc' to quit.", (740, 30),
            cv.FONT_HERSHEY_SIMPLEX,
            0.5, (255, 255, 255), 1,
            cv.LINE_AA
        )

        cv.putText(
            frame, "Icon designed by Roundicons from Flaticon", (700, 10),
            cv.FONT_HERSHEY_SIMPLEX,
            0.35, (150, 150, 150), 1,
            cv.LINE_AA
        )
        
        if len(message) < 60:
            size = 1
        elif len(message) < 120:
            size = 0.5
        else:
            size = 0.35
        
        cv.putText(
            frame, message, (5, oy + h + 50),
            cv.FONT_HERSHEY_SIMPLEX,
            size, (255, 255, 255), 1,
            cv.LINE_AA
        )
        
        self.image = frame
    
    def callback(self, data):
        cmd = data.data.split(":")
        self.load_image(cmd[0], cmd[1])
    
    def run(self):
        window_name = "Facial display"
        cv.namedWindow(window_name, cv.WND_PROP_FULLSCREEN)
        if self.fullscreen:
            cv.setWindowProperty(window_name, cv.WND_PROP_FULLSCREEN, cv.WINDOW_FULLSCREEN)
        
        while not rospy.is_shutdown():
            cv.imshow(window_name, self.image)
            key = cv.waitKey(1)
            if key in [ord('q'), 27]:
                break


if __name__ == "__main__":
    rospy.init_node("home_edu_facial_display", anonymous=True)
    fd = FacialDisplay(fullscreen=True)
    fd.run()
