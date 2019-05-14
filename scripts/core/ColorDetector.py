#!/usr/bin/env python
import rospy
import cv2 as cv
import numpy as np
from Astra import Astra


class ColorDetector(object):
    
    def __init__(self, lower, upper, min_size=1000):
        self.lower = lower
        self.upper = upper
        self.min_size = min_size
        
    def get_mask(self, rgb_image):
        hsv = cv.cvtColor(rgb_image, cv.COLOR_RGB2HSV)
        mask = cv.inRange(hsv, self.lower, self.upper)
        kernel = np.ones((5, 5), np.uint8)
        mask = cv.morphologyEx(mask, cv.MORPH_OPEN, kernel)
        return mask
    
    def find_contours(self, mask):
        _, contours, _ = cv.findContours(
            mask,
            cv.RETR_EXTERNAL,
            cv.CHAIN_APPROX_SIMPLE
        )
        results = []
        for cnt in contours:
            area = cv.contourArea(cnt)
            if area > self.min_size:
                results.append(cnt)
        results.sort(key=cv.contourArea, reverse=True)
        return results
    
    def find_center(self, cnt):
        m = cv.moments(cnt)
        if m["m00"] != 0:
            x = int(np.round(m["m10"] / m["m00"]))
            y = int(np.round(m["m01"] / m["m00"]))
            return x, y
        return 0, 0


if __name__ == "__main__":
    rospy.init_node("home_edu_camera", anonymous=True)
    rate = rospy.Rate(20)
    
    camera = Astra()
    detector = ColorDetector((50, 16, 16), (80, 255, 255))
    while not rospy.is_shutdown():
        rgb_image = camera.rgb_image
        mask = detector.get_mask(rgb_image)
        cnts = detector.find_contours(mask)
        if len(cnts) > 0:
            cv.drawContours(rgb_image, [cnts[0]], 0, (0, 255, 0), 2)
            cx, cy = detector.find_center(cnts[0])
            cv.circle(rgb_image, (cx, cy), 5, (0, 0, 255), -1)

            h, w = camera.depth_image.shape[0:2]
            flag = False
            e = 0
            while not flag and e < 25:
                depth = camera.depth_image[max(cy-e, 0):min(cy+e, h), max(cx-e, 0):min(cx+e, w)].copy()
                index = np.nonzero(depth)
                if len(index[0]) > 0:
                    d = np.min(depth[index])
                    rospy.loginfo("%d mm" % (d))
                    cv.putText(rgb_image, str(d / 10) + "cm", (cx + 20, cy),
                               cv.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2,
                               cv.LINE_AA
                    )
                    flag = True
                else:
                    e = e + 1
            # cv.imshow("depth", camera.depth_image)
        cv.imshow("image", rgb_image)
        if cv.waitKey(1) in [ord('q'), 27]:
            break
