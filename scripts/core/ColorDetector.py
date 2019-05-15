#!/usr/bin/env python
import rospy
import cv2 as cv
import numpy as np
from Astra import Astra
import math
from Manipulator import Manipulator


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

    def physical_distance(self, depth_image, x, y, angle=0, max_range=25):
        radian = float(angle) * math.pi / 180
        
        real_x = 0
        real_y = 0
        real_z = 0
        
        h, w = depth_image.shape
        flag = False
        e = 0
        while not flag and e < max_range:
            depth = depth_image[max(cy - e, 0):min(cy + e, h), max(cx - e, 0):min(cx + e, w)].copy()
            indices = np.nonzero(depth)
            if len(indices[0]) > 0:
                real_z = np.min(depth[indices])
                flag = True
            else:
                e = e + 1
        
        FOV_H = 60.0
        d = real_z
        lw = d * math.tan(FOV_H / 2 * math.pi / 180)
        lx = float(x) / w * lw * 2 - w / 2
        real_x = lx
        
        FOV_V = 49.5
        d = real_z
        lh = d * math.tan(FOV_V / 2 * math.pi / 180)
        ly = float(y) / h * lh * 2 - h / 2
        real_y = ly
        
        real_x = real_x
        real_y = real_y + real_z * math.sin(radian)
        real_z = real_z * math.cos(radian)
                
        return real_x, real_y, real_z
    

if __name__ == "__main__":
    rospy.init_node("home_edu_camera", anonymous=True)
    rate = rospy.Rate(20)
    
    camera = Astra()
    arm = Manipulator()
    detector = ColorDetector((50, 16, 16), (80, 255, 255))
    key = 0
    while not rospy.is_shutdown():
        rgb_image = camera.rgb_image
        mask = detector.get_mask(rgb_image)
        cnts = detector.find_contours(mask)
        if len(cnts) > 0:
            cv.drawContours(rgb_image, [cnts[0]], 0, (0, 255, 0), 2)
            cx, cy = detector.find_center(cnts[0])
            cv.circle(rgb_image, (cx, cy), 5, (0, 0, 255), -1)

            rx, ry, rz = detector.physical_distance(camera.depth_image, cx, cy, 30)
            # rospy.loginfo("%.1f mm, %.1f mm, %.1f mm" % (rx, ry, rz))
            cv.putText(rgb_image, "%.1fcm, %.1fcm, %.1fcm" % (rx/10, ry/10, rz/10),
                       (cx + 20, cy),
                       cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2,
                       cv.LINE_AA
            )
            
            if key == 32:
                ay = -(ry / 10 - 40.5)
                az = -rx / 10
                ax = rz / 10 - 18 + 6 * math.cos(math.atan2(rx, rz)) * (-rx/abs(rx))
                arm.exec_servos_pos(7, 10, 0, mode=2)
                arm.open()
                arm.wait()
                arm.exec_servos_pos(ax, ay, az, 0, mode=1)
                arm.wait()
                arm.close(30)
                arm.wait()
                arm.exec_servos_pos(7, 15, 0)
                arm.wait()
                arm.exec_servos_pos(ax, ay, az, -15)
                arm.wait()
                arm.open()
                arm.wait()
                arm.exec_servos_pos(7, 10, 0, mode=2)
                arm.wait()
                
            # cv.imshow("depth", camera.depth_image)
        cv.imshow("image", rgb_image)
        key = cv.waitKey(1)
        if key in [ord('q'), 27]:
            break
