#!/usr/bin/env python
from core import Astra as astra
from core import Manipulator as Manipulator
import numpy as np
import cv2
import rospy
import math
import time


class manipulator_track(object):
    def __init__(self, color):
        self.area = 10
    
        self.max_range = 20
    
        self.mani_length = 30
    
        self.cameraH = 60 * math.pi / 180
    
        self.cameraV = 49.5 * math.pi / 180
    
        if color == "brown":
            self.lower = [0, 20, 100]
            self.upper = [50, 110, 210]
    
        elif color == "white":
            self.lower = [95, 0, 200]
            self.upper = [150, 50, 255]
        else:
            self.lower = [0, 30, 110]
            self.upper = [50, 90, 200]
    
    def color_detect(self, rgb_image, depth_image):
        lower = np.array(self.lower, dtype="uint8")
    
        upper = np.array(self.upper, dtype="uint8")
        
        image = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(image, lower, upper)
    
        ret, thresh = cv2.threshold(mask, 40, 255, 0)
        im2, contours, hierarchy = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    
        if len(contours) != 0:
            c = max(contours, key=cv2.contourArea)
    
            area = cv2.contourArea(c)
    
            self.area = area
    
            x, y, w, h = cv2.boundingRect(c)
    
            cv2.rectangle(rgb_image, (x, y), (x + w, y + h), (0, 255, 0), 3)
    
            flag = False
    
            e = 0
    
            y = (y + h / 2)
    
            x = (x + w / 2)
             
            cv2.circle(rgb_image, (x, y), 5, (0, 255, 255), -1)
            h2, w2 = depth_image.shape
    
            real_z = 0
    
            while not flag and e < self.max_range:
                depth = depth_image[max(y - e, 0):min(y + e, h2), max(x - e, 0):min(x + e, w2)].copy()
                indices = np.nonzero(depth)
                if len(indices[0]) > 0:
                    real_z = np.min(depth[indices])
                    flag = True
                else:
                    e = e + 1
    
            z = real_z
            
            if z >= 1500:
                return 1, 1, 1
            else:
                pass
            mid = [x, y, z]
            
            if mid is None or mid == []:
                return rgb_image, [10, 10, 10], y - h / 2
            else:
                print('middle_virture:', mid)
                return rgb_image, mid, y - h / 2
        else:
            return 0, 0, 0
    
    def calculation(self, mid):
        try:
            x0 = mid[0]
            y0 = mid[1]
            z0 = mid[2] / 10
    
            # x1 = z0
            # y1 = y0
            # z1 = x0
        
            wR = 2 * z0 * math.tan(self.cameraH / 2)
            hR = 2 * z0 * math.tan(self.cameraV / 2)
            xR = (wR * x0) / 640 - wR / 2
            yR = (hR * y0) / 480 - hR / 2
            zR = z0
        
            print("Real x, y, z, w, h", xR, yR, zR, wR, hR)
            return xR, yR, zR
        except TypeError:
            return 0, 0, 0
    
    def physical_distance(self, x, y, z):
        y0 = (36.5 - y)
        x0 = z - 18
        z0 = -x
    
        l = self.mani_length
        alpha = math.atan2(y0, math.sqrt(x0 * x0 + z0 * z0))
        beta = math.atan2(z0, x0)
        print(alpha, beta)
    
        l0 = l * math.cos(alpha)
        Px = l0 * math.cos(beta)
        Py = l * math.sin(alpha)
        Pz = l0 * math.sin(beta)
    
        return Px, Py, Pz, alpha
    
    def run(self, rgb_image, depth_image):
        if rgb_image is None or depth_image is None:
            
            return rgb_image, False, 0, 0, 0, 0
        
        else:
            frame, mid, y0 = self.color_detect(rgb_image, depth_image)
            if frame is None:
                return frame, False, 0, 0, 0, 0

            Rx, Ry, Rz = self.calculation(mid)
            
            x, y, z, a = self.physical_distance(Rx, Ry, Rz)
            
            return frame, True, x, y, z, a
    

if __name__ == "__main__":
    rospy.init_node("home_edu_manipulator_track", anonymous=True)
    rate = rospy.Rate(20)
    
    c = astra("top_camera")
    
    m = Manipulator()
    
    obj = manipulator_track("brown")
    
    signal = True
    
    m.reset()
    
    m.wait()
    
    m.open()
    while not rospy.is_shutdown():
    
        print(obj.area)
        frame, image = c.depth_image, c.rgb_image
    
        image, status, x, y, z, alpha = obj.run(c.rgb_image, c.depth_image)
    
        if status == True:
            cv2.imshow("image", image)
            cv2.waitKey(1)
        else:
            cv2.imshow("image", image)
            cv2.waitKey(1)
            continue

        if signal == True:
            if obj.area < 6000 or obj.area is None:
                signal = False
                start_time = time.time()
                cv2.imshow("image", image)
                cv2.waitKey(1)
                continue
            else:
                m.exec_servos_pos(x, y, z, -60)
                print('mani x, y, z:', x, y, z, -60)
                cv2.imshow("image", image)
                cv2.waitKey(1)
                continue
        else:
            if obj.area < 6000 or obj.area is None:
                if time.time() - start_time > 3:
                    break
                else:
                    cv2.imshow("image", image)
                    cv2.waitKey(1)
                    continue
            else:
                signal = True
                cv2.imshow("image", image)
                cv2.waitKey(1)
                continue
    print("end loop")
    
    time.sleep(1)
    
    m.close()
    
    m.exec_servos_pos(5,25,0,-30)
    
    time.sleep(5)

   
    m.exec_servos_pos(20, 15, 0, -30)

    m.exec_servos_pos(20, 10, 0, 35)

    m.wait()
    m.open()
    time.sleep(2)

    m.exec_servos_pos(12, 15, 0, -30)

cv2.destroyAllWindows()