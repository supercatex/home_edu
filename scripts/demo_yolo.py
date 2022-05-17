#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from robot_vision_msgs.msg import BoundingBoxes
from geometry_msgs.msg import Twist


def callback_image(msg):
    global _frame
    _frame = CvBridge().imgmsg_to_cv2(msg, "bgr8")

def callback_boxes(msg):
    global _boxes
    _boxes = msg.bounding_boxes
    
def callback_depth(msg):
    global _depth
    _depth = CvBridge().imgmsg_to_cv2(msg, "passthrough")

def turn(v):
    global _cmd_vel
    msg = Twist()
    msg.linear.x = 0.0
    msg.angular.z = v
    _cmd_vel.publish(msg)
    
def move(v):
    global _cmd_vel
    msg = Twist()
    msg.linear.x = v
    msg.angular.z = 0.0
    _cmd_vel.publish(msg)

def find_bottle():
    global _boxes
    cx, cy = None, None
    for box in _boxes:
        if box.Class == "bottle":
            cx = (box.xmin + box.xmax) // 2
            cy = (box.ymin + box.ymax) // 2
            break
    return cx, cy


if __name__ == "__main__":
    rospy.init_node("demo")
    rospy.loginfo("demo node start!")
    
    _frame = None
    rospy.Subscriber("/camera/rgb/image_raw", Image, callback_image)
    _boxes = []
    rospy.Subscriber("/yolo_ros/bounding_boxes", BoundingBoxes, callback_boxes)
    _depth = None
    rospy.Subscriber("/camera/depth/image_raw", Image, callback_depth)
    
    _cmd_vel = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
    
    rospy.sleep(1)
    
    _status = 0
    while not rospy.is_shutdown():
        rospy.Rate(20).sleep()
        if _frame is None: continue
        if _depth is None: continue
        
        if _status == 0:
            turn(0.2)
            
            cx, cy = find_bottle()
            if cx is not None:
                _status = 1
            continue
        
        if _status == 1:
            cx, cy = find_bottle()
            if cx is not None:
                cv2.circle(_frame, (cx, cy), 5, (0, 255, 0), -1)
                
                h, w, c = _frame.shape
                e = w // 2 - cx
                v = 0.000625 * e
                turn(v)
                
                if abs(e) < 5:
                    _status = 2
            continue
        
        if _status == 2:
            cx, cy = find_bottle()
            if cx is not None:
                d = _depth[cy][cx]
                rospy.loginfo("d: %d" % d)
                
                if d > 0 and d < 3000:
                    if d < 800:
                        _status = 3
                        continue
                    move(0.1)
                else:
                    move(0.0)
        
        if _status == 3:
            break

        cv2.imshow("frame", _frame)
        key_code = cv2.waitKey(1)
        if key_code in [27, ord('q')]:
            break
    
    rospy.loginfo("demo node end!")
    
