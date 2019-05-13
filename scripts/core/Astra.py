#!/usr/bin/env python
import cv2 as cv
import numpy as np
import rospy
from ROS_Topic import ROS_Topic_Astra as T
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

'''

# Default ROS command.
roslaunch astra_launch astra.launch

# Rename ROS topic name and use a special device.
roslaunch astra_launch astra.launch camera:="cam1" device_id:="2bc5/0401@1/10"
roslaunch astra_launch astra.launch camera:="cam2" device_id:="2bc5/0401@1/11"

'''


class Astra(object):

    # Constructor:
    def __init__(self, topic_name="camera"):

        # Astra ROS topic object.
        self.topic = T(topic_name)

        # Using CvBridge to convert ROS Image to OpenCV Image.
        self.bridge = CvBridge()

        # RGB and depth image variables and it will update automatically.
        self.rgb_image = None
        self.depth_image = None

        # Subscribe Astra ROS topic for RGB image.
        self.rgb_subscriber = rospy.Subscriber(
            self.topic.rgb_image_raw(),
            Image,
            self.rgb_callback,
            queue_size=1
        )

        # Subscribe Astra ROS topic for depth image.
        self.depth_subscriber = rospy.Subscriber(
            self.topic.depth_image_raw(),
            Image,
            self.depth_callback,
            queue_size=1
        )

        # Waiting callback...
        rospy.loginfo("Waiting Astra topic callback...")
        rospy.wait_for_message(self.topic.rgb_image_raw(), Image)
        rospy.wait_for_message(self.topic.depth_image_raw(), Image)
        rospy.loginfo("Astra (%s) is OK." % self.topic.name)

    # Astra RGB image topic callback.
    def rgb_callback(self, msg):
        self.rgb_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

    # Astra depth image topic callback.
    def depth_callback(self, msg):
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, "passthrough")
        self.depth_image = np.array(self.depth_image, dtype=np.float32)

    # mouse callback for testing.
    def mouse_callback(self, event, x, y, flags, param):
        if self.depth_image is None:
            return

        if event == cv.EVENT_LBUTTONDOWN:
            val = self.depth_image[y, x]
            print("x=%d, y=%d, distance=%d" % (x, y, val))


# How to use?
if __name__ == "__main__":
    rospy.init_node("home_edu_camera", anonymous=True)
    rate = rospy.Rate(20)

    try:
        # 1. Create an Astra object.
        c = Astra()

        # 2. Prepare image window and register mouse event handler.
        cv.namedWindow("image")
        cv.namedWindow("depth")
        cv.setMouseCallback("image", c.mouse_callback)
        cv.setMouseCallback("depth", c.mouse_callback)

        # 3. Main Loop.
        while not rospy.is_shutdown():
            
            # 4. Refresh RGB-D image on the window.
            cv.imshow("image", c.rgb_image)
            cv.imshow("depth", c.depth_image / np.max(c.depth_image))
            
            # 5. press `q` or ESC to break the main loop.
            if cv.waitKey(1) in [ord('q'), 27]:
                break

            # 6. Time cycle control.
            rate.sleep()
            
    except Exception as e:
        print(e)
    finally:
        # 7. Close all windows.
        cv.destroyAllWindows()
