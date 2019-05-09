#!/usr/bin/env python
import actionlib
import rospy
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler

'''
for building a new map.
roslaunch turtlebot_bringup minimal.launch
roslaunch turtlebot_navigation gmapping_demo.launch
roslaunch turtlebot_rviz_launchers view_navigation.launch
rosrun map_server map_saver -f /tmp/my_map

loading an exist map.
export TURTLEBOT_MAP_FILE=/tmp/my_map.yaml
roslaunch turtlebot_bringup minimal.launch
roslaunch turtlebot_navigation amcl_demo.launch
roslaunch turtlebot_rviz_launchers view_navigation.launch --screen
'''


class RobotChassis:
    
    # Constructor:
    def __init__(self, frame_id="map"):
        
        # Action client.
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        
        # Waiting action server...
        rospy.loginfo("Waiting for move_base action server...")
        self.move_base.wait_for_server(rospy.Duration(120))
        rospy.loginfo("Connected to move base server")
        
        # Instance variables.
        self.frame_id = frame_id
        self.initial_pose = PoseWithCovarianceStamped()
        self.goal = None

        # Register shutdown event handler.
        rospy.on_shutdown(self.shutdown)
        
        # Subscribe initialpose topic.
        rospy.Subscriber(
            "initialpose",
            PoseWithCovarianceStamped,
            self.update_initial_pose
        )
    
    # Set current pose.
    def set_pose(self):
        self.initial_pose = PoseWithCovarianceStamped()
        rospy.loginfo("Waiting for the robot's initial pose...")
        rospy.wait_for_message("initialpose", PoseWithCovarianceStamped)
        rospy.loginfo("Set initial pose finished.")
        return self.initial_pose.header.stamp != ""
    
    @staticmethod
    def point_to_pose(x, y, theta):
        q = quaternion_from_euler(0.0, 0.0, theta)
        location = Pose(Point(x, y, 0.0), Quaternion(q[0], q[1], q[2], q[3]))
        return location
    
    # Move to Point.
    def move_to(self, x, y, theta):
        location = self.point_to_pose(x, y, theta)
        self.goal = MoveBaseGoal()
        self.goal.target_pose.header.frame_id = self.frame_id
        self.goal.target_pose.header.stamp = rospy.Time.now()
        self.goal.target_pose.pose = location
        self.move_base.send_goal(self.goal)
        success = self.move_base.wait_for_result(rospy.Duration(300))
        if success == 1:
            rospy.loginfo("Reached point.")
        else:
            rospy.loginfo("Failed to reach point.")
        return success
    
    # Shutdown event handler.
    def shutdown(self):
        rospy.loginfo("Robot Chassis is shutting down...")
        self.move_base.cancel_goal()
    
    # ROS initialpose topic callback.
    def update_initial_pose(self, initial_pose):
        self.initial_pose = initial_pose


# How to use?
if __name__ == "__main__":
    rospy.init_node("home_edu_robot_chassis")
    
    # 1. Create a RobotChassis object.
    chassis = RobotChassis()
    
    # 2. Set current pose at the first time.
    chassis.set_pose()
    
    # 3. Move to Point.
    chassis.move_to(0, 0, 0)
