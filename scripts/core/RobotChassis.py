#!/usr/bin/env python
import actionlib
import rospy
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion, PointStamped
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
        self.current_pose = PoseWithCovarianceStamped()
        self.goal = MoveBaseGoal()

        # Register shutdown event handler.
        rospy.on_shutdown(self.shutdown)
        
        # Subscribe initialpose topic.
        rospy.Subscriber(
            "/initialpose",
            PoseWithCovarianceStamped,
            self.initialpose_callback
        )
        
        # Subscribe amcl_pose topic.
        rospy.Subscriber(
            "/amcl_pose",
            PoseWithCovarianceStamped,
            self.amcl_pose_callback
        )
        
        # Subscribe clicked_point topic.
        rospy.Subscriber(
            "/clicked_point",
            PointStamped,
            self.clicked_point_callback
        )
        
        # Cancel preview goal.
        self.move_base.cancel_goal()
    
    # Set current pose.
    def set_initial_pose(self):
        self.initial_pose = PoseWithCovarianceStamped()
        rospy.loginfo("Waiting for the robot's initial pose...")
        rospy.wait_for_message("initialpose", PoseWithCovarianceStamped)
        rospy.loginfo("Set initial pose finished.")
        return self.initial_pose.header.stamp != ""
    
    def get_clicked_point(self):
        rospy.loginfo("Waiting for the robot's clicked point...")
        rospy.wait_for_message("/clicked_point", PointStamped)
        rospy.loginfo("Get clicked point (%.2f, %.2f, %.2f)" % (
            self.last_clicked_point.point.x,
            self.last_clicked_point.point.y,
            self.last_clicked_point.point.z
        ))
        return self.last_clicked_point
    
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
    def initialpose_callback(self, initial_pose):
        self.initial_pose = initial_pose
    
    def amcl_pose_callback(self, pose):
        self.current_pose = pose
    
    def clicked_point_callback(self, point):
        self.last_clicked_point = point


# How to use?
if __name__ == "__main__":
    rospy.init_node("home_edu_robot_chassis")
    
    # 1. Create a RobotChassis object.
    chassis = RobotChassis()
    
    # 2. Set current pose at the first time.
    chassis.set_initial_pose()
    
    # 3. Get a target point from rviz tool.
    p = chassis.get_clicked_point()
    rospy.loginfo("%.2f, %.2f, %.2f" % (p.point.x, p.point.y, p.point.z))
    
    # 4. Move to the target point.
    success = chassis.move_to(p.point.x, p.point.y, p.point.z)
    rospy.loginfo(success)
    rospy.loginfo("END")
