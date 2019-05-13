#!/usr/bin/env python
import actionlib
import rospy
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion, PointStamped, PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler
from actionlib_msgs.msg import GoalStatusArray
import time

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
        self.last_clicked_point = PointStamped()
        self.last_goal_pose = PoseStamped()
        self.status = GoalStatusArray()

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

        rospy.Subscriber(
            "/move_base_simple/goal",
            PoseStamped,
            self.goal_callback
        )
        
        rospy.Subscriber(
            "/move_base/status",
            GoalStatusArray,
            self.status_callback,
            queue_size=1
        )
        
        # Cancel preview goal.
        self.move_base.cancel_goal()
        rospy.wait_for_message("/amcl_pose", GoalStatusArray)
        rospy.wait_for_message("/move_base/status", GoalStatusArray)
    
    # Set current pose.
    def set_initial_pose_in_rviz(self):
        self.initial_pose = PoseWithCovarianceStamped()
        rospy.loginfo("Waiting for the robot's initial pose...")
        rospy.wait_for_message("initialpose", PoseWithCovarianceStamped)
        rospy.loginfo("Set initial pose finished.")
        return self.initial_pose.header.stamp != ""
    
    def get_clicked_point_in_rviz(self):
        rospy.loginfo("Waiting for the robot's clicked point...")
        rospy.wait_for_message("/clicked_point", PointStamped)
        rospy.loginfo("Get clicked point (%.2f, %.2f, %.2f)" % (
            self.last_clicked_point.point.x,
            self.last_clicked_point.point.y,
            self.last_clicked_point.point.z
        ))
        return self.last_clicked_point
    
    def set_goal_in_rviz(self):
        self.last_goal_pose = PoseStamped()
        rospy.loginfo("Waiting for the robot's goal...")
        rospy.wait_for_message('move_base_simple/goal', PoseStamped)
        rospy.loginfo("Set goal finished.")
        time.sleep(1)
        return self.last_goal_pose.header.stamp != ""
    
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
        time.sleep(1)
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

    def goal_callback(self, pose):
        self.last_goal_pose = pose

    def get_current_pose(self):
        point = self.current_pose.pose.pose.position
        theta = self.current_pose.pose.pose.orientation.z
        return point.x, point.y, theta
    
    def get_goal_pose(self):
        point = self.last_goal_pose.pose.position
        theta = self.last_goal_pose.pose.orientation.z
        return point.x, point.y, theta
    
    def status_callback(self, data):
        self.status = data


# How to use?
if __name__ == "__main__":
    rospy.init_node("home_edu_robot_chassis")
    rate = rospy.Rate(20)
    
    # 1. Create a RobotChassis object.
    chassis = RobotChassis()
    
    # 2. Set current pose at the first time.
    chassis.set_initial_pose_in_rviz()
    P = chassis.get_current_pose()
    rospy.loginfo("From %.2f, %.2f, %.2f" % (P[0], P[1], P[2]))
    
    chassis.set_goal_in_rviz()
    G = chassis.get_goal_pose()
    rospy.loginfo("To %.2f, %.2f, %.2f" % (G[0], G[1], G[2]))
    
    while not rospy.is_shutdown():
        code = chassis.status.status_list[-1].status
        text = chassis.status.status_list[-1].text

        if code == 1:
            pass
        elif code == 3:
            rospy.loginfo("3. Move to %.2f, %.2f, %.2f" % (P[0], P[1], P[2]))
            chassis.move_to(P[0], P[1], P[2])
            P = chassis.get_current_pose()
        elif code == 4:
            chassis.set_goal_in_rviz()
            G = chassis.get_goal_pose()
            rospy.loginfo("To %.2f, %.2f, %.2f" % (G[0], G[1], G[2]))
        else:
            rospy.loginfo("%d, %s" % (code, text))
        
        # if code == 7:
        #     rospy.loginfo("7. Move to %.2f, %.2f, %.2f" % (G[0], G[1], G[2]))
        #     chassis.move_base.cancel_goal()
        #     chassis.move_to(G[0], G[1], G[2])
        # elif code == 4:
        #     chassis.set_initial_pose_in_rviz()
        #     chassis.set_goal_in_rviz()
        #     G = chassis.get_goal_pose()
        #     rospy.loginfo("4. To %.2f, %.2f, %.2f" % (G[0], G[1], G[2]))
        # elif code == 3:
        #     rospy.loginfo("3. Move to %.2f, %.2f, %.2f" % (P[0], P[1], P[2]))
        #     chassis.move_to(P[0], P[1], P[2])
        #     P = chassis.get_current_pose()
        #
        rate.sleep()
    
    # 3. Get a target point from rviz tool.
    # p = chassis.get_clicked_point()
    # rospy.loginfo("%.2f, %.2f, %.2f" % (p.point.x, p.point.y, p.point.z))
    
    # 4. Move to the target point.
    # success = chassis.move_to(p.point.x, p.point.y, p.point.z)
    # rospy.loginfo(success)
    rospy.loginfo("END")
