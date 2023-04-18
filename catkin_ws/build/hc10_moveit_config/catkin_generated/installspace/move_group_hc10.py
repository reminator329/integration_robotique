from __future__ import print_function
from six.moves import input
from gazebo_msgs.srv import SpawnModel

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

try:
    from math import pi, tau, dist, fabs, cos
except:  # For Python 2 compatibility
    from math import pi, fabs, cos, sqrt

    tau = 2.0 * pi

    def dist(p, q):
        return sqrt(sum((p_i - q_i) ** 2.0 for p_i, q_i in zip(p, q)))


from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
import time
import math
import random

class RobotPlanner:
    def __init__(self, waypoints):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("move_group_python", anonymous=True)

        self.robot = moveit_commander.RobotCommander()

        self.scene = moveit_commander.PlanningSceneInterface()

        self.group_name = "hc10_arm"
        self.move_group = moveit_commander.MoveGroupCommander(self.group_name)
        
        self.waypoints = waypoints
        self.scale = 1.
        self.init_pose = self.move_group.get_current_pose().pose
        print("Current pose : {}".format(self.init_pose))

        ## Getting Basic Information
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^
        # We can get the name of the reference frame for this robot:
        planning_frame = self.move_group.get_planning_frame()
        print("============ Planning frame: %s" % planning_frame)

        # We can also print the name of the end-effector link for this group:
        eef_link = self.move_group.get_end_effector_link()
        print("============ End effector link: %s" % eef_link)

        # We can get a list of all the groups in the robot:
        group_names = self.robot.get_group_names()
        print("============ Available Planning Groups:", self.robot.get_group_names())

        # Sometimes for debugging it is useful to print the entire state of the
        # robot:
        print("============ Printing robot state")
        print(self.robot.get_current_state())
        print("")
    
    def create_box(self):
        print("Create box")
        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = "world"
        box_pose.pose.orientation.w = 1.0
        box_pose.pose.position.x = 0.66
        box_pose.pose.position.y = 0.0
        box_pose.pose.position.z = 0.4
        box_name = "box"
        size = .4
        self.scene.add_box(box_name, box_pose, size=(size, size, size))
    
    def goal(self):

        self.pose_goal1 = geometry_msgs.msg.Pose()
        self.pose_goal1.orientation.w = 1.0
        self.pose_goal1.position.x = 0.66
        self.pose_goal1.position.y = 0
        self.pose_goal1.position.z = 0.9

        self.pose_goal2 = geometry_msgs.msg.Pose()
        self.pose_goal2.orientation.w = 1.0
        self.pose_goal2.position.x = 0.66
        self.pose_goal2.position.y = 0.66
        self.pose_goal2.position.z = 0.9

        self.pose_goal3 = geometry_msgs.msg.Pose()
        self.pose_goal3.orientation.w = 1.0
        self.pose_goal3.position.x = -0.66
        self.pose_goal3.position.y = 0
        self.pose_goal3.position.z = 0.9

        self.pose_goal4 = geometry_msgs.msg.Pose()
        self.pose_goal4.orientation.w = 1.0
        self.pose_goal4.position.x = -0.66
        self.pose_goal4.position.y = -0.66
        self.pose_goal4.position.z = 0.9

        self.pose_goal5 = geometry_msgs.msg.Pose()
        self.pose_goal5.orientation.w = 1.0
        self.pose_goal5.position.x = math.cos(random.random()*2*math.pi)
        self.pose_goal5.position.y = math.sin(random.random()*2*math.pi)
        self.pose_goal5.position.z = 0.9 + random.random()*0.1

        self.pose_goal6 = geometry_msgs.msg.Pose()
        self.pose_goal6.orientation.w = 1.0
        self.pose_goal6.position.x = math.cos(random.random()*2*math.pi)
        self.pose_goal6.position.y = math.sin(random.random()*2*math.pi)
        self.pose_goal6.position.z = 0.9 + random.random()*0.1

        goals = [self.pose_goal1, self.pose_goal2, self.pose_goal3, self.pose_goal4 ]
        
        for goal in goals:
                
            self.move_group.set_pose_target(goal)

            self.success = self.move_group.go(wait=True)
            # Calling `stop()` ensures that there is no residual movement
            self.move_group.stop()
            # It is always good to clear your targets after planning with poses.
            # Note: there is no equivalent function for clear_joint_value_targets().
            self.move_group.clear_pose_targets()
    
        
def main():
    planner = RobotPlanner([])
    planner.goal()

if __name__ == '__main__':
    main()