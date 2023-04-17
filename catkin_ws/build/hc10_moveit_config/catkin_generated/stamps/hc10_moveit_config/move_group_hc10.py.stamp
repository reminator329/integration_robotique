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

class RobotPlanner:
    def __init__(self, waypoints):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("move_group_python", anonymous=True)

        self.robot = moveit_commander.RobotCommander()

        self.scene = moveit_commander.PlanningSceneInterface()
        
        self.group_name = "hc10_arm"
        self.move_group = moveit_commander.MoveGroupCommander(self.group_name)

        self.display_trajectory_publisher = rospy.Publisher(
        "/move_group/display_planned_path",
        moveit_msgs.msg.DisplayTrajectory,
        queue_size=20)

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
        ## Create a `DisplayTrajectory`_ ROS publisher which is used to display
        ## trajectories in Rviz:
        self.display_trajectory_publisher = rospy.Publisher(
            "/move_group/display_planned_path",
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=20,
        )
        time.sleep(1)
        self.create_box()
    
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
        self.pose_goal = geometry_msgs.msg.Pose()
        self.pose_goal.orientation.w = 1.0
        self.pose_goal.position.x = 0.9
        self.pose_goal.position.y = 0.1
        self.pose_goal.position.z = 0.9

        print("Before target")
        self.move_group.set_pose_target(self.pose_goal)
        print("After target")

        self.success = self.move_group.go(wait=True)
        # Calling `stop()` ensures that there is no residual movement
        self.move_group.stop()
        # It is always good to clear your targets after planning with poses.
        # Note: there is no equivalent function for clear_joint_value_targets().
        self.move_group.clear_pose_targets()

        print("Before target2")
        self.move_group.set_pose_target(self.init_pose)
        self.success = self.move_group.go(wait=True)
        print("After target2")

        # Calling `stop()` ensures that there is no residual movement
        self.move_group.stop()
        # It is always good to clear your targets after planning with poses.
        # Note: there is no equivalent function for clear_joint_value_targets().
        self.move_group.clear_pose_targets()

    def cartesian_path(self):
        wpose = self.move_group.get_current_pose().pose
        wpose.position.z -= self.scale * 0.1  # First move up (z)
        wpose.position.y += self.scale * 0.2  # and sideways (y)
        self.waypoints.append(copy.deepcopy(wpose))

        wpose.position.x += self.scale * 0.1  # Second move forward/backwards in (x)
        self.waypoints.append(copy.deepcopy(wpose))

        wpose.position.y -= self.scale * 0.1  # Third move sideways (y)
        self.waypoints.append(copy.deepcopy(wpose))

        # We want the Cartesian path to be interpolated at a resolution of 1 cm
        # which is why we will specify 0.01 as the eef_step in Cartesian
        # translation.  We will disable the jump threshold by setting it to 0.0,
        # ignoring the check for infeasible jumps in joint space, which is sufficient
        # for this tutorial.
        (self.plan, self.fraction) = self.move_group.compute_cartesian_path(
            self.waypoints, 0.01, 0.0  # waypoints to follow  # eef_step
        )  # jump_thresholdplanner

    def display_path(self):
        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = self.robot.get_current_state()
        display_trajectory.trajectory.append(self.plan)
        # Publish
        self.display_trajectory_publisher.publish(display_trajectory)

    def execute(self):
        self.move_group.execute(self.plan, wait=True)
    
def main():
    planner = RobotPlanner([])
    """ with open('/home/etudiant/Bureau/catkin_ws/src/motoman/motoman_hc10_support/urdf/box.urdf') as f:
        box = f.read()
    # Appel du service de spawn de modèle Gazebo
    spawn_model = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)
    spawn_model('box', box, [1.,0.66, 0 ,0.4],'world') """
    
    # Attente de 5 secondes pour que Gazebo ait le temps de charger l'objet
    """ rospy.sleep(5)  """   
    print("Before GOAL")
    planner.goal()
    print("After GOAL")
    # planner.cartesian_path()
    # planner.execute()
    # planner.display_path()

if __name__ == '__main__':
    main()