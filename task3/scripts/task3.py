#! /usr/bin/env python

# Importing important libraries
import rospy
import sys
import copy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import actionlib
import math
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from moveit_msgs.msg import CollisionObject, AttachedCollisionObject


# Class definition of the Ur5 arm which holds all the required variables and methods
class Ur5Moveit:

    # Constructor which defines the various planning groups and scenes to get the robot moving
    def __init__(self):

        rospy.init_node('node_task3', anonymous=True)

        self._planning_group = "robotiq_planning_group"
        self._commander = moveit_commander.roscpp_initialize(sys.argv)
        self._robot = moveit_commander.RobotCommander()
        self._scene = moveit_commander.PlanningSceneInterface()
        self._group = moveit_commander.MoveGroupCommander(self._planning_group)


        self._planning_frame = self._group.get_planning_frame()
        self._eef_link = self._group.get_end_effector_link()
        self._group_names = self._robot.get_group_names()
        self._box_name = ''

        self.gripper_group = "gripper_planning_group"
        self._group_ = moveit_commander.MoveGroupCommander(self.gripper_group)

        # self._display_trajectory_publisher_ = rospy.Publisher(
        #     '/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)

        self._exectute_trajectory_client_ = actionlib.SimpleActionClient(
            'execute_trajectory', moveit_msgs.msg.ExecuteTrajectoryAction)
        self._exectute_trajectory_client_.wait_for_server()

        self._planning_frame_ = self._group_.get_planning_frame()
        self._eef_link_ = self._group_.get_end_effector_link()


        self._display_trajectory_publisher = rospy.Publisher(
            '/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)

        self._exectute_trajectory_client = actionlib.SimpleActionClient(
            'execute_trajectory', moveit_msgs.msg.ExecuteTrajectoryAction)
        self._exectute_trajectory_client.wait_for_server()


        # Current State of the Robot is needed to add box to planning scene
        self._curr_state = self._robot.get_current_state()


        rospy.loginfo(
            '\033[94m' + "Planning Group: {}".format(self._planning_frame) + '\033[0m')
        rospy.loginfo(
            '\033[94m' + "End Effector Link: {}".format(self._eef_link) + '\033[0m')
        rospy.loginfo(
            '\033[94m' + "Group Names: {}".format(self._group_names) + '\033[0m')

        rospy.loginfo('\033[94m' + " >>> Ur5Moveit init done." + '\033[0m')

    # Function to make the Ur5 arm end effector move to a given Pose value
    def go_to_pose(self, arg_pose):

        pose_values = self._group.get_current_pose().pose
        rospy.loginfo('\033[94m' + ">>> Current Pose:" + '\033[0m')
        rospy.loginfo(pose_values)



        self._group.set_pose_target(arg_pose)
        
        flag_plan = self._group.go(wait=True)  # wait=False for Async Move

        #touch_links = self._robot.get_link_names(group=self.gripper_group)
        #self._scene.attach_box(self._eef_link, self.box_name, touch_links=touch_links)

        pose_values = self._group.get_current_pose().pose
        rospy.loginfo('\033[94m' + ">>> Final Pose:" + '\033[0m')
        rospy.loginfo(pose_values)

        list_joint_values = self._group.get_current_joint_values()
        rospy.loginfo('\033[94m' + ">>> Final Joint Values:" + '\033[0m')
        rospy.loginfo(list_joint_values)

        if (flag_plan == True):
            rospy.loginfo(
                '\033[94m' + ">>> go_to_pose() Success" + '\033[0m')
        else:
            rospy.logerr(
                '\033[94m' + ">>> go_to_pose() Failed. Solution for Pose not Found." + '\033[0m')


        return flag_plan

    # Function to make the gripper of the arm to go to a pre-defined pose ie. close or open
    def gripper_go_to_predefined_pose(self, arg_pose_name):
        
        pose_values = self._group_.get_current_pose().pose
        rospy.loginfo(pose_values)

        touch_links = self._robot.get_link_names(group=self.gripper_group)
        self._scene.attach_box(self._eef_link_, self._box_name, touch_links=touch_links)

        rospy.loginfo('\033[94m' + "Going to Pose: {}".format(arg_pose_name) + '\033[0m')

        self._group_.set_named_target(arg_pose_name)
        plan = self._group_.plan()
        goal = moveit_msgs.msg.ExecuteTrajectoryGoal()
        goal.trajectory = plan[1]
        self._exectute_trajectory_client_.send_goal(goal)
        self._exectute_trajectory_client_.wait_for_result()
        rospy.loginfo('\033[94m' + "Now at Pose: {}".format(arg_pose_name) + '\033[0m')

        if arg_pose_name == "closedGripper":
            rospy.loginfo("Gripping the object")
        elif arg_pose_name == "openGripper":
            rospy.loginfo("Dropping the object")

        

    # Destructor which shuts down the commander
    def __del__(self):
        moveit_commander.roscpp_shutdown()
        rospy.loginfo(
            '\033[94m' + "Object of class Ur5Moveit Deleted." + '\033[0m')

# Main function of the program
def main():

    ur5 = Ur5Moveit()
    
    # Defining the Poses of the various objects(packages) and drop boxes

    ur5_pose_1 = geometry_msgs.msg.Pose()
    ur5_pose_1.position.x = 0.57
    ur5_pose_1.position.y = -0.03
    ur5_pose_1.position.z = 0.861
    angles = quaternion_from_euler(-1.57,0,-0.84)

    ur5_pose_1.orientation.x = angles[0] 
    ur5_pose_1.orientation.y = angles[1] 
    ur5_pose_1.orientation.z = angles[2] 
    ur5_pose_1.orientation.w = angles[3] 

    angles = quaternion_from_euler(-1.57,0,0.73)

    ur5_pose_2 = geometry_msgs.msg.Pose()
    ur5_pose_2.position.x = 0.486
    ur5_pose_2.position.y = 0.22
    ur5_pose_2.position.z = 0.859
    ur5_pose_2.orientation.x = angles[0]
    ur5_pose_2.orientation.y = angles[1]
    ur5_pose_2.orientation.z = angles[2]
    ur5_pose_2.orientation.w = angles[3]

    angles = quaternion_from_euler(-1.57,0,0)

    ur5_pose_3 = geometry_msgs.msg.Pose()
    ur5_pose_3.position.x = 0.57
    ur5_pose_3.position.y = -0.240
    ur5_pose_3.position.z = 0.91
    ur5_pose_3.orientation.x = angles[0]
    ur5_pose_3.orientation.y = angles[1]
    ur5_pose_3.orientation.z = angles[2]
    ur5_pose_3.orientation.w = angles[3]

    angles = quaternion_from_euler(-1.57,0,0)

    box_1 = geometry_msgs.msg.Pose()
    box_1.position.x = 0.0
    box_1.position.y = 0.7100
    box_1.position.z = 1.2
    box_1.orientation.x = angles[0]
    box_1.orientation.y = angles[1]
    box_1.orientation.z = angles[2]
    box_1.orientation.w = angles[3]

    box_2 = geometry_msgs.msg.Pose()
    box_2.position.x = 0.0
    box_2.position.y = -0.7100
    box_2.position.z = 1.2
    box_2.orientation.x = angles[0]
    box_2.orientation.y = angles[1]
    box_2.orientation.z = angles[2]
    box_2.orientation.w = angles[3]

    # Loop to send the pose values to the various functions of the ur5 arm
    while not rospy.is_shutdown():
        ur5.go_to_pose(ur5_pose_1)
        rospy.sleep(1)
        ur5.gripper_go_to_predefined_pose("closedGripper_Object1")
        rospy.sleep(1)
        ur5.go_to_pose(box_1)
        rospy.sleep(1)
        ur5.gripper_go_to_predefined_pose("openGripper")
        rospy.sleep(1)
        
        ur5.go_to_pose(ur5_pose_2)
        rospy.sleep(1)
        ur5.gripper_go_to_predefined_pose("closedGripper_Object2")
        rospy.sleep(1)
        ur5.go_to_pose(box_2)
        rospy.sleep(1)
        ur5.gripper_go_to_predefined_pose("openGripper")
        rospy.sleep(1)

        ur5.go_to_pose(ur5_pose_3)
        rospy.sleep(1)
        ur5.gripper_go_to_predefined_pose("closedGripper_Object1")
        rospy.sleep(1)
        ur5.go_to_pose(box_2)
        rospy.sleep(1)
        ur5.gripper_go_to_predefined_pose('openGripper')
        rospy.sleep(1)
        break

    del ur5

# Main loop of the program
if __name__ == '__main__':
    main()
