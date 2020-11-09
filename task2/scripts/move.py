#!/usr/bin/env python

# importing important libraries
import rospy
import math

import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose, Point, Quaternion
from tf.transformations import quaternion_from_euler

# defining the waypoints
waypoints = [(-9.1,-1.2),(10.7,10.5),(12.6,-1.9),(18.2,-1.4),(-2,4)]
angles = [0.5337081916392616, -1.418753058931488, 0.0890495826344978, 2.8803741189752583]

def move():
    global waypoints
    global angles

    rospy.init_node('move_base_sequence')
    
    client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
    client.wait_for_server()
    i = 0

    for point in waypoints:
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = point[0]
        goal.target_pose.pose.position.y = point[1]
        goal.target_pose.pose.orientation.w =  angles[i]

        i+=1
        client.send_goal(goal)
        # wait = client.wait_for_result()
        st = "Reaching waypoint : " + str(i+1)
        rospy.loginfo(st)
    
    return True


if __name__ == '__main__':
    try:
        result = move()
        if result:
            rospy.loginfo("Goal execution done!")
    except rospy.ROSInterruptException:
        pass