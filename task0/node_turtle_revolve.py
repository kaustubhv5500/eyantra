#!/usr/bin/env python

import rospy
import math
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

arc = 0

def rotate():
    # Starts a new node
    rospy.init_node('turtle_revolve', anonymous=True)
    velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    vel_msg = Twist()
    
    
    #Radius of the circle and the linear speed
    radius = 1
    speed = 1.5
    
    distance = math.pi*2*radius + 0.43 #Accounting for the size of the turtle

    def pose_callback(msg):
        data = msg.theta
        string = "Moving in a circle : " + str(arc-0.43)
    	rospy.loginfo(string)
    
    #Setting the angular and linear velocities of the turtle 
    vel_msg.linear.x = speed
    vel_msg.linear.y = 0
    vel_msg.linear.z = 0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0
    vel_msg.angular.z = speed/radius
    pose_sub = rospy.Subscriber('/turtle1/pose', Pose, pose_callback)

    #Setting the current time for distance calculus
    t0 = float(rospy.Time.now().to_sec())
    global arc
    del_theta = 0

    #Loop to make the turtle revolve
    while(arc< distance):
        velocity_publisher.publish(vel_msg)  
        t1=float(rospy.Time.now().to_sec())  
        del_theta = float(speed/radius)*(t1-t0)
        arc = radius*del_theta
        rospy.Rate(15)
        #rospy.loginfo(arc)
            
    #After the circle is complete, the turtle is stopped
    vel_msg.linear.x = 0
    vel_msg.angular.z = 0
    velocity_publisher.publish(vel_msg)
    rospy.loginfo("Reached Goal")
        
#Main which calls the move method
if __name__ == '__main__':
    try:
        rotate()
    except rospy.ROSInterruptException: 
    	pass