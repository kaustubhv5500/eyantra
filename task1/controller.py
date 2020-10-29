#!/usr/bin/env python

#Importing important libraries
import rospy
from geometry_msgs.msg import Twist, Point, Pose
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import math
import numpy as np
from tf.transformations import euler_from_quaternion

# Declaring global variables
goal = (12.5,0)   # Defining the goal of the bot
regions = dict()
pose = [0,0,0]


# Function to calculate y value of input argument
def val(x):
    return 2*math.sin(x)*math.sin(x/2)

#Function to determine the waypoints
def Waypoints(t): #t : Number of waypoints 
    x = [i*math.pi*2/t for i in range(t)]
    y = list(map(val,x))

    ret = [(x[i],y[i]) for i in range(len(x))]
    ret.append((2*math.pi,0))

    return ret

# Callback function returning the position and orientation of the bot
def odom_callback(data):
    global pose
    x  = data.pose.pose.orientation.x
    y  = data.pose.pose.orientation.y
    z = data.pose.pose.orientation.z
    w = data.pose.pose.orientation.w
    pose = [data.pose.pose.position.x, data.pose.pose.position.y, euler_from_quaternion([x,y,z,w])[2]]
    
# Callback function returning the laser scanner data
def laser_callback(msg):
    range_max = 1.5
    global regions

    regions = {
        'bright': min(min(msg.ranges[600:720]), range_max),
        'fright': min(min(msg.ranges[460:580]), range_max),
        'front':  min(min(msg.ranges[300:420]), range_max),
        'fleft':  min(min(msg.ranges[140:260]), range_max),
        'bleft':  min(min(msg.ranges[0:120]), range_max), 
        }
    #rospy.loginfo(regions)

# Function to make the bot turn left when obstacle is encountered
def turn_left():
    msg = Twist()
    msg.linear.x = 0
    msg.angular.z = 0.5 
    return msg

# Function to make the bot follow the conave wall
def follow_wall():
    global regions
    
    msg = Twist()
    msg.linear.x = 0.3
    msg.angular.z = 0.2 
    return msg

# Main control loop function to control the bot
def control_loop():

    # Initializing nodes, publishers and subscribers
    rospy.init_node('ebot_controller')
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rospy.Subscriber('/ebot/laser/scan', LaserScan, laser_callback)
    rospy.Subscriber('/odom', Odometry, odom_callback)

    # Setting the rospy refresh rate at 10 Hz
    rate = rospy.Rate(10) 

    global pose
    global goal
    global regions

    # Twist object which publishes to /cmd_vel
    velocity_msg = Twist()
    velocity_msg.linear.x = 0
    velocity_msg.angular.z = 0

    # Declaring constants to help move the bot in the given path
    n = 9 
    i = 0
    P = 2.39 
    d = 1

    wp = Waypoints(n)
    pub.publish(velocity_msg)


    # Loop to make the bot reach the final goal ie. (12.5,0) in our case
    while pose[0] < goal[0]:
        
        # Loop to make the bot move in the given sinusoidal path
        while pose[0] < 2*math.pi:
            x2 = wp[i][0]
            y2 = wp[i][1]
            
            x1 = pose[0]
            y1 = pose[1]
            w = pose[2]

            if x1 > x2 : 
                i+=1

            # Correcting the course of the bot using PID controller
            theta_goal = math.atan2((y2-y1),(x2-x1))
            e_theta = theta_goal - w

            velocity_msg.linear.x = 0.35 
            velocity_msg.angular.z = P * e_theta
            pub.publish(velocity_msg)

            if i > len(wp):
                break

        x1_ = pose[0]
        y1_ = pose[1]
        w_ = pose[2]

        x2_ = goal[0]
        y2_ = goal[1]

        theta_goal_ = math.atan2((y2_-y1_),(x2_-x1_))
        e_theta_ = theta_goal_ - w_   
        

        # Conditions to navigate the obstacle based on the data from laser callback

        if regions['front'] > d and regions['fleft'] > d and regions['fright'] > d:
            # Go to goal algorithm
            P = 2.3
            velocity_msg.linear.x = 0.33
            velocity_msg.angular.z = P * e_theta_
            
        elif regions['front'] < d and regions['fleft'] > d and regions['fright'] > d:
            velocity_msg = turn_left()

        elif regions['front'] > d and regions['fleft'] > d and regions['fright'] < d:
            velocity_msg = turn_left()

        elif regions['front'] > d and regions['fleft'] < d and regions['fright'] > d:
            velocity_msg = follow_wall()
            
        elif regions['front'] < d and regions['fleft'] > d and regions['fright'] < d:
            velocity_msg = turn_left()

        elif regions['front'] < d and regions['fleft'] < d and regions['fright'] > d:
            velocity_msg = turn_left()

        elif regions['front'] < d and regions['fleft'] < d and regions['fright'] < d:
            velocity_msg = turn_left()
            
        elif regions['front'] > d and regions['fleft'] < d and regions['fright'] < d:
            velocity_msg = turn_left()

        pub.publish(velocity_msg)
        rospy.loginfo("Controller message pushed at {}".format(rospy.get_time()))
        rate.sleep()
        

    # Code to stop the bot after it reaches the goal
    velocity_msg.linear.x = 0
    velocity_msg.angular.z = 0
    pub.publish(velocity_msg)
    rospy.loginfo_once("Reached Goal!")
        
        
    

#Main loop of the function
if __name__ == '__main__':
    try:
        control_loop()
    except rospy.ROSInterruptException:
        pass

