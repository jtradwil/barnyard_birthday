#!/usr/bin/env python

# publisher + subscriber that reads laser scan data from Jackal and
# generates a movement pattern for Jackal to generate a map 

# Intro to Robotics - EE5900 - Spring 2017
#          Assignment #3

#       Project #3 Group #2
#         James (Team Lead)
#            Derek
#            Akhil
#
# Revision: v1.3

# imports
import rospy
import random
import sys
import time
import roslaunch
import os

from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import LaserScan


# Global variables for random bounds
scale       =  1
angular_min = -1 
linear_min  = -1 
angular_max =  1 
linear_max  =  1

start_time  =  0

# Constants for laser averaging
front_delta = 15
side_ang    = 30
side_delta  = 15
side_thresh = 1.35


# Radian to degree function
def toAng(rad):
    ang = rad * 180 / 3.14
    return ang
    

# Averaged Sum of scan points function
def getSum(start, end, data):
    angSum = float(0.0)
    index = start
    while index < end :
        if data.ranges[index] < 15:
            angSum = angSum + data.ranges[index]
            
        index = index + 1
        
    angSum = float(angSum) / float(end-start)
    
    return angSum


# Averaged Sum of scan points function
def getMin(start, end, data):
    angSum = float(0.0)
    index = start + 1
    minScan = data.ranges[start]
    while index < end :
        if data.ranges[index] < minScan:
            prev = data.ranges[index]
            
        index = index + 1
    
    return minScan


# define callback for twist
def Callback(data):
    global linear_min, linear_max, angular_min, angular_max
    
    # Calculate front, left, and right angles in the data array
    zeroAng    = int((((abs(data.angle_min) + abs(data.angle_max)) / data.angle_increment) / 2) - 1)
    leftAng    = zeroAng + int(side_ang / toAng(data.angle_increment))
    rightAng   = zeroAng - int(side_ang / toAng(data.angle_increment))
    sideOffset = int(side_delta / toAng(data.angle_increment))
    zeroOffset = int(front_delta / toAng(data.angle_increment))
    
    # Compute averages for left, right, and front laser scan spans
    leftAve  = getMin(leftAng, leftAng + sideOffset, data)
    rightAve = getMin(rightAng - sideOffset, rightAng, data)
    frontAve = getMin(zeroAng - zeroOffset, zeroAng + zeroOffset, data)
    
    # Output for monitoring
    rospy.loginfo('\t%3.4f  -  %3.4f  -  %3.4f', leftAve, frontAve, rightAve)
        
    # Set the threshold levels for randomization
    
    # Too close in front, turn left and slowly back up  
    if frontAve < 1 :
        angular_min = 0.25 * scale
        angular_max = 0.5  * scale
        linear_min  = -0.05 * scale 
        linear_max  = 0 * scale
      
    # All Clear, randomly drive forward with varying turn  
    elif (frontAve > 2) and (leftAve > side_thresh) and (rightAve > side_thresh) :
        angular_min = -1.25 * scale
        angular_max = 1.25 * scale
        linear_min  = 0.50 * scale
        linear_max  = 1.0 * scale
        
    # Close to a wall on one side, turn to side with most time
    else :
        if leftAve > rightAve :
            angular_min = 0.75 * scale
            angular_max = 1.0 * scale
            linear_min  = 0.25 * scale
            linear_max  = 0.75 * scale
        else :
            angular_min = -1.0 * scale
            angular_max = -0.75 * scale
            linear_min  = 0.25 * scale
            linear_max  = 0.75 * scale


# define setup and run routine
def setup():
    global start_time
    start_time = time.time()

    # create node for listening to twist messages
    rospy.init_node("jackal_map")

    # subscribe to all
    rospy.Subscriber("/front/scan", LaserScan, Callback)
    # rate = rospy.Rate(user_rate)
    rate = rospy.Rate(50)

    # publish to cmd_vel of the jackal
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
    
    # Variables for messages and timing
    count = 0
    countLimit = random.randrange(25,75)
    randLin = float(0.0)
    randAng = float(0.0)
    
    # loop
    while not time.time()-start_time>600:

        # generate random movement mapping at random interval
        if count < countLimit :
            count = count + 1
        else :
            count = 0
            countLimit = random.randrange(5,25)
            randLin = random.uniform(linear_min,linear_max)
            randAng = random.uniform(angular_min,angular_max)

        # push Twist msgs
        linear_msg  = Vector3(x=randLin, y=float(0.0), z=float(0.0))
        angular_msg = Vector3(x=float(0.0), y=float(0.0), z=randAng)
        publish_msg = Twist(linear=linear_msg, angular=angular_msg)

		# publish Twist
        pub.publish(publish_msg)
        pub = rospy.Publisher("/jackal_velocity_controller/cmd_vel", Twist, queue_size=10)

        # rospy.loginfo('linear_min=%d  linear_max=%d  angular_min=%d  angular_max=%d'%(linear_min, linear_max, angular_min, angular_max))
        # rospy.loginfo("random movement x = {} z = {}".format(motion.linear.x, motion.angular.z))

        rate.sleep()

    #start save map
    package ='map_server'
    executable ='map_saver'
    # map_name  = rospy.get_param("/jackal_map/map_filename")
    # node = roslaunch.core.Node(package, executable, args="-f "+str(os.path.dirname(os.path.realpath(__file__)))+map_name)
    node = roslaunch.core.Node(package, executable, args="-f "+str(os.path.dirname(os.path.realpath(__file__)))+"/myfile")
    
	
    launch = roslaunch.scriptapi.ROSLaunch()
    launch.start()

    process = launch.launch(node)
    while process.is_alive():
        print process.is_alive()
    #process.stop()
    # rospy.loginfo("-f "+str(os.path.dirname(os.path.realpath(__file__)))+map_name)
    rospy.loginfo("-f "+str(os.path.dirname(os.path.realpath(__file__)))+"/myfile")


# standard ros boilerplate
if __name__ == "__main__":
    try:
        setup()
    except rospy.ROSInterruptException:
        pass
