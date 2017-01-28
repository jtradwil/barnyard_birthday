#!/usr/bin/env python

# publisher + subscriber that reads cmd_vel from Jackal and
# generates a random movement for Jckal to generate map

# Intro to Robotics - EE5900 - Spring 2017
#          Assignment #3

#       Project #3 Group #2
#         James (Team Lead)
#           Derek
#           Akhil
#
# Revision: v1.1

# imports
import rospy
import random
import sys
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

angular_min = -1 
linear_min  = -1 
angular_max =  2 
linear_max  =  2 


# define callback for twist
def Callback(data):
	global linear_min, linear_max, angular_min, angular_max

	# debug info
	rospy.loginfo(rospy.get_caller_id())
	rospy.loginfo('data.scan_time = {}'.format(data.scan_time))
	rospy.loginfo('data.angle_min = {}'.format(data.angle_min))
	rospy.loginfo('data.angle_max = {}'.format(data.angle_max))
	rospy.loginfo('data.angle_increment = {}'.format(data.angle_increment))
	rospy.loginfo('data.front_range = {}'.format(data.ranges[540]))

	# set thresholds
	if data.ranges[540] > 1.2 :
		angular_min = -8 
		linear_min  = -8 
		angular_max =  9 
		linear_max  =  9

	elif data.ranges[540] > 0.75 :
		angular_min = -5 
		linear_min  = -5 
		angular_max =  6 
		linear_max  =  6 

	else :
		angular_min = -2 
		linear_min  = -2 
		angular_max =  3 
		linear_max  =  3 


# define setup and run routine
def setup():
    # get initial parameters
    # user_rate  = rospy.get_param("/teleop_rand/drunken_robot/user_rate")

    # create node for listening to twist messages
    rospy.init_node("jackal_map")

    # subscribe to all
    rospy.Subscriber("/front/scan", LaserScan, Callback)
    # rate = rospy.Rate(user_rate)
    rate = rospy.Rate(1)

    # publish to cmd_vel of the jackal
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

    # loop
    while not rospy.is_shutdown():

		# generate random movement mapping
		map = [random.randrange(linear_min,linear_max), random.randrange(angular_min,angular_max)]

		# push Twist msgs
		motion = Twist()
		motion.linear.x  = map[0]
		motion.angular.z = map[1]

		# publish Twist
		# pub.publish(motion)
		# pub = rospy.Publisher("/jackal_velocity_controller/cmd_vel", Twist, queue_size=10)

		# rospy.loginfo('linear_min=%d  linear_max=%d  angular_min=%d  angular_max=%d'%(linear_min, linear_max, angular_min, angular_max))
		# rospy.loginfo("random movement x = {} z = {}".format(motion.linear.x, motion.angular.z))
		# rospy.logdebug("drunk mode x = {} z = {}".format(motion.linear.x, motion.angular.z))

		rate.sleep()


# standard ros boilerplate
if __name__ == "__main__":
    try:
        setup()
    except rospy.ROSInterruptException:
        pass
