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

last_twist_time_time = 0.0

# define callback for twist
def twistCallback(data):
    global last_twist_time_time
    last_twist_time_time = rospy.get_time()


# define setup and run routine
def setup():
    # get initial parameters
    # user_rate  = rospy.get_param("/teleop_rand/drunken_robot/user_rate")

    # create node for listening to twist messages
    rospy.init_node("jackal_map")

    # subscribe to all
    rospy.Subscriber("cmd_vel", Twist, twistCallback)
    # rate = rospy.Rate(user_rate)
    rate = rospy.Rate(10)

    # publish to cmd_vel of the jackal
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

    # loop
    while not rospy.is_shutdown():
    	
		angular_min = -1 
		linear_min  = -1 
		angular_max =  2 
		linear_max  =  2 

		# generate random movement mapping
		map = [random.randrange(linear_min,linear_max), random.randrange(angular_min,angular_max)]

		# push Twist msgs
		motion = Twist()
		motion.linear.x  = map[0]
		motion.angular.z = map[1]

		# publish Twist
		pub.publish(motion)
		pub = rospy.Publisher("/jackal_velocity_controller/cmd_vel", Twist, queue_size=10)

		rospy.loginfo('linear_min=%d  linear_max=%d  angular_min=%d  angular_max=%d'%(linear_min, linear_max, angular_min, angular_max))
		rospy.loginfo("random movement x = {} z = {}".format(motion.linear.x, motion.angular.z))
		# rospy.logdebug("drunk mode x = {} z = {}".format(motion.linear.x, motion.angular.z))

		rate.sleep()


# standard ros boilerplate
if __name__ == "__main__":
    try:
        setup()
    except rospy.ROSInterruptException:
        pass
