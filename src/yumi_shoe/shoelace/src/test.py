#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import yumi_moveit_utils as yumi
import moveit_msgs.msg
import geometry_msgs.msg
from std_srvs.srv import Empty
import numpy as np
from math import pi
from std_msgs.msg import String
import time
import tf

LEFT = 2        #:ID of the left arm
RIGHT = 1       #:ID of the right arm
BOTH = 3        #:ID of both_arms
xoff = -0.021
yoff = 0.004
gripperoff = 0.136
zoff = 0.145 - gripperoff

class go_zed:
	def __init__(self):
		yumi.init_Moveit()
		while not rospy.is_shutdown():
			'''
			pl = [0.3, 0.3, 0.2, -pi/4, pi, pi]
			pr = [0.3, -0.3, 0.2, pi/4, pi, pi]
			pol = [0.4, 0.3, 0.2, -pi/4, pi, pi]
			por = [0.4, -0.3, 0.2, pi/4, pi, pi]
			posel = []
			poser = []
			posel.append(pl)
			posel.append(pol)
			poser.append(pr)
			poser.append(por)
			yumi.plan_path_dual(posel,poser, 500)
			'''
			yumi.print_current_joint_states(LEFT)
			yumi.print_current_joint_states(RIGHT)
			print(pl)
			#yumi.print_current_pose(LEFT)
			rospy.signal_shutdown('stop')

if __name__ == '__main__':
	rospy.init_node("go_zed", anonymous=True)
	follower=go_zed()
	rospy.spin()
