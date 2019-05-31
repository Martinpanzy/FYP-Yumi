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
from std_msgs.msg import Int32
from rospy.numpy_msg import numpy_msg

import tf

LEFT = 2        #:ID of the left arm
RIGHT = 1       #:ID of the right arm
BOTH = 3        #:ID of both_arms
xoff = -0.0205
yoff = -0.023
gripperoff = 0.136
zoff = 0.17 - gripperoff


class adjust_shoe:
	def __init__(self):
		self._tfsub = tf.TransformListener()
		yumi.init_Moveit()
		while not rospy.is_shutdown():
			try:
				(trans_adr,_) = self._tfsub.lookupTransform('/yumi_body', '/adr', rospy.Time(0))
				(trans_adrr,_) = self._tfsub.lookupTransform('/yumi_body', '/adrr', rospy.Time(0))
				(trans_adl,_) = self._tfsub.lookupTransform('/yumi_body', '/adl', rospy.Time(0))
				(trans_adll,_) = self._tfsub.lookupTransform('/yumi_body', '/adll', rospy.Time(0))
			except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
				continue
			yoffset = yoff - (gripperoff)*np.cos(pi/4)
			x_r = trans_adr[0] + xoff
			y_r = trans_adr[1] + yoffset
			y_rr = trans_adrr[1] + yoffset

			x_l = trans_adl[0] + xoff
			y_l = trans_adl[1] - yoffset
			y_ll = trans_adll[1] - yoffset
		
			#print (x_r, y_r, y_rr, x_l, y_l, y_ll)

			yumi.reset_arm_home(BOTH)
			yumi.plan_and_move_dual(yumi.create_pose_euler(x_l, y_l, 0.2, -pi/4, pi, pi), yumi.create_pose_euler(x_r, y_r, 0.2, pi/4, pi, pi))
			yumi.plan_and_move_dual(yumi.create_pose_euler(x_l, y_l, 0.1, -pi/4, pi, pi), yumi.create_pose_euler(x_r, y_r, 0.1, pi/4, pi, pi))
			yumi.plan_and_move_dual(yumi.create_pose_euler(x_l, y_ll, 0.1, -pi/4, pi, pi), yumi.create_pose_euler(x_r, y_rr, 0.1, pi/4, pi, pi))
			yumi.plan_and_move_dual(yumi.create_pose_euler(x_l, y_l, 0.1, -pi/4, pi, pi), yumi.create_pose_euler(x_r, y_r, 0.1, pi/4, pi, pi))
			yumi.plan_and_move_dual(yumi.create_pose_euler(x_l, y_l, 0.2, -pi/4, pi, pi), yumi.create_pose_euler(x_r, y_r, 0.2, pi/4, pi, pi))
			yumi.reset_arm_home(BOTH)
			yumi.reset_arm_cal(BOTH)



if __name__ == '__main__':
	rospy.init_node("adjust_shoe", anonymous=True)
	follower=adjust_shoe()
	rospy.spin()
