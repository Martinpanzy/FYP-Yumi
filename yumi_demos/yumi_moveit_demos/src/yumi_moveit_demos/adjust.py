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
import signal
from concurrent.futures import TimeoutError

import tf

LEFT = 2        #:ID of the left arm
RIGHT = 1       #:ID of the right arm
BOTH = 3        #:ID of both_arms
#xoff = -0.0205
#yoff = -0.023
xoff = -0.03
yoff = -0.032
gripperoff = 0.136
zoff = 0.18 - gripperoff


class adjust_shoe:
	def handler(self, signum, frame):
		raise TimeoutError
	def done():
		print('------------Finished-----------')

	def __init__(self):
		self._tfsub = tf.TransformListener()
		self.need_adj = False
		yumi.init_Moveit()
		signal.signal(signal.SIGALRM, self.handler)
		signal.alarm(3)
		while not rospy.is_shutdown():
			try:
				(trans_adr,_) = self._tfsub.lookupTransform('/yumi_body', '/adr', rospy.Time(0))
				(trans_adrr,_) = self._tfsub.lookupTransform('/yumi_body', '/adrr', rospy.Time(0))
				(trans_adl,_) = self._tfsub.lookupTransform('/yumi_body', '/adl', rospy.Time(0))
				(trans_adll,_) = self._tfsub.lookupTransform('/yumi_body', '/adll', rospy.Time(0))
				self.need_adj = True
			except (TimeoutError, tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
				pass
			finally:
				signal.signal(signal.SIGALRM, signal.SIG_IGN)

			if(self.need_adj == True):
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
				yumi.plan_and_move_dual(yumi.create_pose_euler(x_l, y_l, 0.25, -pi/4, pi, pi), yumi.create_pose_euler(x_r, y_r, 0.25, pi/4, pi, pi))
				yumi.reset_arm_home(BOTH)
				yumi.reset_arm_cal(BOTH)
				self.need_adj = False
				rospy.sleep(3)

			else:
				try:
					(trans,_) = self._tfsub.lookupTransform('/yumi_body', '/shoe_hole', rospy.Time(0))
					(trans_norm,_) = self._tfsub.lookupTransform('/yumi_body', '/norm_shoe_hole', rospy.Time(0))
				except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
					continue
				x = trans[0]
				y = trans[1]
				z = trans[2]
				xn = trans_norm[0]
				yn = trans_norm[1]
				zn = trans_norm[2]
		
				if(0<xn<=x and zn>=z>0):
					a = np.arctan2((zn - z),(y - yn))
					b = np.arctan2((x - xn),(zn - z)) + pi

					if(a>=0 and a<=pi and b>=pi and b<=1.5*pi):
						zoffset = gripperoff*np.sin(a)*np.cos(b - pi) + zoff
						yoffset = yoff - (gripperoff)*np.cos(a)
						xoffset = xoff - (gripperoff)*np.sin(b - pi)
						a = pi/2 - a
						#set up 6D pose----------------
						x = x+xoffset
						y = y+yoffset
						z = z+zoffset
						xn = xn+xoffset
						yn = yn+yoffset
						zn = zn+zoffset
						print (x, y, z, xn, yn, zn, a, b)

						if(np.isnan(a)==False and np.isnan(b)==False):
							pose_norm = [xn, yn, zn, a, b, pi]
							pose = [x, y, z, a, b, pi]
							yumi.reset_arm(RIGHT)
							yumi.move_and_grasp(yumi.RIGHT, pose_norm, 10.0)
							yumi.move_and_grasp(yumi.RIGHT, pose, -10.0)
							yumi.move_and_grasp(yumi.RIGHT, pose_norm, -10.0)
							yumi.reset_arm(RIGHT)
							yumi.reset_arm_cal(RIGHT)
							rospy.on_shutdown(done)
					

if __name__ == '__main__':
	rospy.init_node("adjust_shoe", anonymous=True)
	follower=adjust_shoe()
	rospy.spin()
