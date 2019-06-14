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
	def state_callback(self,data):
		if(data.data == 'adjust'):
			self.need_adj = True
		else:
			self.need_adj = False

	def __init__(self):
		self._tfsub = tf.TransformListener()
		self.state_lis = rospy.Subscriber('/state', String, self.state_callback)
		yumi.init_Moveit()

		while not rospy.is_shutdown():
			start = time.time()
			if(self.need_adj == True):
				try:
					(trans_adr,_) = self._tfsub.lookupTransform('/yumi_base_link', '/adr', rospy.Time(0))
					(trans_adrr,_) = self._tfsub.lookupTransform('/yumi_base_link', '/adrr', rospy.Time(0))
					(trans_adl,_) = self._tfsub.lookupTransform('/yumi_base_link', '/adl', rospy.Time(0))
					(trans_adll,_) = self._tfsub.lookupTransform('/yumi_base_link', '/adll', rospy.Time(0))
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
				self.need_adj = False
				yumi.reset_arm_home(BOTH)
				yumi.plan_and_move_dual(yumi.create_pose_euler(x_l, y_l, 0.25, -pi/4, pi, pi), yumi.create_pose_euler(x_r, y_r, 0.25, pi/4, pi, pi))
				yumi.plan_and_move_dual(yumi.create_pose_euler(x_l, y_l, 0.1, -pi/4, pi, pi), yumi.create_pose_euler(x_r, y_r, 0.1, pi/4, pi, pi))
				yumi.plan_and_move_dual(yumi.create_pose_euler(x_l, y_ll, 0.1, -pi/4, pi, pi), yumi.create_pose_euler(x_r, y_rr, 0.1, pi/4, pi, pi))
				yumi.plan_and_move_dual(yumi.create_pose_euler(x_l, y_l, 0.1, -pi/4, pi, pi), yumi.create_pose_euler(x_r, y_r, 0.1, pi/4, pi, pi))
				yumi.plan_and_move_dual(yumi.create_pose_euler(x_l, y_l, 0.25, -pi/4, pi, pi), yumi.create_pose_euler(x_r, y_r, 0.25, pi/4, pi, pi))
				yumi.reset_arm_home(BOTH)
				yumi.reset_arm_cal(BOTH)
				rospy.sleep(3)
				
			else:
				try:
					(trans,_) = self._tfsub.lookupTransform('/yumi_base_link', '/shoe_hole', rospy.Time(0))
					(trans_norm,_) = self._tfsub.lookupTransform('/yumi_base_link', '/pre_put', rospy.Time(0))
					(trans_pick,_) = self._tfsub.lookupTransform('/yumi_base_link', '/pick', rospy.Time(0))
				except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
					continue
				x = trans[0]
				y = trans[1]
				z = trans[2]
				xn = trans_norm[0]
				yn = trans_norm[1]
				zn = trans_norm[2]
				xp = trans_pick[0] 
				yp = trans_pick[1] 
				zp = trans_pick[2]
		
				if(0<xn<=x and zn>=z>0 and 0<x<xp):
					a = np.arctan2((y - yn),(zn - z))
					b = np.arctan2((x - xn),(zn - z))
					c = np.arctan2((yp - y),(xp - x))
			
					if(-0.5*pi<=a<=0.5*pi and 0<=b<=0.5*pi and -0.5*pi<=c<=0.5*pi):
						zoffset = gripperoff*np.cos(a)*np.cos(b) + zoff
						yoffset = yoff - (gripperoff)*np.sin(a)
						xoffset = xoff - (gripperoff)*np.sin(b)
						b = b + pi
						c = 0.5*pi+c-0.10
						#set up 6D pose----------------
						x = x+xoffset
						y = y+yoffset
						z = z+zoffset
						xn = xn+xoffset
						yn = yn+yoffset
						zn = zn+zoffset
						xp = xp+xoff
						yp = yp+yoff
						zp = zp+zoff+gripperoff
						print (x, y, z, xn, yn, zn, a, b)

						if(np.isnan(a)==False and np.isnan(b)==False):
							#'''
							pose_norm = [xn, yn, zn, a, b, pi]
							pose = [x, y, z, a, b, pi]
							yumi.reset_arm(RIGHT)
							yumi.move_and_grasp(yumi.RIGHT, pose_norm, 10.0)
							yumi.move_and_grasp(yumi.RIGHT, pose, -10.0)
							yumi.move_and_grasp(yumi.RIGHT, pose_norm, -10.0)
							yumi.reset_arm(RIGHT)
							yumi.reset_arm_cal(RIGHT)
							#'''
							pose_pick = [xp, yp, 0.18+0.08, 0, pi, c]
							pose_grab = [xp, yp, 0.18, 0, pi, c]
							yumi.reset_arm_home(LEFT)
							yumi.move_and_grasp(yumi.LEFT, pose_pick, -10.0)
							yumi.move_and_grasp(yumi.LEFT, pose_grab, 10.0)
							yumi.move_and_grasp(yumi.LEFT, pose_pick, 10.0)
							yumi.reset_arm_home(LEFT)
							yumi.reset_arm_cal(LEFT)
							#'''
							print time.time() - start
							rospy.on_shutdown(done)

if __name__ == '__main__':
	rospy.init_node("go_zed", anonymous=True)
	follower=go_zed()
	rospy.spin()
