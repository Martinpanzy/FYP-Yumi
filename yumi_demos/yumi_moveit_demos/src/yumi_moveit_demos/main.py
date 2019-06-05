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

import tf

LEFT = 2        #:ID of the left arm
RIGHT = 1       #:ID of the right arm
BOTH = 3        #:ID of both_arms
xoff = -0.0205
yoff = -0.023
gripperoff = 0.136
zoff = 0.17 - gripperoff #0.16268


def move_and_grasp(arm, pose_ee, grip_effort):
    try:
        yumi.traverse_path([pose_ee], arm, 10)
    except Exception:
        if (arm == yumi.LEFT):
            yumi.plan_and_move(yumi.group_l, yumi.create_pose_euler(pose_ee[0], pose_ee[1], pose_ee[2], pose_ee[3], pose_ee[4], pose_ee[5]))
        elif (arm == yumi.RIGHT):
            yumi.plan_and_move(yumi.group_r, yumi.create_pose_euler(pose_ee[0], pose_ee[1], pose_ee[2], pose_ee[3], pose_ee[4], pose_ee[5]))

    if (grip_effort <= 20 and grip_effort >= -20):
        yumi.gripper_effort(arm, grip_effort)
    else:
        print("The gripper effort values should be in the range [-20, 20]")

#-----------------------------------------------------------------------------------------
def gogo():
    rospy.init_node('yumi_moveit_demo')
    yumi.init_Moveit()
    #yumi.print_current_joint_states(yumi.RIGHT)
    #yumi.print_current_joint_states(yumi.LEFT)
    yumi.reset_arm_home(BOTH)
    yumi.plan_and_move_dual(yumi.create_pose_euler(0.5133834823214563, 0.24797905921554042, 0.2, -pi/4, pi, pi), yumi.create_pose_euler(0.3424706232530676, -0.18748739397696912, 0.2, pi/4, pi, pi))

    yumi.plan_and_move_dual(yumi.create_pose_euler(0.5133834823214563, 0.24797905921554042, 0.1, -pi/4, pi, pi), yumi.create_pose_euler(0.3424706232530676, -0.18748739397696912, 0.1, pi/4, pi, pi))
    yumi.plan_and_move_dual(yumi.create_pose_euler(0.5133834823214563, 0.10721286835208456, 0.1, -pi/4, pi, pi), yumi.create_pose_euler(0.3424706232530676, -0.05191701573110484, 0.1, pi/4, pi, pi))
    yumi.plan_and_move_dual(yumi.create_pose_euler(0.5133834823214563, 0.24797905921554042, 0.1, -pi/4, pi, pi), yumi.create_pose_euler(0.3424706232530676, -0.18748739397696912, 0.1, pi/4, pi, pi))
    yumi.plan_and_move_dual(yumi.create_pose_euler(0.5133834823214563, 0.24797905921554042, 0.2, -pi/4, pi, pi), yumi.create_pose_euler(0.3424706232530676, -0.18748739397696912, 0.2, pi/4, pi, pi))

    yumi.reset_arm_home(BOTH)
    yumi.reset_arm_cal(BOTH)
    rospy.spin()



def run(pose_norm, pose):
    """Starts the node

    Runs to start the node and initialize everthing. Runs forever via Spin()

    :returns: Nothing
    :rtype: None
    """

    #rospy.init_node('yumi_moveit_demo')

    #Start by connecting to ROS and MoveIt!
    yumi.init_Moveit()


    # Print current joint angles
    #yumi.print_current_joint_states(yumi.RIGHT)
    #yumi.print_current_joint_states(yumi.LEFT)

    # Print current joint angles
    #yumi.print_current_pose(yumi.RIGHT)


    # Reset YuMi joints to "home" position
    #yumi.reset_pose()
    yumi.reset_arm(RIGHT)

    # Drive YuMi end effectors to a desired position (pose_ee), and perform a grasping task with a given effort (grip_effort)
    # Gripper effort: opening if negative, closing if positive, static if zero
    #pose_ee = [0.3, 0.15, 0.2, 0.0, 3.14, 3.14]
    #grip_effort = -10.0
    #move_and_grasp(yumi.LEFT, pose_ee, grip_effort)

    grip_effort = 10.0
    move_and_grasp(yumi.RIGHT, pose_norm, grip_effort)

    grip_effort = -10.0
    move_and_grasp(yumi.RIGHT, pose, grip_effort)

    move_and_grasp(yumi.RIGHT, pose_norm, grip_effort)

    yumi.reset_arm(RIGHT)
    yumi.reset_arm_cal(RIGHT)
    rospy.spin()

def tf_listener():
	rospy.init_node('yumi_tf_listener')
	listener = tf.TransformListener()
	receive = True
	while not rospy.is_shutdown():
		try:
			(trans,rot) = listener.lookupTransform('/yumi_body', '/shoe_hole', rospy.Time(0))
			(trans_norm,rot_norm) = listener.lookupTransform('/yumi_body', '/norm_shoe_hole', rospy.Time(0))
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
			else: 
				a = 0.0
				b = pi		
			
			
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

				while receive == True:
					receive = False
					run(pose_norm, pose)
			
				#receive = True
			
		
if __name__ == '__main__':
    try:
        gogo()
	#tf_listener()

    	print "####################################     Program finished     ####################################"
    except rospy.ROSInterruptException:
        pass
