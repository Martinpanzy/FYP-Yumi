#!/usr/bin/env python
import rospy, sys, numpy as np
import cv2, cv_bridge
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import Image, PointCloud2
import tf
import pcl

def depth_callback(cloud):
	ne = cloud.make_IntegralImageNormalEstimation()
	print ('set_NormalEstimation_Method_AVERAGE_3D_GRADIENT: ')
	ne.set_NormalEstimation_Method_AVERAGE_3D_GRADIENT ()
	print ('set_MaxDepthChange_Factor: ')
	ne.set_MaxDepthChange_Factor(0.02)
	print ('set_NormalSmoothingSize: ')
	ne.set_NormalSmoothingSize(10.0)
	print ('set OK')
	print ('compute2 - start')
	normals = ne.compute2(cloud)
	print ('compute2 - end')
	print (str(normals.size))

def listener():
	rospy.init_node("normalnormal", anonymous=True)
	rospy.Subscriber('/camera/depth_registered/points', p, depth_callback)
	rospy.spin()
if __name__ == '__main__':
	listener()
