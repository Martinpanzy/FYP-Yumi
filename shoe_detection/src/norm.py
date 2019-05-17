#!/usr/bin/env python
import rospy, sys, numpy as np
import cv2, cv_bridge
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import Image, PointCloud2
import tf
import pcl
import ros_numpy

def depth_callback(cloud):
	'''	
	pc = pc2.read_points(cloud, field_names = ('x', 'y', 'z'), skip_nans = True)
	pc_list = []
	for p in pc:
		pc_list.append([p[0], p[1], p[2]])
	p = pcl.PointCloud()
	p.from_list(pc_list)
	'''

	#pc = ros_numpy.numpify(cloud)
	#pts = np.zeros((pc.shape[0],3))
	#print(np.shape(pc))
	#pts[:,0] = pc['x']
	#pts[:,1] = pc['y']
	#pts[:,2] = pc['z']
	
	pts = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(cloud)	
	p = pcl.PointCloud(np.array(pts, dtype = np.float32))

	seg = p.make_segmenter_normals(ksearch=50)
	seg.set_optimize_coefficients(True)
	seg.set_model_type(pcl.SACMODEL_NORMAL_PLANE)
	seg.set_method_type(pcl.SAC_RANSAC)
	seg.set_distance_threshold(0.01)
	seg.set_normal_distance_weight(0.01)
	seg.set_max_iterations(100)
	indices, coefficients = seg.segment()
	print('!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!')
	print('Model coefficients: ' + str(coefficients[0]) + ' ' + str(coefficients[1]) + ' ' + str(coefficients[2]) + ' ' + str(coefficients[3]))

def listener():
	rospy.init_node("normalnormal", anonymous=True)
	rospy.Subscriber('/camera/depth_registered/points', PointCloud2, depth_callback)
	rospy.spin()
if __name__ == '__main__':
	listener()
