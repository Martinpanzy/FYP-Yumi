#!/usr/bin/env python
import rospy, sys, numpy as np
import cv2, cv_bridge
from darknet_ros_msgs.msg import BoundingBoxes

class ROI_roi:
	def __init__(self):
		#self.bridge = cv_bridge.CvBridge()
		#self.depth_sub = rospy.Subscriber('/camera/depth_registered/points', PointCloud2, self.depth_callback)
		#self.image_sub = rospy.Subscriber('/camera/rgb/image_raw', Image, self.image_callback)
		self.bbx_sub = rospy.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes, self.bbx_callback)

	def bbx_callback(self,bx):
		for box in bx.bounding_boxes:
			#print(box)
			if(box.Class == 'mouse'):
				x = box.xmin
				y = box.ymin
				xx = box.xmax
				yy = box.ymax
				print x, y, xx, yy


if __name__ == '__main__':
	rospy.init_node("ROI_roi", anonymous=True)
	follower=ROI_roi()
	rospy.spin()
