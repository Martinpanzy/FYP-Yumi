#!/usr/bin/env python
import rospy, sys, numpy as np
import cv2, cv_bridge
import sensor_msgs.point_cloud2 as pc2
import tf
import ros_numpy
import pcl
from std_msgs.msg import String
from sensor_msgs.msg import Image, PointCloud2
from darknet_ros_msgs.msg import BoundingBoxes
from math import pi
import time
#from sklearn.cluster import KMeans

#lower_blue = np.array([110, 50, 50])
#upper_blue = np.array([130, 255, 255])
lower_blue = np.array([100, 100, 50])
upper_blue = np.array([120, 255, 255])

lower_red = np.array([136,87,111])
upper_red = np.array([180,255,255])
lower_yellow = np.array([22,60,200])
upper_yellow = np.array([60,255,255])
lower_green = np.array([29, 86, 6])
upper_green = np.array([64, 255, 255])

class kinect_vision:
	def __init__(self):
		self.cx = 400
		self.cy = 400
		self._tfpub = tf.TransformBroadcaster()
		self.bridge = cv_bridge.CvBridge()	
		self.depth_sub = rospy.Subscriber('/zed/zed_node/point_cloud/cloud_registered', PointCloud2, self.depth_callback)
		self.image_sub = rospy.Subscriber('/zed/zed_node/rgb/image_rect_color', Image, self.image_callback)
		self.bbx_sub = rospy.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes, self.bbx_callback)
		self.state_pub = rospy.Publisher('state', String, queue_size = 10)
		self.coe = np.empty((0,3))	
		self.shoehole = np.empty((0,3))
		self.need_adj = False

	#get 6D pose of hole(x,y)-------------------------------------------------------------
	def depth_callback(self,data):
		if(self.need_adj == False):
			self.state_pub.publish('centroid')
			#print('centroid')
			#z depth----------------------------------------------------------------------
			holes = np.empty((0,3))	

			pc = ros_numpy.numpify(data) #pc[480, 640]
			for pps in self.pixelpoints:
					[x, y, z, _] = pc[pps[0]+self.ymin,pps[1]+self.xmin]
					if(np.isnan(x)==False and np.isnan(y)==False and np.isnan(z)==False):
						hole = np.array([x, y, z])
						holes = np.vstack((holes, hole))

			if len(self.shoehole) < 3:
				[cx, cy, cz, _] = pc[self.cy, self.cx]
				if(np.isnan(cx)==False and np.isnan(cz)==False):
					#holepose = np.mean(holes, axis=0)
					#shoehole = [cz, -cx, holepose[2]]
					shoehole = [cx, cy, cz]
					self.shoehole = np.vstack((self.shoehole, shoehole))
			else:
				self.shoehole = np.mean(self.shoehole, axis=0)
				self._tfpub.sendTransform((self.shoehole), tf.transformations.quaternion_from_euler(0, 0, 0), rospy.Time.now(), "shoe_hole", 'zed_left_camera_frame')
				self.shoehole = np.empty((0,3))

			
			#3D orientation----------------------------------------------------------------
			p = pcl.PointCloud(np.array(holes, dtype = np.float32))

			seg = p.make_segmenter_normals(ksearch=300)
			seg.set_optimize_coefficients(True)
			seg.set_model_type(pcl.SACMODEL_NORMAL_PLANE)
			seg.set_method_type(pcl.SAC_RANSAC)
			seg.set_distance_threshold(0.01)
			#seg.set_normal_distance_weight(0.01)
			seg.set_max_iterations(200)
			indices, coefficients = seg.segment()
			#print('Model coefficients: ' + str(coefficients[0]) + ' ' + str(coefficients[1]) + ' ' + str(coefficients[2]) + ' ' + str(coefficients[3]))

			if len(self.coe) < 5:	
				if(np.isnan(coefficients[0])==False and np.isnan(coefficients[1])==False and np.isnan(coefficients[2])==False and coefficients[0]>0):
					self.coe = np.vstack((self.coe, [coefficients[0], coefficients[1], coefficients[2]]))
			else:
					#kmeans = KMeans(n_clusters = 5).fit(self.coe)
					#print(len(kmeans.labels_))
					#l = np.bincount(kmeans.labels_).argmax()
					#self.coe = self.coe[np.where(kmeans.labels_ == l)]
		
					self.coe = np.mean(self.coe, axis=0)
					ppp = [-0.1*self.coe[0], -0.1*self.coe[1], -0.1*self.coe[2]]
					self._tfpub.sendTransform((ppp), tf.transformations.quaternion_from_euler(0, 0, pi), rospy.Time.now(), "pre_put", 'shoe_hole')
					pick = [0.045*self.coe[0], 0.045*self.coe[1], 0.045*self.coe[2]]
					self._tfpub.sendTransform((pick), tf.transformations.quaternion_from_euler(0, 0, pi), rospy.Time.now(), "pick", 'shoe_hole')
					self.coe = np.empty((0,3))

		#adjustment waypoint----------------------------------------------------------
		else:
			self.state_pub.publish('adjust')
			#print('adjust')
			adr = pc2.read_points(data, field_names = ('x', 'y', 'z'), skip_nans = True, uvs = [(self.adr_x, self.adr_y)])
			adrr = pc2.read_points(data, field_names = ('x', 'y', 'z'), skip_nans = True, uvs = [(self.adr_xx, self.adr_y)])
			adl = pc2.read_points(data, field_names = ('x', 'y', 'z'), skip_nans = True, uvs = [(self.adl_x, self.adl_y)])
			adll = pc2.read_points(data, field_names = ('x', 'y', 'z'), skip_nans = True, uvs = [(self.adl_xx, self.adl_y)])
			adr = list(adr)
			adrr = list(adrr)
			adl = list(adl)
			adll = list(adll)
			self.need_adj = False
			if len(adr) > 0 and len(adl) > 0 and len(adll) > 0 and len(adrr) > 0:
				adr_x, adr_y, adr_z = adr[0]
				adr_xx, adr_yy, adr_zz = adrr[0]
				adl_x, adl_y, adl_z = adl[0]
				adl_xx, adl_yy, adl_zz = adll[0]

				adr = [adr_x, adr_y, adr_z]
				adrr = [adr_xx, adr_yy, adr_zz]
				adl = [adl_x, adl_y, adl_z]
				adll = [adl_xx, adl_yy, adl_zz]
				self._tfpub.sendTransform((adr), tf.transformations.quaternion_from_euler(0, 0, 0), rospy.Time.now(), "adr", 'zed_left_camera_frame')
				self._tfpub.sendTransform((adrr), tf.transformations.quaternion_from_euler(0, 0, 0), rospy.Time.now(), "adrr", 'zed_left_camera_frame')
				self._tfpub.sendTransform((adl), tf.transformations.quaternion_from_euler(0, 0, 0), rospy.Time.now(), "adl", 'zed_left_camera_frame')
				self._tfpub.sendTransform((adll), tf.transformations.quaternion_from_euler(0, 0, 0), rospy.Time.now(), "adll", 'zed_left_camera_frame')
	#find shoe bounding box------------------------------------------------------------------
	def bbx_callback(self,bx):
		for box in bx.bounding_boxes:
			if(box.Class == 'footwear' or box.Class == 'shoe'):
				self.xmin = box.xmin
				self.ymin = box.ymin
				self.xmax = box.xmax
				self.ymax = box.ymax
				box_h = box.ymax - box.ymin
				box_w = box.xmax - box.xmin
				if (box_h >= 1.8* box_w):
					self.adr_x = int(box.xmax + 60)
					self.adr_xx = int(box.xmin + 0.15*box_w)
					self.adr_y = int(box.ymax - 0.15*box_h)

					self.adl_x = int(box.xmin - 60)
					self.adl_xx = int(box.xmax - 0.15*box_w)
					self.adl_y = int(box.ymin + 0.25*box_h)
					self.need_adj = True

	#find shoe hole x,y position-------------------------------------------------------------
	def image_callback(self,msg):
		#self.start = time.clock()
		image = self.bridge.imgmsg_to_cv2(msg,'bgr8')
		image = image[self.ymin:self.ymax, self.xmin:self.xmax]
		blurred = cv2.GaussianBlur(image, (11, 11), 0)
		#blurred = cv2.medianBlur(image, 11)
		#blurred = cv2.bilateralFilter(image, 11, 75, 75)
		#blurred = cv2.boxFilter(image, -1, (5,5))
		hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

		mask = cv2.inRange(hsv, lower_blue, upper_blue)

		mask1 = cv2.erode(mask, None, iterations=1)
		mask2 = cv2.dilate(mask1, None, iterations=1)

		(_, cnts, _) = cv2.findContours(mask2.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
		h, w, d = image.shape

		if len(cnts) > 0:
			cnt = max(cnts, key=cv2.contourArea)
			area = cv2.contourArea(cnt)
			#print area
			M = cv2.moments(cnt)
			if M['m00'] > 0:
				cx = int(M['m10']/M['m00'])
				cy = int(M['m01']/M['m00'])
			#if area > 80 and area < 350:
				self.pixelpoints = np.transpose(np.nonzero(mask2))	
				#print(len(self.pixelpoints))
				self.cx = cx + self.xmin
				self.cy = cy + self.ymin
				#print time.clock() - self.start
		
#'''
				cv2.circle(image, (cx, cy), 3, (0,0,255), -1)
				#cv2.putText(image, "({}, {})".format(int(cx), int(cy)), (int(cx-5), int(cy+15)), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 1)
				cv2.drawContours(image, cnt, -1, (255, 255, 255),1)

		cv2.imshow("image", image)
		#cv2.imshow("blurred", blurred)
		#cv2.imshow("hsv", hsv)
		#cv2.imshow("mask", mask)
		#cv2.imshow("mask1", mask1)
		#cv2.imshow("mask2", mask2)
		cv2.waitKey(1)
#'''

#-----------------------------------------------------------------------------------------------------
if __name__ == '__main__':
	rospy.init_node("kinect_vision", anonymous=True)
	follower=kinect_vision()
	rospy.spin()
