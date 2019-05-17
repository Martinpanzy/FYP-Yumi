#!/usr/bin/env python
import rospy, sys, numpy as np
import cv2, cv_bridge
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import Image, PointCloud2
from darknet_ros_msgs.msg import BoundingBoxes
import tf

lower_blue = np.array([110, 50, 50])
upper_blue = np.array([130, 255, 255])
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
		self.depth_sub = rospy.Subscriber('/camera/depth_registered/points', PointCloud2, self.depth_callback)
		self.image_sub = rospy.Subscriber('/camera/rgb/image_raw', Image, self.image_callback)
		self.bbx_sub = rospy.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes, self.bbx_callback)		
		#self.cxy_pub = rospy.Publisher('cxy', Tracker, queue_size=1)

	def depth_callback(self,data):
		#data_out = pc2.read_points(data, field_names = None, skip_nans = True)
		data_out = pc2.read_points(data, field_names = None, skip_nans = True, uvs = [(self.cx, self.cy)])
		
		int_data = list(data_out)
		if len(int_data) > 0:
			(point_x, point_y, point_z, _) = int_data[0]
			object_tf = [point_z, -point_x, -point_y]
			frame = 'camera_link'
			print object_tf
			self._tfpub.sendTransform((object_tf), tf.transformations.quaternion_from_euler(0, 0, 0), rospy.Time.now(), "point_centroid", frame)

	def bbx_callback(self,bx):
		for box in bx.bounding_boxes:
			if(box.Class == 'mouse'):
				self.xmin = box.xmin
				self.ymin = box.ymin
				self.xmax = box.xmax
				self.ymax = box.ymax
				#print self.xmin, self.ymin, self.xmax, self.ymax

	def image_callback(self,msg):
		image = self.bridge.imgmsg_to_cv2(msg,'bgr8')
		image = image[self.ymin:self.ymax, self.xmin:self.xmax]
		blurred = cv2.GaussianBlur(image, (11, 11), 0)
		hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

		mask = cv2.inRange(hsv, lower_blue, upper_blue)
		#mask = cv2.inRange(hsv, lower_red, upper_red)
		#mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
		#mask = cv2.inRange(hsv, lower_green, upper_green)

		mask = cv2.erode(mask, None, iterations=2)
		mask = cv2.dilate(mask, None, iterations=2)

		(_, cnts, _) = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
		h, w, d = image.shape #480, 640, 3

		if len(cnts) > 0:
			cnt = max(cnts, key=cv2.contourArea)
			area = cv2.contourArea(cnt)
			#print area
			M = cv2.moments(cnt)
			if M['m00'] > 0:
				cx = int(M['m10']/M['m00'])
				cy = int(M['m01']/M['m00'])
			if area > 150 and area < 350:
				self.cx = cx
				self.cy = cy
#'''
				cv2.circle(image, (cx, cy), 10, (0,0,0), -1)
				cv2.putText(image, "({}, {})".format(int(cx), int(cy)), (int(cx-5), int(cy+15)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
				cv2.drawContours(image, cnt, -1, (255, 255, 255),1)

		cv2.namedWindow("window", 1)
		cv2.imshow("window", image)
		cv2.waitKey(1)
#'''

if __name__ == '__main__':
	rospy.init_node("kinect_vision", anonymous=True)
	follower=kinect_vision()
	rospy.spin()
