#!/usr/bin/env python
import rospy, sys, numpy as np
import cv2, cv_bridge
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import Image, PointCloud2
import tf

class kinect_vision:
    def __init__(self):
        self.cx = 400.0
        self.cy = 400.0
	self._tfpub = tf.TransformBroadcaster()
        self.bridge = cv_bridge.CvBridge()
	self.depth_sub = rospy.Subscriber('/camera/depth_registered/points', PointCloud2, self.depth_callback)
        self.image_sub = rospy.Subscriber('/camera/rgb/image_raw', Image, self.image_callback)
        #self.cxy_pub = rospy.Publisher('cxy', Tracker, queue_size=1)

    def depth_callback(self,data):
	data_out = pc2.read_points(data, field_names = None, skip_nans = False, uvs = [(self.cx, self.cy)])
	int_data = list(data_out)
	if len(int_data) > 0:
		publish_tf = True
		(point_x, point_y, point_z, _) = int_data[0]
	if publish_tf:
		object_tf = [point_z, -point_x, -point_y]
		frame = 'camera_link'
		self._tfpub.sendTransform((object_tf), tf.transformations.quaternion_from_euler(0, 0, 0), rospy.Time.now(), "point_centroid", frame)

    def image_callback(self,msg):
        image = self.bridge.imgmsg_to_cv2(msg,'bgr8')
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        lower_red = np.array([110, 50, 50])
        upper_red = np.array([130, 255, 255])
	mask = cv2.inRange(hsv, lower_red, upper_red)
	(_, cnts, _) = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        h, w, d = image.shape
        
	#BEGIN FINDER
        M = cv2.moments(mask)
        if M['m00'] > 0:
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
        # cx range (55,750) cy range( 55, ~ )
        # END FINDER

        # Isolate largest contour
        # contour_sizes = [(cv2.contourArea(contour), contour) for contour in cnts]
        # biggest_contour = max(contour_sizes, key=lambda x: x[0])[1]
	for i, c in enumerate(cnts):
		area = cv2.contourArea(c)
		if area > 300:
		    self.cx = cx
		    self.cy = cy
		    #self.error_x = self.cx - w/2
		    #self.error_y = self.cy - (h/2+195)

if __name__ == '__main__':
	rospy.init_node("kinect_vision", anonymous=True)
	follower=kinect_vision()
	rospy.spin()
