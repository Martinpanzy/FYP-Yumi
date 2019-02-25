#!/usr/bin/env python
import rospy, sys, numpy as np
import moveit_commander
from copy import deepcopy
import geometry_msgs.msg
#from ur5_notebook.msg import Tracker
import moveit_msgs.msg
import cv2, cv_bridge
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
#tracker = Tracker()

class kinect_vision:
    def __init__(self):
        rospy.init_node("kinect_vision", anonymous=True)
        self.track_flag = False
        self.default_pose_flag = True
        self.cx = 400.0
        self.cy = 400.0
        self.bridge = cv_bridge.CvBridge()
        self.image_sub = rospy.Subscriber('/camera/rgb/image_raw', Image, self.image_callback)
        #self.cxy_pub = rospy.Publisher('cxy', Tracker, queue_size=1)


    def image_callback(self,msg):
        image = self.bridge.imgmsg_to_cv2(msg,'bgr8')
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        lower_red = np.array([110, 50, 50])
        upper_red = np.array([130, 255, 255])  
	mask = cv2.inRange(hsv, lower_red, upper_red)
        (_, cnts, _) = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        #area = cv2.contourArea(cnts)
        h, w, d = image.shape
        
	#BEGIN FINDER
        M = cv2.moments(mask)
        if M['m00'] > 0:
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
        # cx range (55,750) cy range( 55, ~ )
        # END FINDER

        # Isolate largest contour
        #  contour_sizes = [(cv2.contourArea(contour), contour) for contour in cnts]
        #  biggest_contour = max(contour_sizes, key=lambda x: x[0])[1]
	for i, c in enumerate(cnts):
		area = cv2.contourArea(c)
		if area > 300:
		    self.track_flag = True
		    self.cx = cx
		    self.cy = cy
		    self.error_x = self.cx - w/2
		    self.error_y = self.cy - (h/2+195)
		    #tracker.x = cx
		    #tracker.y = cy
		    #tracker.flag1 = self.track_flag
		    #tracker.error_x = self.error_x
		    #tracker.error_y = self.error_y
		    
		    #(_,_,w_b,h_b)=cv2.boundingRect(c)
		    #print w_b,h_b
		    # BEGIN circle
		    cv2.circle(image, (cx, cy), 10, (0,0,0), -1)
		    cv2.putText(image, "({}, {})".format(int(cx), int(cy)), (int(cx-5), int(cy+15)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
		    cv2.drawContours(image, cnts, -1, (255, 255, 255),1)
		    break
		else:
		    self.track_flag = False


        #self.cxy_pub.publish(tracker)
        cv2.namedWindow("window", 1)
        cv2.imshow("window", image )
        cv2.waitKey(1)
	
	cv2.namedWindow("window2", 1)
	cv2.imshow("window2", mask )
        cv2.waitKey(1)

follower=kinect_vision()
rospy.spin()
