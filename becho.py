#!/usr/bin/env python
import cv2
import rospy
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Point
from racecar.msg import BlobDetections
import threading


class Echo:
    def __init__(self):
        self.node_name = "Echo"
        self.thread_lock = threading.Lock()
        self.sub_image = rospy.Subscriber("/camera/rgb/image_rect_color",\
                Image, self.cbImage, queue_size=1)
        self.pub_image = rospy.Publisher("~echo_image",\
                Image, queue_size=1)
	self.pub_blob = rospy.Publisher("blob_detections", BlobDetections, queue_size=1)
        self.bridge = CvBridge()

        rospy.loginfo("[%s] Initialized." %(self.node_name))

    def cbImage(self,image_msg):
        thread = threading.Thread(target=self.processImage,args=(image_msg,))
        thread.setDaemon(True)
        thread.start()


    def processImage(self, image_msg):
        if not self.thread_lock.acquire(False):
            return
        image_cv = self.bridge.imgmsg_to_cv2(image_msg)

	# Image processing starts here
	#filters = [np.array([0, 230, 170]), np.array([6, 255, 255])] # Red
	filters = [np.array([45, 127, 100]), np.array([67, 255, 255])] # Green

	image_hsv = cv2.cvtColor(image_cv, cv2.COLOR_BGR2HSV)
        
	mask = cv2.inRange(image_hsv, filters[0], filters[1])
	mask = cv2.GaussianBlur(mask, (3,3), 0)
	X = 0
	Y = 0
	contours, h = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
	if(len(contours) > 0):
		bcont = contours[0]
		for j in range(0, len(contours)):
			if(cv2.contourArea(contours[j]) > cv2.contourArea(bcont)):
				bcont = contours[j]
		cv2.drawContours(image_cv, [bcont], -1, (0, 255, 0))
		x,y,w,h = cv2.boundingRect(bcont)
		cv2.rectangle(image_cv,(x,y),(x+w,y+h),(0,255,0),2)
		cv2.circle(image_cv, (x+w/2, y+h/2), 4, (255, 255, 255), -1)
		X = (x+w/2)/float(len(image_cv[0]))
		Y = (y+h/2)/float(len(image_cv))
                blobs = BlobDetections()
		blobs.colors= [[0,0,0,0]]
		blobs.sizes = [cv2.contourArea(bcont)]
		p = Point()
		p.x = X
		p.y = Y
		p.z = 0
		blobs.locations = [p]	
		print X, " ", Y
	    	self.pub_blob.publish(blobs)
	# Image processing stops here
        try:
            self.pub_image.publish(self.bridge.cv2_to_imgmsg(image_cv, "bgr8"))
        except CvBridgeError as e:
            print(e)
        self.thread_lock.release()


if __name__=="__main__":
    rospy.init_node('Echo')
    e = Echo()
    rospy.spin()
