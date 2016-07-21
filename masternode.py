#!/usr/bin/env python
import cv2
import rospy
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import *
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

		filters_red = [np.array([0, 230, 170]), np.array([6, 255, 255])] # Red
		filters_green = [np.array([40, 100, 40]), np.array([88, 255, 255])] # Green

		# Image processing starts here
		image_hsv = cv2.cvtColor(image_cv, cv2.COLOR_BGR2HSV)
		    
		mask_red = cv2.inRange(image_hsv, filters_red[0], filters_red[1])
		mask_green = cv2.inRange(image_hsv, filters_green[0], filters_green[1])

		mask_red = cv2.GaussianBlur(mask_red, (3,3), 0)
		mask_green = cv2.GaussianBlur(mask_green, (3,3), 0)

		X = 0
		Y = 0
		contours_red, h_red = cv2.findContours(mask_red, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
		contours_green, h_green = cv2.findContours(mask_green, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

		test_red = len(contours_red) > 0
		test_green = len(contours_green) > 0


		c = None

		bcont = None

		if(test_red and test_green):
			bcont_red = contours_red[0]
			bcont_green = contours_green[0]

			for j in range(0, len(contours_red)):
				if(cv2.contourArea(contours_red[j]) > cv2.contourArea(bcont_red)):
					bcont_red = contours_red[j]

			for j in range(0, len(contours_green)):
				if(cv2.contourArea(contours_green[j]) > cv2.contourArea(bcont_green)):
					bcont_green = contours_green[j]
		
			if(cv2.contourArea(bcont_red) < cv2.contourArea(bcont_green)):
				c = std_msgs.msg.ColorRGBA(0.0, 255.0, 0.0, 128.0)
				bcont = bcont_green
			else:
				c = std_msgs.msg.ColorRGBA(255.0, 0.0, 0.0, 128.0)
				bcont = bcont_red



		elif(test_red):
			bcont_red = contours_red[0]
			for j in range(0, len(contours_red)):
				if(cv2.contourArea(contours_red[j]) > cv2.contourArea(bcont_red)):
					bcont_red = contours_red[j]
			std_msgs.msg.ColorRGBA(255.0, 0.0, 0.0, 128.0)
			bcont = bcont_red

		elif(test_green):
			bcont_green = contours_green[0]
			for j in range(0, len(contours_green)):
				if(cv2.contourArea(contours_green[j]) > cv2.contourArea(bcont_green)):
					bcont_green = contours_green[j]
			c = std_msgs.msg.ColorRGBA(0.0, 255.0, 0.0, 128.0)
			bcont = bcont_green

		if(test_red or test_green):

			cv2.drawContours(image_cv, [bcont], -1, (0, 255, 0))
			x,y,w,h = cv2.boundingRect(bcont)
			cv2.rectangle(image_cv,(x,y),(x+w,y+h),(0,255,0),2)
			cv2.circle(image_cv, (x+w/2, y+h/2), 4, (255, 255, 255), -1)
			X = (x+w/2)/float(len(image_cv[0]))
			Y = (y+h/2)/float(len(image_cv))
			blobs = BlobDetections()
			blobs.sizes = [Float64(cv2.contourArea(bcont))]
			p = Point()
			p.x = X
			p.y = Y
			p.z = 0
			blobs.colors = [c]
			blobs.locations = [p]	
			#print X, " ", Y
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
