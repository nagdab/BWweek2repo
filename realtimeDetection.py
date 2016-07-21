#!/usr/bin/env python
import cv2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np;
import imutils

def detecting(image_cv):
	hsv = cv2.cvtColor(image_cv, cv2.COLOR_BGR2HSV)

	r1 = np.array([0, 220, 200])
	r2 = np.array([15, 255, 230])
	
	mask = cv2.inRange(hsv, r1, r2)

	mask = cv2.GaussianBlur(mask, (21,21), 0)
	mask = cv2.erode(mask, (3, 3), iterations=5)
	contours, h = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

	#cv2.drawContours(image_cv, contours, -1, (0, 255, 0))

	if (len(contours) > 0):
		contour = contours[0]

		for j in range(0, len(contours)):
		    if(cv2.contourArea(contours[j]) > cv2.contourArea(contour)):
			countour = contours[j]


		
		#rect = cv2.minAreaRect(contours[j])
		#box = cv2.cv.BoxPoints(rect)
		    #box = np.int0(box)
		    #cv2.drawContours(image_cv,[box],0,(0,0,255),2)

		M = cv2.moments(contour)
		cX = int(M["m10"] / M["m00"])
		cY = int(M["m01"] / M["m00"])

		# draw the contour and center of the shape on the image

		cv2.drawContours(image_cv, contour, -1, (0, 255, 0), 2)
		cv2.circle(image_cv, (cX, cY), 4, (255, 255, 255), -1)
		cv2.putText(image_cv, "center", (cX - 20, cY - 20),
		cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

	return image_cv

class Echo:
    def __init__(self):
        self.node_name = "Echo"
        self.sub_image = rospy.Subscriber("/camera/rgb/image_rect_color",Image, self.cbImage, queue_size=1)
        self.pub_image = rospy.Publisher("~echo_image",Image, queue_size=1)
        self.bridge = CvBridge()

        rospy.loginfo("[%s] Initialized." %(self.node_name))


    def cbImage(self,image_msg):
        image_cv = self.bridge.imgmsg_to_cv2(image_msg)

	images = detecting(image_cv)

        self.pub_image.publish(self.bridge.cv2_to_imgmsg(images, "bgr8"))


if __name__=="__main__":
    rospy.init_node('Echo')
    e = Echo()
    rospy.spin()
