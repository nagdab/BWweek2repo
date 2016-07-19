#!/usr/bin/env python
import cv2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class Echo:
    def __init__(self):
        self.node_name = "Echo"
        self.sub_image = rospy.Subscriber("/camera/rgb/image_rect_color",Image, self.cbImage, queue_size=1)
        self.pub_image = rospy.Publisher("~echo_image",Image, queue_size=1)
        self.bridge = CvBridge()

        rospy.loginfo("[%s] Initialized." %(self.node_name))

    def cbImage(self,image_msg):
        image_cv = self.bridge.imgmsg_to_cv2(image_msg)
        self.pub_image.publish(self.bridge.cv2_to_imgmsg(image_cv, "bgr8"))


if __name__=="__main__":
    rospy.init_node('Echo')
    e = Echo()
    rospy.spin()
