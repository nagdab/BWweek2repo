#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image

def callBack(msg):
        pub.publish(msg)

scanResult = rospy.Subscriber("/camera/rgb/image_rect_color", Image, callBack)
pub = rospy.Publisher('image_echo', Image,queue_size=10)
rospy.init_node('echo')
rospy.spin()
