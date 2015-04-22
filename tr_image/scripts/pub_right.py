#!/usr/bin/env python

import roslib
import sys
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import cv2
import cv
from sensor_msgs.msg import Image


rospy.init_node('test_right')         
template = cv2.imread('myright.jpg',1)

bridge = CvBridge()
newim=bridge.cv2_to_imgmsg(template, "bgr8")
image_pub = rospy.Publisher("image_right",Image,queue_size=10)    
cv2.waitKey(3)

while 1:
    try:
        image_pub.publish(newim)
    except CvBridgeError, e:
        print e 
        
        

 
     

    

