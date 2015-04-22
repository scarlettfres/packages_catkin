#!/usr/bin/env python
import roslib
roslib.load_manifest('stereo_color_tracker')
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String, Header
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped, Point
from cv_bridge import CvBridge, CvBridgeError
# import IPython

class image_track:

  def __init__(self):
    # self.image_pub = rospy.Publisher("left_tracker_image",Image, queue_size=5)
    self.left_point_pub = rospy.Publisher("left_point", PointStamped, queue_size=5)
    self.right_point_pub = rospy.Publisher("right_point", PointStamped, queue_size=5)

    
    # cv2.namedWindow("Image window", 1)
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/left_cam/image_raw",Image,self.left_callback)
    self.image_sub = rospy.Subscriber("/right_cam/image_raw",Image,self.right_callback)

    # Trackbar stuff
    # Green Marker
    # self.lower_threshold = np.array([66, 97, 180])
    # self.upper_threshold = np.array([96, 222, 255])

    # Green cloth on a plastic stick
    # self.lower_threshold = np.array([37, 64, 73])
    # self.upper_threshold = np.array([63, 149, 233])

    # Green Marker 3d print table nov 26
    self.lower_threshold = np.array([60, 96, 131])
    self.upper_threshold = np.array([84, 221, 255])

    self.f = 640.0
    # 1280 x 960 image
    # self.center_x = (1280.0/2.0) # half x pixels
    # self.center_y = (960.0/2.0) # half y pixels
    # 640 x 480
    self.center_x = (640.0/2.0) # half x pixels
    self.center_y = (480.0/2.0) # half y pixels
    self.invCameraMatrix = np.matrix([[self.f, 0, self.center_x],
                                      [0, self.f, self.center_y],
                                      [0, 0, 1.0]]).I

    cv2.namedWindow("Control", cv2.CV_WINDOW_AUTOSIZE); # Threshold Controller window
    # cv2.namedWindow("Thresholded Image", cv2.CV_WINDOW_AUTOSIZE); # Threshold image window

    cv2.createTrackbar("LowH", "Control", self.lower_threshold[0], 255, self.updateLowH); # Hue (0 - 255)
    cv2.createTrackbar("HighH", "Control", self.upper_threshold[0], 255, self.updateHighH);
    cv2.createTrackbar("LowS", "Control", self.lower_threshold[1], 255, self.updateLowS); # Saturation (0 - 255)
    cv2.createTrackbar("HighS", "Control", self.upper_threshold[1], 255, self.updateHighS);
    cv2.createTrackbar("LowV", "Control", self.lower_threshold[2], 255, self.updateLowV); # Value (0 - 255)
    cv2.createTrackbar("HighV", "Control", self.upper_threshold[2], 255, self.updateHighV);
    cv2.createTrackbar("offset F", "Control", int(self.f), 3000, self.updateF);

  def updateLowH(self, value):
    self.lower_threshold[0] = value
  def updateHighH(self, value):
    self.upper_threshold[0] = value
  def updateLowS(self, value):
    self.lower_threshold[1] = value
  def updateHighS(self, value):
    self.upper_threshold[1] = value
  def updateLowV(self, value):
    self.lower_threshold[2] = value
  def updateHighV(self, value):
    self.upper_threshold[2] = value
  def updateF(self, value):
    self.f = float(value)
    self.invCameraMatrix = np.matrix([[self.f, 0, self.center_x],
                                          [0, self.f, self.center_y],
                                          [0, 0, 1.0]]).I
  def left_callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError, e:
      print e

    # IPython.embed()

    # Get HSV image
    hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

    # Threshold image to range
    mask = cv2.inRange(hsv, self.lower_threshold, self.upper_threshold)

    # Erode/Dilate mask to remove noise
    mask = cv2.erode(mask, cv2.getStructuringElement(cv2.MORPH_ERODE, (3,3) ))
    mask = cv2.dilate(mask, cv2.getStructuringElement(cv2.MORPH_DILATE, (3,3) ))

    # Mask image

    # cv_image = cv2.bitwise_and(cv_image, cv_image, mask=mask)

    # Use Mask to get blob information
    moments = cv2.moments(mask)
    area = moments['m00']
    if (area > 0):
      cx = int(moments['m10']/moments['m00']) # cx = M10/M00
      cy = int(moments['m01']/moments['m00']) # cy = M01/M00
      # cv2.circle(cv_image, (cx,cy), 10, (0,0,255))
      self.postLeftPoint(cx,cy) # Publish it
      # cv2.putText(cv_image,"Area: %10d, X: %3d, Y: %3d" % (area, cx, cy), (10,20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,0)) # bgr

    # (rows,cols,channels) = cv_image.shape

    # cv2.imshow("Thresholded Image", cv_image)
    k = cv2.waitKey(3) & 0xFF
    if k == 113 or k == 27: # Escape key = 27, 'q' = 113
      rospy.signal_shutdown("User Exit")

    # try:
    #   self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
    # except CvBridgeError, e:
    #   print e

  def right_callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError, e:
      print e

    # Get HSV image
    hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

    # Threshold image to range
    mask = cv2.inRange(hsv, self.lower_threshold, self.upper_threshold)

    # Erode/Dilate mask to remove noise
    mask = cv2.erode(mask, cv2.getStructuringElement(cv2.MORPH_ERODE, (3,3) ))
    mask = cv2.dilate(mask, cv2.getStructuringElement(cv2.MORPH_DILATE, (3,3) ))

    # Use Mask to get blob information
    moments = cv2.moments(mask)
    area = moments['m00']
    if (area > 0):
      cx = int(moments['m10']/moments['m00']) # cx = M10/M00
      cy = int(moments['m01']/moments['m00']) # cy = M01/M00
      self.postRightPoint(cx,cy) # Publish it

  def postLeftPoint(self, x, y):
    worldPos = self.invCameraMatrix * np.matrix([x,y,1]).T
    point = PointStamped(header=Header(stamp=rospy.Time.now(),
                                       frame_id='/left_camera'),
                         point=Point(x,y,worldPos[2]))
    self.left_point_pub.publish(point)

  def postRightPoint(self, x, y):
    worldPos = self.invCameraMatrix * np.matrix([x,y,1]).T
    point = PointStamped(header=Header(stamp=rospy.Time.now(),
                                       frame_id='/right_camera'),
                         point=Point(x,y,worldPos[2]))
    self.right_point_pub.publish(point)


def main(args):
  ic = image_track()
  rospy.init_node('image_track', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print "Shutting down"
  cv2.destroyAllWindows()
  print "Finished."

if __name__ == '__main__':
    main(sys.argv)