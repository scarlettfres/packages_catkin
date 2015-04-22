#!/usr/bin/env python
import roslib
roslib.load_manifest('stereo_color_tracker')
import sys
import rospy
import cv2
import cv
import time
import numpy as np
from std_msgs.msg import String, Header
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped, Point
from cv_bridge import CvBridge, CvBridgeError
from pylab import * 
# import IPython


class disparity_track:

  def __init__(self):
    # self.tracked_point_pub = rospy.Publisher("tracked_point", PointStamped, queue_size=5)
    self.left_point_pub = rospy.Publisher("left_point", PointStamped, queue_size=5)

    # cv2.namedWindow("Image window", 1)
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/left_cam/image_raw",Image,self.left_callback)
    self.image_sub = rospy.Subscriber("/right_cam/image_raw",Image,self.right_callback)
    self.X_calib=0
    self.Y_calib=0
    self.Z_calib=-2./1741.69166872
    self.FLAG=0

    #===================Parser une matric opencv xml en python ! ========
    try :
      self.Q = np.asarray(cv2.cv.Load("src/StereoColorTracking-master/camera_info/Q.xml"))
      self.mx1 = np.asarray(cv2.cv.Load("src/StereoColorTracking-master/camera_info/mx1.xml"))
      self.my1 = np.asarray(cv2.cv.Load("src/StereoColorTracking-master/camera_info/my1.xml"))
      self.mx2 = np.asarray(cv2.cv.Load("src/StereoColorTracking-master/camera_info/mx2.xml"))
      self.my2 = np.asarray(cv2.cv.Load("src/StereoColorTracking-master/camera_info/my2.xml"))

     

    except Exception, e:
      print "can't read calibration files !! they should be in /src/StereoColorTracking-master/camera_info/ : ",e  
      sys.exit()
    #===================Parser une matric opencv xml en python ! ========

    print "aaaa"

    self.lower_threshold = np.array((118, 41, 0))
    self.upper_threshold = np.array((140, 255, 255))


    self.disparity_ratio = 163.1 # pour environ 21cm entre les 2 cam ( dx=47.5cm pour dx=338.35)
    # 1280 x 960 image
    # self.center_x = (1280.0/2.0) # half x pixels
    # self.center_y = (960.0/2.0) # half y pixels
    # 640 x 480
    self.center_x = (640.0/2.0) # half x pixels
    self.center_y = (480.0/2.0) # half y pixels
   
    #matrice qui correspont a la matrice de calibration 

    self.Kinv = np.matrix([[879.3029856737741, 0, 296.5510457981297], [0, 885.3267823276828, 213.0903768923715], [0, 0, 1]]).I # K inverse

    self.last_left_image_pos = np.array([0.0, 0.0])  



  def left_callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError, e:
      print e

    self.left_image = cv2.remap(cv_image,self.mx1,self.my1,cv2.INTER_LINEAR)
    


  def right_callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError, e:
      print e
    # Get HSV image

    self.right_image = cv2.remap(cv_image,self.mx2,self.my2,cv2.INTER_LINEAR)

    hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
    print "ThresholdAAAIALE" ,len(hsv)
    print "ThresholdAAAIALE" ,len(hsv[0])
  
    print "cx",self.last_left_image_pos[0] 
    print"cy", self.last_left_image_pos[1]
    #img = img[c1:c1+25,r1:r1+25]

    roi = hsv[self.last_left_image_pos[1]-1:self.last_left_image_pos[1]+1, 0:len(hsv)-1]
    # Threshold image to range

    #self.last_left_image_pos[0] = cx
    #self.last_left_image_pos[1] = cy
    mask = cv2.inRange(roi, self.lower_threshold, self.upper_threshold)

    mask = cv2.erode(mask, cv2.getStructuringElement(cv2.MORPH_ERODE, (3,3) ))
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(3,3))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    closing = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    moments = cv2.moments(mask)
    area = moments['m00']
    if (area > 0):
      cx = moments['m10']/moments['m00'] # cx = M10/M00
      cy = moments['m01']/moments['m00'] # cy = M01/M00
      cv2.circle(cv_image, (int(cx),int(self.last_left_image_pos[1])), 10, (0,255,255),-1)
      dx = float(cx - self.last_left_image_pos[0])

      vect=[[self.last_left_image_pos[0]],[self.last_left_image_pos[1]],[dx],[1]]
      result = dot(self.Q, vect)
      
      print "X=", result[0]/result[3]," Y= ",result[1]/result[3]," Z= ", result[2]/result[3]

      depth = self.disparity_ratio/dx
      
      f=self.Q[2][3]
      T=1/self.Q[3][2]
      depth = float(f*T/dx)
     
      # print "dx: %3.2f, dy: %3.2f, guessed depth: %3.2f" % ((self.last_left_image_pos[0] - cx), (self.last_left_image_pos[1] - cy), depth)
      #self.postLeftPoint(self.last_left_image_pos[0],self.last_left_image_pos[1],depth)
      self.postLeftPoint(result[0]/result[3],result[1]/result[3],result[2]/result[3])
    else:
      print "RIGH DETECTE PAS"
    cv2.imshow("image post calib right ", cv_image)


  def postLeftPoint(self, x, y, z):
    #worldPos = self.Kinv * np.matrix([x,y,1]).T * depth # cette formule permet de trouver x et y 
     
  
    point = PointStamped(header=Header(stamp=rospy.Time.now(),
                                       frame_id='/left_camera'),
                         point=Point(x,y,z*self.Z_calib))
    print "point=", point
    self.left_point_pub.publish(point)


def main(args):
  rospy.init_node('Poin_Cloud', anonymous=True)

  ic = disparity_track()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print "Shutting down"
  cv2.destroyAllWindows()
  print "Finished."

if __name__ == '__main__':
    main(sys.argv)