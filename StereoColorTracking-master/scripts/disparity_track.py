#!/usr/bin/env python
import roslib
roslib.load_manifest('stereo_color_tracker')
import sys
import rospy
import cv2
import time
import numpy as np
import math
from std_msgs.msg import String, Header, Float32
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped, Point 
from cv_bridge import CvBridge, CvBridgeError
from pylab import * 
# import IPython


class disparity_track:

  def __init__(self):
    # self.tracked_point_pub = rospy.Publisher("tracked_point", PointStamped, queue_size=5)
    self.left_point_pub = rospy.Publisher("left_point", PointStamped, queue_size=5)
    self.disp_pub = rospy.Publisher("disparity", Float32, queue_size=5)

    # cv2.namedWindow("Image window", 1)
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/left_cam/image_raw",Image,self.left_callback)
    self.image_sub = rospy.Subscriber("/right_cam/image_raw",Image,self.right_callback)
    self.line_to_keep=[[[0, 0, 0, 0],[0, 0, 0, 0]]]
    #=============== ICIIIIIIIIIIIIIIIII <====
    self.ratio=105.32390708
    self.last_left_image_pos = np.array([0.0, 0.0])  

    self.defect_stable = array([[[0., 0., 0.,0. ]]])
    self.poids_droites = 0 # ???
    self.it = 0
    self.vect_point_max_x=[]
    self.vect_point_min_x=[]
    self.vect_point_max_y=[]
    self.vect_point_min_y=[]

    self.fin_point_max_x = (888,888)
    self.fin_point_max_y = (0,0)
    self.fin_point_min_x = (0,0)
    self.fin_point_min_y = (0,0)

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
    self.lower_threshold = np.array((118, 41, 0))
    self.upper_threshold = np.array((140, 255, 255))

    """cv2.namedWindow("Control", cv2.CV_WINDOW_AUTOSIZE); # Threshold Controller window
    cv2.createTrackbar("LowH", "Control", self.lower_threshold[0], 255, self.updateLowH); # Hue (0 - 255)
    cv2.createTrackbar("HighH", "Control", self.upper_threshold[0], 255, self.updateHighH);
    cv2.createTrackbar("LowS", "Control", self.lower_threshold[1], 255, self.updateLowS); # Saturation (0 - 255)
    cv2.createTrackbar("HighS", "Control", self.upper_threshold[1], 255, self.updateHighS);
    cv2.createTrackbar("LowV", "Control", self.lower_threshold[2], 255, self.updateLowV); # Value (0 - 255)
    cv2.createTrackbar("HighV", "Control", self.upper_threshold[2], 255, self.updateHighV);
    """
    

  def updateLowH(self, value):
    self.lower_threshold[0] = value
   # print "self.lower_threshold[0]=",value
  def updateHighH(self, value):
    self.upper_threshold[0] = value
    #print "self.upper_threshold[0] = ",value
  def updateLowS(self, value):
    self.lower_threshold[1] = value
    #print "self.lower_threshold[1] =", value
  def updateHighS(self, value):
    self.upper_threshold[1] = value
   # print"self.upper_threshold[1] = ",value
  def updateLowV(self, value):
    self.lower_threshold[2] = value
   # print "self.lower_threshold[2] = ", value
  def updateHighV(self, value):
    self.upper_threshold[2] = value
    #print " self.upper_threshold[2] = ",value
  


  def left_callback(self,data):
    try:
      get = self.bridge.imgmsg_to_cv2(data, "bgr8")
      cv_image = get.copy()
    except CvBridgeError, e:
      print e
    
    cv_image = cv2.remap(cv_image,self.mx1,self.my1,cv2.INTER_LINEAR)
    cv_image = cv2.blur(cv_image,(3,3))
    # Get HSV image
    hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

    # Threshold image to range
    mask = cv2.inRange(hsv, self.lower_threshold, self.upper_threshold)
   
    # Erode/Dilate mask to remove noise

    mask = cv2.erode(mask, cv2.getStructuringElement(cv2.MORPH_ERODE, (3,3) ))
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(3,3))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    # Use Mask to get blob information

    moments = cv2.moments(mask)
    area = moments['m00']
    if (area > 0):
      cx = moments['m10']/moments['m00'] # cx = M10/M00
      cy = moments['m01']/moments['m00'] # cy = M01/M00

      # on cherche l'extremite gauche de notre object 
      flag = 0
      for h in range (0,640):
        if mask[cy][h]!=0 and flag ==0 :
            cx=h
            flag=1
          

      cv2.circle(cv_image, (int(cx),int(cy)), 10, (0,255,255),-1)
      self.last_left_image_pos[0] = cx
      self.last_left_image_pos[1] = cy
    else:
      print "LEFT DETECT PAS "
      # self.postLeftPoint(cx,cy) # Publish it
    #cv2.imshow("mask left", mask)
    #cv2.imshow("image post calib left ", cv_image)
    k = cv2.waitKey(3) & 0xFF
    if k == 113 or k == 27: # Escape key = 27, 'q' = 113
      rospy.signal_shutdown("User Exit")


  def right_callback(self,data):
    try:
      get = self.bridge.imgmsg_to_cv2(data, "bgr8")
      cv_image = get.copy()
    except CvBridgeError, e:
      print e
    # Get HSV image

    cv_image = cv2.remap(cv_image,self.mx2,self.my2,cv2.INTER_LINEAR)
    cv_image = cv2.blur(cv_image,(3,3))

    hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, self.lower_threshold, self.upper_threshold)
    mask = cv2.erode(mask, cv2.getStructuringElement(cv2.MORPH_ERODE, (3,3) ))
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(3,3))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    im=mask.copy()
    moments = cv2.moments(mask)
    area = moments['m00']
    if (area > 0):
      cx = moments['m10']/moments['m00'] # cx = M10/M00
      cy = moments['m01']/moments['m00'] # cy = M01/M00

      flag = 0
      for h in range (0,640):
        if mask[self.last_left_image_pos[1]][h]!=0 and flag ==0 :

            cx=h
            flag=1

      cnt = self.find_orientation(im)
      
      #cv2.drawContours(cv_image,contours,0,(0,0,255),2)
      
      """
      for i in range(defects.shape[0]):
        s,e,f,d = defects[i,0]
        start = tuple(cnt[s][0])
        end = tuple(cnt[e][0])
        far = tuple(cnt[f][0])
        cv2.line(cv_image,start,end,[0,255,0],2)
       """


      """
      cv2.line(cv_image, (258,32), (337,32),(0,0,255),5)
      cv2.line(cv_image, (254, 50), (258,142),(0,0,255),5)
      cv2.line(cv_image, (338,86), (347,170),(0,0,255),5)
      """
      


      #cv2.line(cv_image, (120,236), (25,56) ,255)


      #cv2.line(cv_image,(line[0],line[1]),(line[2],line[3]),255,2)
      cv2.circle(cv_image, (int(cx),int(self.last_left_image_pos[1])), 10, (0,255,255),-1)
     # print cx 
     # print self.last_left_image_pos[0]
      dx = float(cx - self.last_left_image_pos[0])
     # print "dx= ", dx
      W =  abs(dx * self.Q[3][2]) + self.Q[3][3]
   

      X = (self.last_left_image_pos[0]+ self.Q[0][3])/(W*self.ratio)
      Y = (self.last_left_image_pos[1]+ self.Q[1][3])/(W*self.ratio)
      Z =  self.Q[2][3]/(W*self.ratio)
     
      self.disp_pub.publish(dx)
      self.postLeftPoint(X,Y,Z)
   
    else:
      print "RIGH DETECTE PAS"
    #cv2.imshow("image post calib right ", cv_image)
    
    #cv2.imshow("ieeeeeeeeeet ", mask)

  def postLeftPoint(self, x, y, z):
    
    point = PointStamped(header=Header(stamp=rospy.Time.now(),
                                       frame_id='/left_camera'),
                         point=Point(x,y,z))
   
    self.left_point_pub.publish(point)





  def find_orientation(self, data):


    self.it+=1
    contours,hierarchy = cv2.findContours(data, 1, 2)
    cnt = contours[0]
    # copy or Canny will modify our image
    data_mod=data.copy()
    #TODO
    epsilon = 0.1*cv2.arcLength(cnt,True)
    approx = cv2.approxPolyDP(cnt,epsilon,False)

    # find contours in the threshold image
    contours,hierarchy = cv2.findContours(data,cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)
    contours=array(contours)
    hull = cv2.convexHull(contours[0],returnPoints = False)
    defects = cv2.convexityDefects(contours[0],hull)

    edges = cv2.Canny(data_mod,0,255)
    # give 2 extreme points of all the line created by the points ( around 3 and 5 lines ) TODO 

    #minpix=15 # minimum number of points that can form a line
    #lines = cv2.HoughLinesP(edges, 1, math.pi/180.0, 1,None,10,3)
    #lines = cv2.HoughLines(edges, 1, math.pi/180.0, 40)
    #lines=cv2.HoughLinesP(edges, rho = 1, theta = math.pi / 180, threshold = 70, minLineLength = 15, maxLineGap = 10)
    #print "edge[0]",edges[0][0]
    # just keep the left corner 
    
    blank_image = np.zeros((640,480,3), np.uint8)
    if defects != None:
      vect_point=[]
      for i in range(defects.shape[0]):
        
        s,e,f,d = defects[i,0]
        start = tuple(cnt[s][0])
        end = tuple(cnt[e][0])
        far = tuple(cnt[f][0])
        vect_point.append(start)
        vect_point.append(end)
        
        cv2.line(blank_image,start,end,[0,255,0],2)
      
      point_max_x, point_min_x, point_max_y, point_min_y = self.max_min(vect_point) # ici on recupere les max et min  
      self.vect_point_max_x.append(point_max_x)
      self.vect_point_min_x.append(point_min_x)
      self.vect_point_max_y.append(point_max_y)
      self.vect_point_min_y.append(point_min_y)

      if self.it == 10 :
        print "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaa"
        self.fin_point_max_x, _, _, _ = self.max_min(self.vect_point_max_x)
        _,self.fin_point_min_x, _, _ = self.max_min(self.vect_point_min_x)
        _, _, self.fin_point_max_y, _ = self.max_min(self.vect_point_max_y)
        _, _, _,self.point_min_y = self.max_min(self.vect_point_min_y)
        self.it=0
        self.vect_point_max_x=[]
        self.vect_point_min_x=[]
        self.vect_point_max_y=[]
        self.vect_point_min_y=[]



      cv2.circle(blank_image, self.fin_point_max_x, 2, (0,0,255),2)
      cv2.circle(blank_image, self.fin_point_max_y, 2, (0,0,255),2)
      cv2.circle(blank_image, self.fin_point_min_x, 2, (0,0,255),2)
      cv2.circle(blank_image, self.fin_point_min_y, 2, (0,0,255),2)
      
    #abs_top = [defects[0], j in enumerate(defects[0][0]) if j == top]
    
    dist=3
   

    #self.filtre(defects[0])
    #print "aa", defects[0]
    cv2.imshow("imageddddddddddddt ", blank_image)
    
    return contours[0]
    #return defects, contours[0],lines



  def max_min(self,vect_point): # si vect trouve environ egal au current 

    self.it += 1
    max_x = max([vect_point[l][0] for l in range (0,len(vect_point))])
    min_x = min([vect_point[l][0] for l in range (0,len(vect_point))])
    max_y = max([vect_point[l][1] for l in range (0,len(vect_point))])
    min_y = min([vect_point[l][1] for l in range (0,len(vect_point))])

    
    for l in range (0,len(vect_point)):
      if vect_point[l][0] == max_x:
        point_max_x=(vect_point[l][0],vect_point[l][1])
      if vect_point[l][0] == min_x:
        point_min_x=(vect_point[l][0],vect_point[l][1])
      if vect_point[l][1] == max_y:
        point_max_y=(vect_point[l][0],vect_point[l][1])
      if vect_point[l][1] == min_y:
        point_min_y=(vect_point[l][0],vect_point[l][1])
    
    return point_max_x, point_min_x, point_max_y, point_min_y

      


def main(args):
  rospy.init_node('disparity_track', anonymous=True)

  ic = disparity_track()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print "Shutting down"
  cv2.destroyAllWindows()
  print "Finished."

if __name__ == '__main__':
    main(sys.argv)