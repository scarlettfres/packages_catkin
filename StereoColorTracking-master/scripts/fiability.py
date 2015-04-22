#!/usr/bin/env python
import roslib
roslib.load_manifest('stereo_color_tracker')
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String, Header, Float32
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped, Point

# !!!!! On publie nos coordonnees avec :
# rostopic pub valeur_point geometry_msgs/PointStamped '[0, now, /left_camera]' '[1.0, 2.0, 3.0]' -1
# le -1 signifie juste une fois 




mon_fichier = open("fichier.txt", "w")



def write_point(point):
	text= str(point.point.x) + " "+ str(point.point.y)+" "+ str(point.point.z)
	return text


class result_point:

	def __init__(self):
  		rospy.Subscriber("/left_point", PointStamped,self.point_estimation_callback)
  		#rospy.Subscriber("/valeur_point", PointStamped,self.point_reel_callback)
  		rospy.Subscriber("/disparity",Float32, self.disparity_callback)
  		self.compteur=0
  		self.reel_enter=0
#self.image_sub = rospy.Subscriber("/left_cam/image_raw",Image,self.left_callback)
		self.point_estimation=point = PointStamped(header=Header(stamp=rospy.Time.now(),
                                       frame_id='/left_camera'),
                         point=Point(0.0,0.0, 0.0))
		self.point_reel=point = PointStamped(header=Header(stamp=rospy.Time.now(),
                                       frame_id='/left_camera'),
                         point=Point(0.0,0.0, 0.0))
                
                self.disparity=0

	def point_estimation_callback(self,data):
		self.point_estimation = data 
	def disparity_callback(self,data):
		self.disparity = data 
		self.compteur += 1
		if self.compteur == 25:
			print "11111111111111111"
		if self.compteur == 100:
			print "2222222222222222"
		if self.compteur == 200:
			print "3333333333333333"
			print "======================================>>>>>>>>>>>>>>>>>>>>"
			self.compteur = 0
			reel=str(self.reel_enter)
			ecrire=reel+" "+ write_point(self.point_estimation) + " " +str(self.disparity)
			print ecrire
			mon_fichier.write(ecrire)
			mon_fichier.write("\n")
			self.reel_enter+=0.1


		
	"""def point_reel_callback(self,data):
		print "entry received: ",data.point, "m"
		self.point_reel=data 
		ecrire=write_point(self.point_reel)+" "+ write_point(self.point_estimation) + " " +str(self.disparity)
		mon_fichier.write(ecrire)
		mon_fichier.write("\n")
		
	"""
		



def main(args):

	rospy.init_node('fiability_tracker', anonymous=True)
	noeud = result_point()
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print "Shutting down"
		mon_fichier.close()
		print "Finished."

if __name__ == '__main__':
    main(sys.argv)





