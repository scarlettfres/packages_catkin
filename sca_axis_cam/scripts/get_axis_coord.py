#!/usr/bin/env python
import roslib
roslib.load_manifest('detection_multi_cam')
import sys
import rospy
import time
import numpy as np
from std_msgs.msg import String, Header, Float32
import tf
from axis_camera.msg import Axis

class get_axis_coord:

	def __init__(self):

		rospy.Subscriber("/axis/state", Axis,self.state_callback)
		#rospy.Subscriber("/axis_twist/parameter_updates", Axis,self.parameter_callback)
		print "init"

	def state_callback(self,data):
		print "aaaaaaaaaaaaaa"
		print data

	def parameter_callback(self,data):
		print data



			
def main(args):

	rospy.init_node('get_axis_coord', anonymous=True)
	noeud = get_axis_coord()
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print "Shutting down"
		mon_fichier.close()
		print "Finished."

if __name__ == '__main__':
    main(sys.argv)





