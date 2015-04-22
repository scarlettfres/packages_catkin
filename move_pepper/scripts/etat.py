#!/usr/bin/env python
import rospy
from std_msgs.msg import String

import naoqi
from naoqi import ALProxy
from naoqi import motion
def etat():


	rospy.init_node('etat_node', anonymous=True)
	
	motionProxy = ALProxy("ALMotion","127.0.0.1",9559)

	while 1:
		initialTransform = motionProxy.getPosition('LArm',0, True)

		print(initialTransform)
		print("=================================")

	


	# spin() simply keeps python from exiting until this node is stopped
	#rospy.spin()

if __name__ == '__main__':
    etat()
