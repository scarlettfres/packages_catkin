#!/usr/bin/env python
import roslib
import rospy
import naoqi
import time

import numpy as np
from razer_hydra.msg import Hydra
from std_msgs.msg import Empty
from std_msgs.msg import String

from geometry_msgs.msg import Twist # for sending commands to the drone
from naoqi import ALProxy

FINESSE=0.005
#IP= "127.0.0.1"
IP="192.168.0.102"
PORT=9559
class NaoRazer:

	def __init__(self):
		
		self.init()


	def init(self):
		rospy.init_node('stop', anonymous=True)
		motionProxy = ALProxy("ALMotion",IP,PORT)
		postureProxy = ALProxy("ALRobotPosture", IP,PORT)
		
		motionProxy.setStiffnesses("Head", 0.0)
		motionProxy.setStiffnesses("LArm", 0.0)
		motionProxy.setStiffnesses("RArm", 0.0)

if __name__ == '__main__':
    NaoRazer()
