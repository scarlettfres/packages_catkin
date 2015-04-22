#!/usr/bin/env python
import rospy
#from std_msgs.msg import String
from razer_hydra.msg import Hydra
from std_msgs.msg import Empty

import roslib; 


from geometry_msgs.msg import Twist # for sending commands to the drone




class BasicDroneController:

	def __init__(self):
		self.pilote()

	def callback(self,data):
		Xleft = data.paddles[0].transform.translation.x
		Yleft = data.paddles[0].transform.translation.y
		Zleft = data.paddles[0].transform.translation.z
		
		Xright = data.paddles[1].transform.translation.x
		Yright = data.paddles[1].transform.translation.y
		Zright = data.paddles[1].transform.translation.z
				
		rospy.loginfo(rospy.get_caller_id() + "XLeft :  %s", Xleft)
		rospy.loginfo(rospy.get_caller_id() + "YLeft :  %s", Yleft)
		rospy.loginfo(rospy.get_caller_id() + "ZLeft :  %s", Zleft)
		e=Empty
		rospy.loginfo(rospy.get_caller_id() + "XRight :  %s", Xright)
		rospy.loginfo(rospy.get_caller_id() + "YRight :  %s", Yright)
		rospy.loginfo(rospy.get_caller_id() + "ZRight :  %s", Zright)
		
		data.paddles[0].buttons 
					
	def pilote(self):

		# In ROS, nodes are uniquely named. If two nodes with the same
		# node are launched, the previous one is kicked off. The
		# anonymous=True flag means that rospy will choose a unique
		# name for our 'listener' node so that multiple listeners can
		# run simultaneously.
		rospy.init_node('pilote', anonymous=True)
		self.reset = rospy.Publisher('/ardrone/reset',Empty, queue_size=10)
		rospy.Subscriber("/hydra_calib", Hydra, self.callback)

		# spin() simply keeps python from exiting until this node is stopped
		rospy.spin()

if __name__ == '__main__':
    BasicDroneController()

