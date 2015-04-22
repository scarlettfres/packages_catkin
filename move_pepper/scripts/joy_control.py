#!/usr/bin/env python
import roslib
import rospy
import naoqi
import time


from razer_hydra.msg import Hydra
from std_msgs.msg import Empty
from std_msgs.msg import String

from geometry_msgs.msg import Twist # for sending commands to the drone
from naoqi import ALProxy


class NaoRazer:
	

	def __init__(self):
		self.FLAG=False
		#pour homme 
		self.initLeftX=0
		self.initLeftY=0
		self.initLeftZ=0
		self.initRightX=0
		self.initRightY=0
		self.initRightZ=0
		#pour Nao
		self.XleftNao=0
		self.YleftNao=0
		self.ZleftNao=0
		
		self.r1leftNao = 0
		self.r2leftNao= 0
		self.r3leftNao=0
		
		self.XrightNao=0
		self.YrightNao=0
		self.ZrightNao=0

	
		self.init()

	def callback(self,data):
	
		Xleft = data.paddles[0].joy[0]
		Yleft = data.paddles[0].joy[1]
		
		Xright = data.paddles[1].joy[0]
		Yright = data.paddles[1].joy[1]

		self.move(Xleft,Yleft,Zleft,Xright,Yright,Zright)
	
	def init(self):
		rospy.init_node('mon_node', anonymous=True)
		self.motionProxy = ALProxy("ALMotion","127.0.0.1",9559)
		self.postureProxy = ALProxy("ALRobotPosture", "127.0.0.1",9559)
		self.motionProxy.setStiffnesses("LArm", 1.0)
		self.motionProxy.setStiffnesses("RArm", 1.0)
			
   		self.motionProxy.wbEnableEffectorControl("LArm", False)
   		
		rospy.Subscriber("/hydra_calib", Hydra, self.callback)
			
		rospy.spin()
		
	


	def move(self,x,y,z,xr,yr,zr):
		
		
		
		#self.motionProxy.wbEnable(True)	# active whole body control 
		
		PositionLeft = self.motionProxy.getPosition('RArm',0, True)
		print ("x ",PositionLeft[0])
		print ("y ",PositionLeft[1])
		print ("z",PositionLeft[2])
		
		
		vectl= [x/3.+self.XleftNao,y/3.+self.YleftNao,z/3.+self.ZleftNao,self.r1leftNao,self.r2leftNao,self.r3leftNao]
		vectr= [xr+self.XrightNao,yr+self.YrightNao,zr+self.ZrightNao,self.r1rightNao,self.r2rightNao,self.r3rightNao]
		
		
		
		ecartx = PositionLeft[0]-(x/3+self.XleftNao)
		ecarty = PositionLeft[1]-(y/3+self.YleftNao)
		ecartz = PositionLeft[2]-(z/3+self.ZleftNao)
		print "ecartx =", ecartx
		print "ecarty =", ecarty
		print "ecartz =", ecartz
		#self.motionProxy.wbSetEffectorControl("LArm", vectl)
		#self.motionProxy.wbSetEffectorControl("RArm", vectr)
		
			
		#self.motionProxy.setPosition ("LArm", 0, vectl, 0.5, 7)
		
		#self.motionProxy.setPosition ("RArm", 0, [0.,0.,1,0.,0.,0.], 0.5, 7)
	
	


if __name__ == '__main__':
    NaoRazer()
