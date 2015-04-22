#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from axis_camera.msg import Axis
import sys, select, termios, tty
import time
print " i^, j<- ,k->, l!, Q to end "
time.sleep(2)

def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def vels(speed,turn):
    return "currently:\tspeed %s\tturn %s " % (speed,turn)

if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)
    
    rospy.init_node('_teleop')
    pub = rospy.Publisher('cmd', Axis, queue_size=5)
    state = Axis()
    state.pan=0
    state.tilt=-45


    pub.publish(state)
    try:
        while(1):
			key = getKey()
			print key
			if key == "i":
				state.tilt+=10
				if state.tilt>-5: 
					state.tilt=-5
				print state
				pub.publish(state)
			elif key == "k":
				state.tilt+=-10
				if state.tilt<-85: 
					state.tilt=-85
				print state
				pub.publish(state)
			elif key == "j":
				state.pan+=-10
				if state.pan<-175: 
					state.tilt=-175
				print state
				pub.publish(state)
			elif key == "l":
				state.pan+=10
				if state.pan>175: 
					state.tilt=175
				print state
				pub.publish(state)
			elif key == "q":
				break
         	
    except:
        print e

    finally:
    	print state
        pub.publish(state)

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)