#!/usr/bin/env python
from utils.offboard import mavcon
import time
import rospy
rospy.init_node('offboard_node', anonymous=True)
mvc = mavcon()
mvc.setarm(1)
time.sleep(2)
mvc.offboard()
while(not rospy.is_shutdown()):
	x,y,z = input('Where do u wanna go baby? ')
	mvc.setmode('OFFBOARD')
	mvc.gotopose(x,y,z)