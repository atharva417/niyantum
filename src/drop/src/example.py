#!/usr/bin/env python
from utils.offboard import mavcon
import time
import rospy
rospy.init_node('offboard_node', anonymous=True)
mvc = mavcon()
mvc.setarm(1)
time.sleep(2)
mvc.setmode('OFFBOARD')
mvc.gotopose(0,0,8)
mvc.gotopose(5,5,5)
