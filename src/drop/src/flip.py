#!/usr/bin/env python
from utils.offboard import mavcon
import time
import rospy
rospy.init_node('offboard_node', anonymous=True)
mvc = mavcon()
mvc.setarm(1)
time.sleep(2)
mvc.offboard()
mvc.gotopose(0,0,20)
time.sleep(2)
mvc.flip()
mvc.gotopose(0,0,10)