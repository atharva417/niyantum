#!/usr/bin/env python
from utils.offboard import mavcon
import time
import rospy
rospy.init_node('offboard_node', anonymous=True)
mvc = mavcon()
mvc.setarm(1)
time.sleep(2)
mvc.offboard()
mvc.gotopose(0.0,0.0,3.275)
mvc.gotopose(3.0,0.0,3.275)
mvc.gotopose(4.0,0.0,3.275)
mvc.gotopose(5.0,0.0,3.275)
mvc.gotopose(6.0,0.0,3.275)
mvc.gotopose(6.5,0.0,3.275)
mvc.gotopose(6.5,1.0,3.275)
mvc.gotopose(7.0,1.0,3.275)
mvc.gotopose(7.5,0.0,3.275)
mvc.gotopose(7.5,-1.0,3.275)
mvc.gotopose(7.0,-1.0,3.275)
mvc.gotopose(8.5,-1.0,3.275)
mvc.gotopose(8.5,0.0,3.275)
mvc.gotopose(8.5,1.0,3.275)