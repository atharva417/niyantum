#!/usr/bin/env python
import rospy
import time
import numpy as np
# 3D point & Stamped Pose msgs
from geometry_msgs.msg import Point, PoseStamped
# import all mavros messages and services
#from mavros_msgs.msg import *
from mavros_msgs.srv import SetMode, CommandBool
class mavcon:
	def __init__(self):
		rospy.Subscriber('mavros/local_position/pose', PoseStamped, self.loc_pose)
		self.pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=1)
		self.pt = Point()

	def setmode(self,md):
		rospy.wait_for_service('/mavros/set_mode')
		try:
			mode = rospy.ServiceProxy('/mavros/set_mode', SetMode)
			response = mode(0,md)
			response.mode_sent
		except rospy.ServiceException, e:
			print "Service call failed: %s"%e

	def setarm(self,av): # input: 1=arm, 0=disarm
		rospy.wait_for_service('/mavros/cmd/arming')
		try:
			arming = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
			response = arming(av)
			response.success
		except rospy.ServiceException, e:
			print "Service call failed: %s"%e

	def gotopose(self,x,y,z):
		rate = rospy.Rate(20)
		sp = PoseStamped()
		sp.pose.position.x = x
		sp.pose.position.y = y
		sp.pose.position.z = z
		dist = np.sqrt(((self.pt.x-x)**2) + ((self.pt.y-y)**2) + ((self.pt.z-z)**2))
		while(dist > 0.08):
			self.pub.publish(sp)
			dist = np.sqrt(((self.pt.x-x)**2) + ((self.pt.y-y)**2) + ((self.pt.z-z)**2))
			rate.sleep()
		print('Reached ',x,y,z)

	def offboard(self):
		rate = rospy.Rate(10)
		sp = PoseStamped()
		sp.pose.position.x = 0.0
		sp.pose.position.y = 0.0
		sp.pose.position.z = 0.0
		for i in range(10):
			self.pub.publish(sp)
			rate.sleep()
		print('We are good to go!!')
		self.setmode("OFFBOARD")


	def loc_pose(self,data):
		self.pt.x = data.pose.position.x
		self.pt.y = data.pose.position.y
		self.pt.z = data.pose.position.z

# if __name__ == '__main__':
# 	rospy.init_node('offboard_node', anonymous=True)
# 	rospy.Subscriber('mavros/local_position/pose', PoseStamped, loc_pose)
# 	pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=1)
# 	setarm(1)
# 	time.sleep(2)
# 	setmode('OFFBOARD')
# 	gotopose(3,3,3)
# 	gotopose(-5,-5,5)