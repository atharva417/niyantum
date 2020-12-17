#!/usr/bin/env python
# ROS python API
import rospy
import time
import numpy as np
# 3D point & Stamped Pose msgs
from geometry_msgs.msg import *
# import all mavros messages and services
from mavros_msgs.msg import *
from mavros_msgs.srv import * 
from sensor_msgs.msg import *
import math



def setarm(x): # input: 1=arm, 0=disarm
	rospy.wait_for_service('/mavros/cmd/arming')
	try:
		arming = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
		response = arming(x)
		response.success
	except rospy.ServiceException, e:
		print "Service call failed: %s"%e

def setmode(x):
	rospy.wait_for_service('/mavros/set_mode')
	try:
		mode = rospy.ServiceProxy('/mavros/set_mode', SetMode)
		response = mode(0,x)
		response.mode_sent
	except rospy.ServiceException, e:
		print "Service call failed: %s"%e

def imu_call(imu_val):
	global z2
	z2 = imu_val.linear_acceleration.z
	#print(z2)

# def man_pub(man_msg):
#     rate = rospy.Rate(10) # 10hz
#     while not rospy.is_shutdown():
#         pub.publish(man_msg)
#         rate.sleep()

# def man_send(x,y,z,r):
# 	man_msg.x = x
# 	man_msg.y = y
# 	man_msg.z = z
# 	man_msg.r = r
# 	man_pub(man_msg)

def lid_call(lid_val):
	global z
	z = lid_val.ranges[0]

def alt_control():
	print('In alt_control')
	rate = rospy.Rate(20)
	#P = 2.0
	#D = 1.0
	I = 0.0
	ez_n = 5.0
	ezi = 0.0
	z_des = 5.0
	throttle = 710.0
	while not rospy.is_shutdown():
		ez_o = ez_n
		ez_n = z_des - z
		if(abs(ez_n) < 0.5):
			P = 0.5
			D = 0.2
		else:
			P = 4
			D = 0

		dt = 0.1
		ezd = (ez_n - ez_o)/dt
		ezi += (ez_n + ez_o)*dt/2

		t_pid = (P*ez_n) + (D*ezd) + (I*ezi)
		throttle = 710.0 + t_pid
		#throttle = np.sqrt((720.0**2) + t_pid)
		if(throttle<0.0):
			throttle = 0.0
		elif(throttle>1000.0):
			throttle = 1000.0

		man_msg = ManualControl()
		man_msg.x = 0.0
		man_msg.y = 0.0
		man_msg.z = throttle
		man_msg.r = 0.0
		#print(ez_n)
		#print(t_pid)

		print(throttle, ez_n, t_pid)

		pub.publish(man_msg)
        rate.sleep()


if __name__ == '__main__':
	rospy.init_node('drop_node', anonymous=True)
	print("drop_node initialised...")
	pub = rospy.Publisher('mavros/manual_control/send', ManualControl, queue_size=100)
	rospy.Subscriber("/mavros/imu/data", Imu, imu_call)
	rospy.Subscriber("/lidar/data", LaserScan, lid_call)
	
	setarm(1)
	
	setmode('MANUAL')
	time.sleep(3)
	alt_control()
	# while True:
	# 	if(0<z2<0.2):
	# 		print('In if loop')
	# 		setmode('MANUAL')
	# 		alt_control()
	# 		print('reached end')
