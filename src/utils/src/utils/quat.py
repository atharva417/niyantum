import tf
from tf.transformations import euler_from_quaternion
import rospy
import rospy
import time
import numpy as np
# 3D point & Stamped Pose msgs
from geometry_msgs.msg import *
# import all mavros messages and services
from mavros_msgs.msg import *
from mavros_msgs.srv import * 
from sensor_msgs.msg import *

def imu_call(imu_val):
	x = imu_val.orientation.x
	y = imu_val.orientation.y
	z = imu_val.orientation.z
	w = imu_val.orientation.w
	e = euler_from_quaternion([x,y,z,w])
	e = np.array(e)
	e = e*28.644
	print(e)


if __name__ == '__main__':
	rospy.init_node('drop_node', anonymous=True)
	rospy.Subscriber("/mavros/imu/data", Imu, imu_call)
	rospy.spin()
	