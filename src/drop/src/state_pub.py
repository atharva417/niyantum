#!/usr/bin/env python
import rospy
import time
from mavros_msgs.msg import *
from gazebo_msgs.msg import ModelState

rospy.init_node('pub_once', anonymous=True)

pub = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=5)

time.sleep(3)

data = ModelState()
data.model_name = 'iris'

data.pose.position.x = 0.0
data.pose.position.y = 0.0
data.pose.position.z = 20.0

data.pose.orientation.x = 0.0
data.pose.orientation.y = 0.246
data.pose.orientation.z = 0.0
data.pose.orientation.w = 0.966

data.twist.linear.x = 0.0
data.twist.linear.y = 0.0
data.twist.linear.z = 0.0

data.twist.angular.x = 0.0 
data.twist.angular.y = 0.0
data.twist.angular.z = 0.0

data.reference_frame = 'world'

# for i in range(5):
pub.publish(data)
print('done')


#rostopic pub -r 20 /gazebo/set_model_state gazebo_msgs/ModelState '{model_name: iris, pose: { position: { x: 0, y: 0, z: 10 }, orientation: {x: 0, y: 0.491983115673, z: 0, w: 0.870604813099 } }, twist: { linear: { x: 0, y: 0, z: 0 }, angular: { x: 0, y: 0, z: 0}  }, reference_frame: world }'
