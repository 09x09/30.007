#! /usr/bin/env python

import rospy

from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Imu

def callback(msg):
	msg = msg.data
	imu_msg = Imu()
	q  = imu_msg.orientation
	q.x, q.y, q.z, q.w = msg[0], msg[1], msg[2], msg[3]

	cov_q = imu_msg.orientation_covariance

	av = imu_msg.angular_velocity
	av.x, av.y, av.z = msg[13], msg[14], msg[15]
	
	cov_a = imu_msg.angular_velocity_covariance

	lv = imu_msg.linear_acceleration
	lv.x, lv.y, lv.z = msg[25], msg[26], msg[27]

	cov_l = imu_msg.linear_acceleration_covariance

	for i in range(8):
		cov_q[i] = msg[i+4] 
		cov_a[i] = msg[i+16]
		cov_l[i] = msg[i+28]

	pub = rospy.Publisher("imu_data", Imu, queue_size = 30)
	pub.publish(imu_msg)
	rospy.loginfo("published")



if __name__ == "__main__":
	rospy.init_node("test")
	rospy.Subscriber("readings", Float32MultiArray, callback)
	
	rospy.spin()
