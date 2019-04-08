#! /usr/bin/env python

import rospy
import time
import math

from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
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

	for i in range(9):
		cov_q[i] = msg[i+4] 
		cov_a[i] = msg[i+16]
		cov_l[i] = msg[i+28]


	imu_msg.header.frame_id = "base_footprint"	
	imu_msg.header.stamp = rospy.Time.now()
	pub = rospy.Publisher("imu_data", Imu, queue_size = 30)
	pub.publish(imu_msg)
	rospy.loginfo("imu published")

class Encoder(object):
	def __init__(self):
		self.start_time = time.time()
		self.x_val = [0]*30
		self.y_val = [0]*30
		self.yaw_val = [0]*30

		self.vx = [0]*30
		self.vy = [0]*30
		self.vt = [0]*30

		self.current_pose = [0,0,0,0,0,0]

		self.counter = 1
		self.max_counter = 30

	def convertToPose(self,vl,vr):
		if vl == "inf":
			vl = 0

		if vr == "inf":
			vr = 0

		next_pose = [0,0,0,0,0,0] #x,y,theta, vx, vy, w
		elapsed_time = time.time() - self.start_time

		vc = 0.5 * (vl*vr)
		wc = 0.5 * (vl-vr)

		current_angle = self.current_pose[2]
		next_angle = current_angle + wc*elapsed_time
	
		if wc != 0:		
			next_pose[0] = self.current_pose[0] + vc/wc*(math.sin(next_angle)-math.sin(current_angle))
			next_pose[1] = self.current_pose[1] - vc/wc*(math.cos(next_angle)-math.cos(current_angle))
			next_pose[2] = next_angle
			next_pose[5] = wc
			next_pose[3] = (next_pose[1] - self.current_pose[1])/elapsed_time
			next_pose[4] = (next_pose[2] - self.current_pose[2])/elapsed_time				

		else:
			next_pose[0] = self.current_pose[0] + vc*elapsed_time*math.cos(current_angle)
			next_pose[1] = self.current_pose[1] + vc*elapsed_time*math.sin(current_angle) 
			next_pose[2] = next_angle
			next_pose[5] = wc
			next_pose[3] = (next_pose[1] - current_pose[1])/elapsed_time
			next_pose[4] = (next_pose[2] - current_pose[2])/elapsed_time
				
	
		self.current_pose = next_pose
	
		i = self.counter -1
		self.x_val[i] = next_pose[0]
		self.y_val[i] = next_pose[1]
		self.yaw_val[i] = next_pose[2]
		self.vx[i] = next_pose[3]
		self.vy[i] = next_pose[4]
		self.vt[i] = next_pose[5]


	def compute_covariance(self,arr1, arr2):
		mean1 = sum(arr1)/self.max_counter
		mean2 = sum(arr2)/self.max_counter

		output = 0
		for i in range(self.max_counter):
			output += (arr1[i] - mean1)*(arr2[i]-mean2)/(self.max_counter-1)

		return output

	def encoderCallback(self,msg):
		msg = msg.data
		self.convertToPose(msg[0], msg[1])
		self.counter += 1
		if self.counter == 31:
			self.counter = 1
			new_msg = Odometry()
			pose_covariance = [-1 for i in range(36)] #x,y,z,r,p,y
			twist_covariance = [-1 for i in range(36)] #vx,vy,vz, wr,wp,wy

			kxy = self.compute_covariance(self.x_val, self.y_val)
			kxt = self.compute_covariance(self.x_val, self.yaw_val)
			kyt = self.compute_covariance(self.y_val, self.yaw_val)
		
			kvxy = self.compute_covariance(self.vx,self.vy)		
			kvxt = self.compute_covariance(self.vx,self.vt)
			kvyt = self.compute_covariance(self.vt,self.vy)

			varx = self.compute_covariance(self.x_val, self.x_val)
			vary = self.compute_covariance(self.y_val, self.y_val)		
			vart = self.compute_covariance(self.yaw_val, self.yaw_val)

			varvx = self.compute_covariance(self.vx,self.vx)
			varvy = self.compute_covariance(self.vy,self.vy)
			varvt = self.compute_covariance(self.vt,self.vt)

			if varvt == -1:
				varvt = 1

			pose_covariance[0] = varx
			pose_covariance[7] = vary
			pose_covariance[35] = vart
			pose_covariance[14] = 1
			pose_covariance[21] = 1
			pose_covariance[28] = 1

			pose_covariance[1] = kxy
			pose_covariance[5] = kxt
			pose_covariance[11] = kyt
	
			pose_covariance[6] = pose_covariance[1]
			pose_covariance[30] = pose_covariance[5]
			pose_covariance[31] = pose_covariance[11]

			twist_covariance[0] = varvx
			twist_covariance[7] = varvy
			twist_covariance[35] = varvt
			twist_covariance[14] = 1
			twist_covariance[21] = 1			
			twist_covariance[28] = 1

			twist_covariance[1] = kvxy
			twist_covariance[5] = kvxt
			twist_covariance[11] = kvyt

			twist_covariance[6] = twist_covariance[1]
			twist_covariance[30] = twist_covariance[5]
			twist_covariance[31] = twist_covariance[11]

			new_msg.pose.pose.position.x = self.x_val[29]
			new_msg.pose.pose.position.y = self.y_val[29]

			q = new_msg.pose.pose.orientation
			q1 = quaternion_from_euler(0,0,self.yaw_val[29])
			q.x, q.y, q.z, q.w  = q1[0], q1[1], q1[2], q1[3]

			new_msg.twist.twist.linear.x = self.vx[29]
			new_msg.twist.twist.linear.y =  self.vy[29]
			new_msg.twist.twist.angular.z =  self.vt[29]

			new_msg.pose.covariance = pose_covariance
			new_msg.twist.covariance = twist_covariance

			new_msg.header.frame_id = "odom_combined"
			new_msg.header.stamp = rospy.Time.now()
			new_msg.child_frame_id = "base_footprint"
			pub = rospy.Publisher("odom", Odometry, queue_size = 30)
			pub.publish(new_msg)
			rospy.loginfo("odom published")


if __name__ == "__main__":
	rospy.init_node("test")
	rospy.Subscriber("readings", Float32MultiArray, callback)
	encode = Encoder()
	rospy.Subscriber("encoded", Float32MultiArray, encode.encoderCallback)
	rospy.spin()
