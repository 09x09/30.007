#! /usr/bin/env python

import rospy
import time
import math

from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from std_msgs.msg import Float32MultiArray


class Encoder(object):
	def __init__(self):
		self.start_time = rospy.get_time()
		self.x_val = [0]*30
		self.y_val = [0]*30
		self.yaw_val = [0]*30

		self.vx = [0]*30
		self.vy = [0]*30
		self.vt = [0]*30

		self.current_pose = [0,0,0,0,0,0]

		self.total_time = 0
		self.counter = 0
			
	def convertToPose(self,vl,vr):
		if vl == "nan" or vl > 1:
			vl = 0.001

		if vr == "nan" or vr > 1:
			vr = 0.001


		next_pose = [0,0,0,0,0,0] #x,y,theta, vx, vy, w
		elapsed_time = rospy.get_time() - self.start_time

		l = 0.5		
		vc = (vl+vr) / l
		wc = (vr-vl) / l

		current_angle = self.current_pose[2]
		next_angle = current_angle + wc*elapsed_time
		#rospy.loginfo("current angle: %f, next_angle: %f, vc: %f, wc: %f" % (current_angle, next_angle, vc, wc))
	
		if wc != 0:
			rospy.loginfo("vl: %f, vr: %f" % (vl,vr))	
			R = l/2*(vl+vr)/(vl-vr)
			ICCx = self.current_pose[0] - R*math.sin(current_angle)
			ICCy = self.current_pose[1] + R*math.cos(current_angle)
			next_pose[0] = math.cos(wc*elapsed_time)*(self.current_pose[0] - ICCx) - math.sin(wc*elapsed_time)*(self.current_pose[1] - ICCy) + ICCx 
			next_pose[1] = math.sin(wc*elapsed_time)*(self.current_pose[0] - ICCx) + math.cos(wc*elapsed_time)*(self.current_pose[1] - ICCy) + ICCy
			next_pose[2] = next_angle
			next_pose[5] = wc
			next_pose[3] = (vr+vl)/2 * math.cos(current_angle)
			next_pose[4] = (vl + vl)/2 * math.sin(current_angle)
			self.total_time += elapsed_time

			rospy.loginfo("total time: %f" % (self.total_time))								

		else:
			next_pose[0] = self.current_pose[0] + vc*elapsed_time*math.cos(current_angle)
			next_pose[1] = self.current_pose[1] + vc*elapsed_time*math.sin(current_angle) 
			next_pose[2] = next_angle
			next_pose[5] = wc
			next_pose[3] = (next_pose[1] - self.current_pose[1])/elapsed_time
			next_pose[4] = (next_pose[2] - self.current_pose[2])/elapsed_time
		
		#rospy.loginfo(next_pose)				
	
		self.current_pose = next_pose
		self.start_time = rospy.get_time()


	def encoderCallback(self,msg):
		if self.counter < 25:
			self.counter += 1

		else:
			factorl = 1.7
			factorr = 2

			msg = [msg.data[0]*factorl, msg.data[1]*factorr]		
			self.convertToPose(msg[0], msg[1])

			if msg[0] < 0:
				msg[0] *= -1
		
			if msg[1] < 0:
				msg[1] *= -1


			if msg[0] != 0 or msg[1] != 0:		
				rospy.loginfo("x : %f, y: %f" % (self.current_pose[0], self.current_pose[1]))

			new_msg = Odometry()
			pose_covariance = [-1 for i in range( 36)] #x,y,z,r,p,y
			twist_covariance = [-1 for i in range(36)] #vx,vy,vz, wr,wp,wy

			kxy = 1e-8#self.compute_covariance(self.x_val, self.y_val)
			kxt = 1e-8#self.compute_covariance(self.x_val, self.yaw_val)
			kyt = 1e-8#self.compute_covariance(self.y_val, self.yaw_val)
	
			kvxy = 1e-8#self.compute_covariance(self.vx,self.vy)		
			kvxt = 1e-8#self.compute_covariance(self.vx,self.vt)
			kvyt = 1e-8#self.compute_covariance(self.vt,self.vy)

			varx = 0.01#self.compute_covariance(self.x_val, self.x_val)
			vary = 0.01#self.compute_covariance(self.y_val, self.y_val)		
			vart = 0.01#self.compute_covariance(self.yaw_val, self.yaw_val)

			varvx = 0.01#self.compute_covariance(self.vx,self.vx)
			varvy = 0.01#self.compute_covariance(self.vy,self.vy)
			varvt = 0.01#self.compute_covariance(self.vt,self.vt)

			pose_covariance[0] = varx
			pose_covariance[7] = vary
			pose_covariance[35] = vart
			pose_covariance[14] = 1e-9
			pose_covariance[21] = 1e-9
			pose_covariance[28] = 1e-9

			pose_covariance[1] = kxy
			pose_covariance[5] = kxt
			pose_covariance[11] = kyt

			pose_covariance[6] = pose_covariance[1]
			pose_covariance[30] = pose_covariance[5]
			pose_covariance[31] = pose_covariance[11]

			twist_covariance[0] = varvx
			twist_covariance[7] = varvy
			twist_covariance[35] = varvt
			twist_covariance[14] = 1e-9
			twist_covariance[21] = 1e-9			
			twist_covariance[28] = 1e-9

			twist_covariance[1] = kvxy
			twist_covariance[5] = kvxt
			twist_covariance[11] = kvyt

			twist_covariance[6] = twist_covariance[1]
			twist_covariance[30] = twist_covariance[5]
			twist_covariance[31] = twist_covariance[11]

			new_msg.pose.pose.position.x = self.current_pose[0]*1.5*-1
			new_msg.pose.pose.position.y = self.current_pose[1]*1.5

			q = new_msg.pose.pose.orientation
			q1 = quaternion_from_euler(0,0,self.current_pose[2])
			q.x, q.y, q.z, q.w  = q1[0], q1[1], q1[2], q1[3]

			new_msg.twist.twist.linear.x = self.current_pose[3]
			new_msg.twist.twist.linear.y =  self.current_pose[4]
			new_msg.twist.twist.angular.z =  self.current_pose[5]

			new_msg.pose.covariance = pose_covariance
			new_msg.twist.covariance = twist_covariance

			new_msg.header.frame_id = "odom_combined"
			new_msg.header.stamp = rospy.Time.now()
			new_msg.child_frame_id = "base_footprint"
			pub = rospy.Publisher("odom", Odometry, queue_size = 10)
			pub.publish(new_msg)
			#rospy.loginfo("odom published")
			#rospy.loginfo("left: %f, right: %f" % (msg[0], msg[1]))
			self.counter = 0



if __name__ == "__main__":
	rospy.init_node("encoder_listener")
	encode = Encoder()
	rospy.Subscriber("encoded", Float32MultiArray, encode.encoderCallback)

	rospy.spin()
