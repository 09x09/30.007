#! /usr/bin/env python

import rospy

from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion, quaternion_from_euler


class IMU(object):
	def __init__(self):
		super(IMU,self).__init__()
		self.x_corrector = 0
		self.y_corrector = 0
		self.z_corrector = 0

		self.r_corrector = 0
		self.p_corrector = 0
		self.yw_corrector = 0

		self.x = []
		self.y = []
		self.z = []
		

		self. counter = 0
		rospy.loginfo("Zeroing")
		while self.counter < 100:
			rospy.loginfo(self.counter)
			rospy.Subscriber("readings", Float32MultiArray,self.zerocb)
			rospy.sleep(0.1)
		
		self.x_corrector = sum(self.x)/len(self.x)
		self.y_corrector = sum(self.y)/len(self.y)
		self.z_corrector = sum(self.z)/len(self.z)
		rospy.loginfo("Zeroing complete, x correction : %f, y correction: %f, z correction: %f" % (self.x_corrector, self.y_corrector, self.z_corrector))
		rospy.Subscriber("readings", Float32MultiArray, self.callback)
	


	def zerocb(self, msg):
		msg = msg.data	
		lvx, lvy, lvz = msg[25], msg[26], msg[27]

		self.x.append(lvx)
		self.y.append(lvy)
		self.z.append(lvz)
		self.counter += 1
	

	def callback(self,msg):
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
		#lv.x, lv.y, lv.z = msg[25]-self.x_corrector, msg[26]-self.y_corrector, msg[27]-self.z_corrector

		cov_l = imu_msg.linear_acceleration_covariance

		for i in range(9):
			cov_q[i] = 0#msg[i+4] 
			cov_a[i] = 0#msg[i+16]
			cov_l[i] = 0#msg[i+28]


		imu_msg.header.frame_id = "base_footprint"	
		imu_msg.header.stamp = rospy.Time.now()
		pub = rospy.Publisher("imu/data_raw", Imu, queue_size = 30)
		pub.publish(imu_msg)
		(r,p,y) = euler_from_quaternion((q.x,q.y,q.z,q.w))
		rospy.loginfo("lx: %f, ly: %f, lz: %f" % (lv.x, lv.y,lv.z))
		#rospy.loginfo("roll: %f, pitch: %f, yaw: %f" % (r,p,y))
		#rospy.loginfo("published")



if __name__ == "__main__":
	rospy.init_node("test")
	imu = IMU()
	rospy.spin()
		
