#! /usr/bin/env python

import rospy

from std_msgs.msg import Float64
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int32MultiArray

class Obstacle(object):
	def __init__(self):
		self.pub = rospy.Publisher("obstacle", Float64, queue_size = 30)
		self.counter = 0 
		self.count = 0
		self.left = 0
		self.right = 0

	def callback(self, msg):
		try:
			data = msg.data
			self.left = int(msg.data[0])
			self.right = int(msg.data[1])

		except:
			self.count += 1	

	
		

		if self.count > 30 and self.left > 8000 and self.right > 8000:		
			msg1 = Float64()
			msg1.data = 0
			self.pub.publish(msg1)
			self.count = 0
			rospy.loginfo("go")
			self.counter = 0




if __name__ == "__main__":
	rospy.init_node("safety")
	eye = Obstacle()
	while True:
		rospy.Subscriber("aruco_single/pose", PoseStamped, eye.callback)
		rospy.Subscriber("rangefinder", Int32MultiArray, eye.callback)
		pub = rospy.Publisher("obstacle", Float64, queue_size = 5)
		msg = Float64()	
		msg.data = 1

		if eye.counter == 1:
			pub.publish(msg)

		else:
			eye.counter = 1
		
		rospy.sleep(0.5)
