#! /usr/bin/env python

import rospy
import time

from trolley.msg import Wheels

msg = Wheels()

if __name__ == "__main__":
	rospy.init_node("tester")
	msg.left = 0.2
	msg.right = 0.2
	pub = rospy.Publisher("/cmd_vel", Wheels, queue_size = 10)
	while True:
		connections = pub.get_num_connections()
		if connections > 0:
			pub.publish(msg)
			rospy.loginfo("Publishing movement")
			rospy.sleep(5)

			break
		else:
			self.rate.sleep()
