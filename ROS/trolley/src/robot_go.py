#! /usr/bin/env python

import rospy
import time
import math
import pathplanner

from geometry_msgs.msg import PoseWithCovarianceStamped
from trolley.msg import Wheels

'''
Algorithm:
1. Init bot
	- get trolley pose
	- set control points
	- set control points
2. Start with initial velocity
3. Get current pose
4. Compute error
5. Compute new velocity
6. Publish new velocity
'''

if __name__ == "__main__":
	rospy.init_node("Zoom_Zoom_Control")
	bot = pathplanner.Robot()
	rospy.loginfo("Bot initialised")
	rospy.sleep(5)
	rospy.loginfo("Begin docking operations")
	try:
		bot.vel_publish()
		while bot.is_docked != True:
			bot.ekf_sub()
			bot.compute_error(self.elapsed_time, self.current_pose)
			bot.vel_publish(bot.gen_vel())

	except rospy.ROSInterruptException:
        	pass	

