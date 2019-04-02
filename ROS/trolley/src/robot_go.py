#! /usr/bin/env python

import rospy
import time
import math
import pathplanner
import numpy

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
			rospy.loginfo("Loop")
			bot.ekf_sub()
			err = bot.compute_error(bot.elapsed_time, bot.current_pose)
			new_command = bot.compute_new_vel(err, bot.current_pose, bot.elapsed_time)
			v_new = bot.convert_to_wheel_vel(new_command[0], new_command[1]) 
			bot.vel_publish(v_new[0], v_new[1])

	except rospy.ROSInterruptException:
        	pass	

