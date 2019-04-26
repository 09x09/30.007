#! /usr/bin/env python
import rospy
import pathplanner

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int32

		
def cb(msg):
	data = msg.data
	rospy.loginfo("RFID: %d" % (data))

if __name__ == "__main__":
	rospy.init_node("botherder")
	bot = pathplanner.Robot()
	rospy.Subscriber("rfidread", Int32, cb)

	try:
		rospy.loginfo("Starting in 3")
		rospy.sleep(3)
		"""
		if bot.trolley.x * bot.trolley.theta < 0 and abs(bot.trolley.x > 1) :
			rospy.loginfo("S-curve initiated")
			if bot.trolley.x > 0:
				bot.s_curve(bot.trolley.x + 0.5)

			else:
				bot.s_curve(bot.trolley.x - 0.5)
		

		while bot.trolley.y > 1.2:
			bot.check_obstacle()
			rospy.loginfo("distance: %f" % (bot.trolley.y))
			bot.go()
			
		bot.stop()			
		rospy.loginfo("begin alignment")
		
		bot.align()
		bot.vel_publish(0,0,3)

		if bot.trolley.y < 0.1:
			bot.gripper_up()
			#rospy.sleep(5)
			bot.vel_publish(-0.12,-0.12,5)
			bot.gripper_down()
			bot.rev_stop()
		"""
		while not rospy.is_shutdown():
			pass

	except rospy.ROSInterruptException:
		rospy.loginfo("Done")

	
		


	
