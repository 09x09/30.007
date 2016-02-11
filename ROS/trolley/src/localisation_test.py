#! /usr/bin/env python
import rospy
import pathplanner

from geometry_msgs.msg import PoseStamped

def gripper_test(bot):
	rospy.loginfo("Gripper Test")
	rospy.loginfo("Lower gripper")
	bot.gripper_down()
	rospy.sleep(5)
	rospy.loginfo("Raise gripper")
	bot.gripper_up()
	rospy.loginfo("Gripper test complete")

def localisation_test(bot, vl, vr):	
	vell, velr = vl, vr
	counter = 1	
	test_x = 1
	test_time = 10
	while counter < test_time:
		bot.vel_publish(vell,velr)
		bot.ekf_sub()
		counter +=1
	bot.vel_publish(0,0)

		
def cb(msg):
	rospy.loginfo("received")

if __name__ == "__main__":
	rospy.init_node("botherder")
	bot = pathplanner.Robot()

	try:
		rospy.loginfo("Starting in 5")
		rospy.sleep(5)
		while bot.trolley.y > 1:
			rospy.loginfo("distance: %f" % (bot.trolley.y))
			bot.go()
			
		bot.stop()			
		rospy.loginfo("begin alignment")
		bot.align()
		bot.stop()
		bot.gripper_up()
		rospy.sleep(5)
		#bot.publish_vel(-0.12,-0.12,5)
		bot.gripper_down()

	except rospy.ROSInterruptException:
		pass

	
		


	
