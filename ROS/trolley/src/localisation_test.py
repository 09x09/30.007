#! /usr/bin/env python
import rospy
import pathplanner


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

		
	
if __name__ == "__main__":
	rospy.init_node("botherder")
	bot = pathplanner.Robot()
	bot.ekf_sub()

	try:
		localisation_test(bot, 0.12, 0.12)
		rospy.loginfo("x: %f, y: %f, w: %f" % (bot.current_pose.x, bot.current_pose.y, bot.current_pose.theta))


	except rospy.ROSInterruptException:
		pass

	
		


	
