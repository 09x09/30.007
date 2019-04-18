import rospy
import pathplanner

if __name__ == "__main__":
	rospy.init_node("botherder")
	bot = pathplanner.Robot()
	bot.trolley = pathplanner.Coordinate(0,1)
	try:	
		while bot.is_docked != True:
			bot.vel_publish()
			bot.ekf_sub()

			if bot.current_pose.y >  1:
				rospy.loginfo("overshoot")				
				break

		bot.vel_publish(0,0)

	except rospy.ROSInterruptException:
		bot.vel_publish(0,0)
		rospy.loginfo("crashed")

	
		


	
