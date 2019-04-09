#! /usr/bin/env python
import rospy
import time

from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion


def callback(msg):
	msg = msg.data
	now = time.time()
	x = msg.pose.pose.position.x
	y = msg.pose.pose.position.y

	q = msg.pose.pose.orientation
	t = euler_from_quaternion(q.x, q.y, q.z, q.w)

	vx = msg.twist.twist.linear.x
	vy = msg.twist.twist.linear.y
	w = msg.twist.twist.angular.z

	out = [x,y,t,vx,vy,vt,now]
	rospy.loginfo(out)

if __name__ == "__main__":

	rospy.init_node("test_listener")
	sub = rospy.Subscriber("/robot_pose_ekf/odom_combined", PoseWithCovarianceStamped, callback)
	rospy.spin()
