#! /usr/bin/env python
import rospy
import cv2


aruco_path = r"/home/odroid/Desktop/Aruco_Tracker/range&pose.yaml"

if __name__ == "__main__":
	rospy.init_node("Test")
	rospy.loginfo("Beginning test, hold marker to webcam")
	f = open("/home/odroid/Desktop/Aruco_Tracker/rangeandpose.txt", "r")
	lines = f.readlines()
	rospy.loginfo("Distance: " + str(lines[0]))
	rospy.loginfo("Pose: " + str(lines[1]))
