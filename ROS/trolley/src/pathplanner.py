import rospy
import time
import math
import numpy
import matplotlib.pyplot as plt

from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from std_msgs.msg import Float64
from trolley.msg import Wheels



class Coordinate(object):
	def __init__(self, a = 0, b = 0, t = 0):
		self.x = a
		self.y = b
		self.theta = t

class Path_Planner(object):
	def __init__(self):
		super(Path_Planner, self).__init__()
		self.base_length = 1 #wheel to wheel length
		self.timeStep = 0.2
		self.pt1 = Coordinate()
		self.trolley = Coordinate()
		self.base_speed = 0.12
		self.get_dock_pose()
		rospy.loginfo("Marker pose set")



	def compute_radius(self):
		A = Coordinate()
		B = self.pt1
		C = self.trolley
		
		lenAB = math.sqrt((A.x - B.x)**2 + (A.y - B.y)**2)
		lenAC = math.sqrt((A.x - C.x)**2 + (A.y - C.y)**2) 
		lenBC = math.sqrt((B.x - C.x)**2 + (B.y - C.y)**2)
		area = abs((A.x-C.x)*(B.y-A.y)-(A.x-B.x)*(C.y-A.y))/2

		radius = lenAB * lenAC * lenBC / (4 * area)
		rospy.loginfo("Radius: %f" % (radius))
		return float(radius)

	def compute_wheel_vel(self, r):
		"""
		If the control point is on the right of the robot, robot should turn right
		Wheel velocities are related by the equation vr(r-l/2) = vl(r+l/2)
		The slower wheel will be set to the minimum wheel velocity
		"""	
		d = float(self.base_length)/2
		if self.pt1.x > 0.05:
			rospy.loginfo("x > 0")
			vr = self.base_speed
			vl = vr*float(r-d)/(r+d) + 0.1

		elif self.pt1.x < -0.05:
			rospy.loginfo("x < 0")
			vl = self.base_speed
			vr = vl*(r+d)/(r-d) + 0.1

		else:
			vl = self.base_speed
			vr = self.base_speed
		
		rospy.loginfo("vl: %f vr: %f" % (vl,vr))
		return(vl, vr)

	def compute_control_point(self):
		self.pt1.x = self.trolley.x / 2 
		self.pt1.y = self.trolley.y / 2 - 0.3
		rospy.loginfo("Control point x: %f, y: %f" % (self.pt1.x, self.pt1.y))

		
class Gripper(object):
	def __init__(self):
		super(Gripper, self).__init__()
		self.rate = rospy.Rate(10)
		
	def gripper_up(self):
		pub = rospy.Publisher("servo", Float64, queue_size = 30)
		msg = Float64()
		msg.data = float(1)
		pub.publish(msg)
		start_time = time.time()
		while True:
	   		now = time.time()
			pub.publish(msg)
			rospy.loginfo("Raise grippers")
			rospy.sleep(0.5)
			now = time.time()
			if now - start_time > 5:
				return 0
		
	def gripper_down(self):
		pub = rospy.Publisher("servo", Float64, queue_size = 30)
		msg = Float64()
		msg.data = float(0) 
		start_time = time.time()
		while True:
	   		now = time.time()
			pub.publish(msg)
			rospy.loginfo("Lower grippers")
			rospy.sleep(0.5)
			now = time.time()
			if now - start_time > 5:
				return 0			



class Robot(Path_Planner, Gripper):
	def __init__(self):
		super(Robot,self).__init__()
		self.rate=rospy.Rate(10)
		self.current_pose = Coordinate()
		self.start_time = time.time()
		rospy.loginfo("Time set")
		self.elapsed_time = 0
		self.start_pose = Coordinate()
		self.obstacle_flag = 0

	def check_obstacle(self):
		rospy.Subscriber("obstacle", Float64, self.obstacle_cb)

	def obstacle_cb(self, msg):
		self.obstacle_flag = msg.data

	def vel_publish(self, left_vel = 0.12, right_vel = 0.12, t = 0): #publish movements
		pub = rospy.Publisher("/cmd_vel", Wheels, queue_size = 30)
		msg = Wheels()
		if int(self.obstacle_flag) == 1:
			rospy.loginfo("obstacle_detected, stopping")
			msg.left = 0
			msg.right = 0
			pub.publish(msg)

		else:
			msg.left = left_vel
			msg.right = right_vel

			rospy.loginfo("Obstacle gone, continuing")
			self.current_command = msg
			start_time = time.time()
			pub.publish(msg)
			if t != 0:
				while True:
		   			now = time.time()
					pub.publish(msg)
					rospy.sleep(0.5)
					now = time.time()
					if now - start_time > t:
						return 0

	def ekf_sub(self):
		rospy.Subscriber("/odometry/filtered", Odometry, self.get_current_pose)
		self.elapsed_time = time.time() - self.start_time
		rospy.sleep(0.2)

	def get_current_pose(self, msg):
		if self.start_pose.x == 0 or self.start_pose.y == 0 or self.start_pose.theta == 0:
			rospy.loginfo("setting initial pose")
			self.start_pose.x = msg.pose.pose.position.x
			self.start_pose.y = msg.pose.pose.position.y
			(roll, ptich,yaw) = euler_from_quaternion([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
			self.start_pose.theta = yaw
			rospy.loginfo("initial x: %f, initial y: %f, initial theta: %f" % (self.start_pose.x, self.start_pose.y, self.start_pose.theta))
		
		self.current_pose.x = msg.pose.pose.position.x - self.start_pose.x
		self.current_pose.y = msg.pose.pose.position.y - self.start_pose.y
		
		(roll, ptich,yaw) = euler_from_quaternion([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
		self.current_pose.theta = yaw - self.start_pose.theta
		#rospy.loginfo("msg x: %f, msg y: %f, msg theta: %f" % (msg.pose.pose.position.x, msg.pose.pose.position.y, yaw))
		#rospy.loginfo("current x: %f, current y: %f, current theta: %f" % (self.current_pose.x, self.current_pose.y, self.current_pose.theta))


	def go(self):
		"""
		The robot travels to the trolley by generating an arc between it and the dock
		1. get marker pose
		2. generate control point to set the arc
		3. compute radius of arc
		4. compute wheel velocities given arc radius
		5. publish velocities
		6. repeat
		"""		
		self.compute_control_point()
		r = self.compute_radius()
		vl, vr = self.compute_wheel_vel(r)
		self.vel_publish(vl, vr, 1)


	def align(self):
		"""
		Align the robot to the marker by driving the yaw of the marker to zero
		bang bang control
		"""
		rospy.loginfo("alignment")
		while abs(self.trolley.theta) > 0.05 or self.trolley.y > 0.1:	
			self.get_dock_pose()
			x_err = 0.2 * self.trolley.x
			rospy.loginfo(self.trolley.theta)			
			if self.trolley.theta > 0.3:
				vl = 0.15 
				vr = 0.24

			elif self.trolley.theta > 0.15:
				vl = 0.15
				vr = 0.19

			elif self.trolley.theta > 0.05:
				vl = 0.15
				vr = 0.18

			elif self.trolley.theta < -0.3:
				vr = 0.15
				vl = 0.24

			elif self.trolley.theta < -0.15:
				vr = 0.15
				vl = 0.19

			elif self.trolley.theta < -0.05:
				vr = 0.15
				vl = 0.15

			else:
				rospy.loginfo("angle aligned")
				if x_err > 0.05:
					vl = 0.16
					vr = 0.12

				elif x_err < -0.05:
					vl = 0.12
					vr = 0.16

				else:
					rospy.loginfo("aligned")
					vl = 0.15
					vr = 0.15

			self.vel_publish(vl,vr)
		if self.trolley.y < 0.1:
			self.vel_publish(vl,vr,2)


	def stop(self):
		self.vel_publish(0.13)
		self.vel_publish(0.12)
		self.vel_publish(0.12)
		self.vel_publish(0.11)
		self.vel_publish(0.11)
		self.vel_publish(0,0,2)	

	def is_docked(self):
		if abs(self.current_pose.x - self.trolley.x) < 0.3 and abs(self.current_pose.y - self.trolley.y) < 0.3 and abs(self.current_pose.theta - self.trolley.theta) < 0.1:
			rospy.loginfo("Docked")
			self.gripper_up()
			return True

		else:
			return False


	def get_dock_pose(self):
		rospy.Subscriber("aruco_single/pose", PoseStamped, self.aruco_callback)
		

					
		
	def aruco_callback(self, msg):
		dist = msg.pose.position
		quaternion = msg.pose.orientation
		r,p,y = euler_from_quaternion((quaternion.x, quaternion.y, quaternion.z, quaternion.w))

		if dist.x < 0:
			self.trolley.x = dist.x - 0.1

		else:
			self.trolley.x = dist.x + 0.1

		self.trolley.y = dist.z - 0.3
		self.trolley.theta = p


		#rospy.loginfo("roll: %f, pitch: %f yaw: %f" % (r,p,y))
		#rospy.loginfo("trolley x: %f, trolley y: %f, trolley angle: %f" % (dist.x, dist.z, y))
		
		
	def aruco_callback2(self, msg):
		dist = msg.pose.position
		quaternion = msg.pose.orientation
		r,p,y = euler_from_quaternion((quaternion.x, quaternion.y, quaternion.z, quaternion.w))

		if dist.x < 0:
			self.trolley.x = dist.x - 0.1

		else:
			self.trolley.x = dist.x + 0.1

		self.trolley.y = dist.z - 0.3
		self.trolley.theta = p
		
		if self.trolley.y > 1.2:
			self.go()
			
		elif self.trolley.y > 0.1:
			rospy.loginfo("Beginning alignment")
			if self.trolley.theta > 0.3:
				vl = 0.15 
				vr = 0.3

			elif self.trolley.theta > 0.15:
				vl = 0.15
				vr = 0.24

			elif self.trolley.theta > 0.05:
				vl = 0.15
				vr = 0.18

			elif self.trolley.theta < -0.3:
				vr = 0.15
				vl = 0.3

			elif self.trolley.theta < -0.15:
				vr = 0.15
				vl = 0.24

			elif self.trolley.theta < -0.05:
				vr = 0.15
				vl = 0.18

			else:
				rospy.loginfo("angle aligned")
				if x_err > 0.05:
					vl = 0.16
					vr = 0.12

				elif x_err < -0.05:
					vl = 0.12
					vr = 0.16

				else:
					rospy.loginfo("aligned")
					vl = 0.15
					vr = 0.15

			self.vel_publish(vl,vr)
			
		else:
			rospy.loginfo("docked")
		
