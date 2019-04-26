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

"""
All dimensions are in SI units
"""

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
		self.base_speed = 0.13 
		self.get_dock_pose()
		self.check_obstacle()
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
		self.pt1.x = self.trolley.x  *5/6
		self.pt1.y = self.trolley.y * 2 

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
		self.reverse_flag = 0
		
	
	def check_obstacle(self):
		rospy.Subscriber("obstacle", Float64, self.obstacle_cb)

	def obstacle_cb(self, msg):
		rospy.loginfo(msg.data)
		self.obstacle_flag = msg.data

	def vel_publish(self, left_vel = 0.12, right_vel = 0.12, t = 0): #publish movements
		pub = rospy.Publisher("/cmd_vel", Wheels, queue_size = 30)
		msg = Wheels()
		rospy.loginfo(self.obstacle_flag)
		rospy.loginfo(self.reverse_flag)
		if self.obstacle_flag == 1 and self.reverse_flag == 0:
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

	def s_curve(self, d):
		"""
		Function takes diameter and converts it into the necessary wheel velocities to traverse a S-curve
		"""	
		radius = d/2
		vl, vr = self.compute_wheel_vel(radius)
		w = (vl-vr)/2
		
		#time taken to travel quarter circle
		t = (math.pi/2)/w -1

		self.vel_publish(vl,vr,t)
		self.vel_publish(self.base_speed, self.base_speed, radius)
		self.vel_publish(vr,vl,t)
		self.stop()
		

	def go(self):
		"""
		The robot travels to the trolley by generating an arc between it and the dock
		1. get marker pose
		2. If trolley is faced away from robot traverse in s curve to shift robot laterally
		3. generate control point to set the arc
		4. compute radius of arc
		5. compute wheel velocities given arc radius
		6. publish velocities
		7. repeat
		"""		
		self.compute_control_point()
		r = self.compute_radius()
		vl, vr = self.compute_wheel_vel(r)
		self.vel_publish(vl, vr)


	def align(self):
		"""
		Align the robot to the marker by driving the yaw of the marker to zero
		bang bang control
		"""
		rospy.loginfo("alignment")
		while abs(self.trolley.theta) > 0.05 or self.trolley.y > 0.05:	
			x_err = 0.2 * self.trolley.x
			rospy.loginfo("Loop")
			rospy.loginfo(self.trolley.theta)			
			if self.trolley.theta > 0.2:
				vl = 0.15 
				vr = 0.30

			elif self.trolley.theta > 0.1:
				vl = 0.15
				vr = 0.22

			elif self.trolley.theta > 0.01:
				vl = 0.15
				vr = 0.17

			elif self.trolley.theta < -0.2:
				vr = 0.15
				vl = 0.24

			elif self.trolley.theta < -0.1:
				vr = 0.15
				vl = 0.19

			elif self.trolley.theta < -0.01:
				vr = 0.15
				vl = 0.17

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
					vl = 0.2
					vr = 0.2

			self.vel_publish(vl,vr,2)
			if self.trolley.y < 0.1:
				break
		


	def stop(self):
		self.vel_publish(0.12, 0.12)
		self.vel_publish(0.10, 0.10)
		self.vel_publish(0.08, 0.08)
		self.vel_publish(0.06, 0.06)
		self.vel_publish(0,0,5)	

	def rev_stop(self):
		self.vel_publish(-0.12, -0.12)
		self.vel_publish(-0.10, -0.10)
		self.vel_publish(-0.08, -0.08)
		self.vel_publish(-0.06, -0.06)
		self.vel_publish(-0,-0,5)

	def is_docked(self):
		if abs(self.current_pose.x - self.trolley.x) < 0.3 and abs(self.current_pose.y - self.trolley.y) < 0.3 and abs(self.current_pose.theta - self.trolley.theta) < 0.1:
			rospy.loginfo("Docked")
			self.gripper_up()
			return True

		else:
			return False


	def get_dock_pose(self):
		rospy.Subscriber("aruco_single/pose", PoseStamped, self.aruco_callback2)
		

					
		
	def aruco_callback(self, msg):
		dist = msg.pose.position
		quaternion = msg.pose.orientation
		r,p,y = euler_from_quaternion((quaternion.x, quaternion.y, quaternion.z, quaternion.w))

		y_val = math.sqrt(dist.z**2 - dist.x**2) 

		self.trolley.theta = p
		self.trolley.x = dist.x + 0.62 * math.sin(p)
		self.trolley.y = y_val - 0.62 * math.cos(p)
		rospy.loginfo("y: %f" % (self.trolley.y))

		#rospy.loginfo("roll: %f, pitch: %f yaw: %f" % (r,p,y))
		#rospy.loginfo("trolley x: %f, trolley y: %f, trolley angle: %f" % (dist.x, dist.z, y))


	def aruco_callback2(self, msg):
		dist = msg.pose.position
		quaternion = msg.pose.orientation
		r,p,y = euler_from_quaternion((quaternion.x, quaternion.y, quaternion.z, quaternion.w))

		y_val = math.sqrt(dist.z**2 - dist.x**2) 

		self.trolley.theta = p
		self.trolley.x = dist.x + 0.62 * math.sin(p)
		self.trolley.y = y_val - 0.62 * math.cos(p)
		rospy.loginfo("y: %f" % (self.trolley.y))
		
		if self.trolley.y > 1.2:
			self.go()
			
		elif self.trolley.y > 0.1:
			rospy.loginfo("Beginning alignment")
			if self.trolley.theta > 0.3:
				vl = 0.15 
				vr = 0.25

			elif self.trolley.theta > 0.15:
				vl = 0.15
				vr = 0.2

			elif self.trolley.theta > 0.05:
				vl = 0.15
				vr = 0.18

			elif self.trolley.theta < -0.3:
				vr = 0.15
				vl = 0.25

			elif self.trolley.theta < -0.15:
				vr = 0.15
				vl = 0.2

			elif self.trolley.theta < -0.05:
				vr = 0.15
				vl = 0.18

			else:
					vl = 0.15
					vr = 0.15

			self.vel_publish(vl,vr)
			
		else:
			rospy.loginfo("docked")
			self.vel_publish(0,0,1)
			self.reverse_flag = 1
			self.gripper_up()
			#rospy.sleep(5)
			self.vel_publish(-0.12,-0.12,5)
			self.gripper_down()
			self.rev_stop()
			rospy.is_shutdown = True
			self.reverse_flag = 0
			rospy.sleep(5)

