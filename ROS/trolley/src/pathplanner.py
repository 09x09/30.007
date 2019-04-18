import rospy
import time
import math
import numpy
import matplotlib.pyplot as plt

from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import Int32
from trolley.msg import Wheels

class Coordinate(object):
	def __init__(self, a = 0, b = 0, t = 0):
		self.x = a
		self.y = b
		self.theta = t

class Path_Planner(object):
	def __init__(self):
		super(Path_Planner, self).__init__()
		self.bpath = []
		self.timeStep = 0.01
		self.origin = Coordinate()
		self.controlPt2 = Coordinate()
		self.trolley = Coordinate(5,5,1.5)
		self.controlPt1 = Coordinate(0, self.trolley.y,0)
		self.bezier_curve()
		self.base_length = 0.7
		self.a_max = 0.2
		self.landing_coeff = 0.5
		self.command = Wheels()
		self.total_time = 30

	def bezier_curve(self):
		self.controlPt2.y = 0
		if self.trolley.theta == 0:
			self.controlPt2.x = self.trolley.x
		else:
			self.controlPt2.x = self.trolley.x-self.trolley.y/math.tan(math.pi/2 - self.trolley.theta)
			if self.controlPt2.x <= 0:
				self.controlPt2.x = 0
				self.controlPt2.y = -self.trolley.x*math.tan(math.pi/2 - self.trolley.theta)+self.trolley.y
				self.controlPt1.y *= 0.4

		current_angle = 0
		rospy.loginfo([self.controlPt2.x, self.controlPt2.y])
		t = 0
		append = self.bpath.append
		while t < 1:
			x_value = (1-t)**3*self.origin.x+3*(1-t)**2*t*self.controlPt1.x+ 3*(1-t)*t**2*self.controlPt2.x + t**3 * self.trolley.x
			y_value = (1-t)**3*self.origin.y+3*(1-t)**2*t*self.controlPt1.y+ 3*(1-t)*t**2*self.controlPt2.y + t**3 * self.trolley.y

			x_change = 3*(1-t)**2*(self.controlPt1.x-self.origin.x)+ 6*(1-t)*t*(self.controlPt2.x-self.controlPt1.x) + 3*t**2*(self.trolley.x-self.controlPt2.x)
			y_change = 3*(1-t)**2*(self.controlPt1.y-self.origin.y)+ 6*(1-t)*t*(self.controlPt2.y-self.controlPt1.y) + 3*t**2*(self.trolley.y-self.controlPt2.y)
			angle_value = math.atan2(y_change, x_change)

			angle_change = angle_value - current_angle
			vx = x_change/self.timeStep
			vy = y_change/self.timeStep
			w = angle_change/self.timeStep			

			append([x_value, y_value, angle_value, vx, vy, w])
			t += self.timeStep
			current_angle = angle_value

		xv = [i[0] for i in self.bpath]
		yv = [j[1] for j in self.bpath]
		
		plt.scatter(self.controlPt1.x, self.controlPt1.y)
		plt.scatter(self.controlPt2.x, self.controlPt2.y)
		plt.scatter(xv,yv)
		plt.show()

		rospy.loginfo("Curve generated")
		rospy.loginfo(self.bpath)

	def compute_error(self, current_time, current_pose):
		target_pose = self.bpath[int(current_time/self.total_time*len(self.bpath))]
		rospy.loginfo(target_pose)
		rospy.loginfo(int(current_time/self.total_time*len(self.bpath)))
		err_x = (target_pose[0] - current_pose.x) * math.cos(target_pose[2]) + (target_pose[1] - current_pose.y)*math.sin(target_pose[2]);
		err_y = (target_pose[0] - current_pose.x) * math.sin(target_pose[2]) + (target_pose[1] - current_pose.y)*math.cos(target_pose[2]);
		err_theta = target_pose[2] - current_pose.theta
		
		err = [err_x,err_y,err_theta]
		return err

	def convert_to_wheel_vel(self, vc, wc):
		vr = vc + wc*self.base_length/2
		vl =vc - wc*self.base_length/2
		return [vl,vr]

	def compute_new_vel(self, err, current_pose, current_time):
		target_pose = self.bpath[int(current_time/self.total_time*len(self.bpath))]
		theta_p = target_pose[2] + math.atan(2*self.landing_coeff*(err[1]/self.landing_coeff)**(2/3)*numpy.sign(err[1]))
		wp = target_pose[5] + 2*(err[1]/self.landing_coeff)**(-1/3) / (1 + (math.tan(theta_p - target_pose[2])**2) * (-target_pose[5] * err[0] + 0.5*(self.command.left + self.command.right)* math.sin(err[2])))*numpy.sign(err[1])

		ws = wp + math.sqrt(2*self.a_max*abs(theta_p - current_pose.y)) * numpy.sign(theta_p - current_pose.y)
		ac = ws/self.timeStep
		if abs(ac > self.a_max):
			ac = self.a_max
		new_w = (self.command.right-self.command.left)/2 + ac*self.timeStep

		vs = math.sqrt(target_pose[3]**2 + target_pose[4]**2) + math.sqrt(2*self.a_max*abs(err[1]))*numpy.sign(err[1])
		ac1 = vs/self.timeStep
		if ac1 > self.a_max:
			ac1 = self.a_max
		new_v = (self.command.left+self.command.right)/2 + ac1*self.timeStep
		return [new_v, new_w] 
		
		
class Gripper(object):
	def __init__(self):
		super(Gripper, self).__init__()
		self.rate = rospy.Rate(10)
		
	def gripper_up(self):
		pub = rospy.Publisher("servo", Int32, queue_size = 30)
		msg = Int32
		msg.data = 0
		while True:
	    		connections = pub.get_num_connections()
	    		if connections > 0:
				pub.publish(msg)
				rospy.loginfo("Raising grippers")
				rospy.sleep(1)

				break
	    		else:
				self.rate.sleep()
		
	def gripper_down(self):
		pub = rospy.Publisher("servo", Int32, queue_size = 30)
		msg = Int32
		msg.data = 1
		while True:
	    		connections = pub.get_num_connections()
	    		if connections > 0:
				pub.publish(msg)
				rospy.loginfo("Raising grippers")
				rospy.sleep(1)

				break
	    		else:
				self.rate.sleep()

class Robot(Path_Planner, Gripper):
	def __init__(self):
		super(Robot,self).__init__()
		self.rate=rospy.Rate(10)
		self.current_pose = Coordinate()
		self.start_time = time.time()
		rospy.loginfo("Time set")
		self.elapsed_time = 0

	def vel_publish(self, left_vel = 0.2, right_vel = 0.2): #publish movements
		pub = rospy.Publisher("/cmd_vel", Wheels, queue_size = 30)
		msg = Wheels()
		msg.left = left_vel
		msg.right = right_vel
		self.current_command = msg
		while True:
	    		connections = pub.get_num_connections()
	    		if connections > 0:
				pub.publish(msg)
				rospy.loginfo("Publishing movement")
				rospy.sleep(1)

				break
	    		else:
				self.rate.sleep()

	def ekf_sub(self):
		rospy.Subscriber("robot_pose_ekf/odom_combined", PoseWithCovarianceStamped, self.get_current_pose)
		self.elapsed_time = time.time() - self.start_time
		rospy.loginfo(self.elapsed_time)
		rospy.sleep(0.05)

	def get_current_pose(self, msg):
		rospy.loginfo("EKF msg received")
		rospy.loginfo(self.elapsed_time)
		self.current_pose.x = msg.pose.pose.position.x
		self.current_pose.y = msg.pose.pose.position.y
		
		(roll, ptich,yaw) = euler_from_quaternion(msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
		self.current_pose.theta = yaw


	
	def is_docked(self):
		if abs(self.current_pose.x - self.trolley.x) < 0.3 and abs(self.current_pose.y - self.trolley.y) < 0.3 and abs(self.current_pose.theta - self.trolley.theta) < 0.1:
			rospy.loginfo("Docked")
			self.gripper_up()
			return True

		else:
			return False




