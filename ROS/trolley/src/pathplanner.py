import rospy
import time
import math

from geometry_msgs.msg import PoseWithCovarianceStamped
from trolley.msg import Wheels

class Path_Planner(object):
	def __init__(self):
		super(Path_Planner, self).__init__()
		self.bpath = []
		self.timeStep = 0.05
		self.origin = [0,0,0]
		self.controlPt1 = [0,5]
		self.controlPt2 = []
		self.trolley = [1,1,0]
		self.bezier_curve(self.trolley[0], self.trolley[1], self.trolley[2])

	def bezier_curve(self,x,y,angle):
		if angle < 0:
			self.controlPt2 = [x+2, 2*math.atan(angle)+y]
		else:
			self.controlPt2 = [x-2, 2*math.atan(angle)+y]

		t = 0
		while t < 1:
			x_value = (1-t)**3*self.origin[0]+3*(1-t)**2*t*self.controlPt1[0]+ 3*(1-t)*t**2*self.controlPt2[0] + t**3 * self.trolley[0]
			y_value = (1-t)**3*self.origin[1]+3*(1-t)**2*t*self.controlPt1[1]+ 3*(1-t)*t**2*self.controlPt2[1] + t**3 * self.trolley[1]

			x_change = 3*(1-t)**2*(self.controlPt1[0]-self.origin[0])+ 6*(1-t)*t*(self.controlPt2[0]-self.controlPt1[0]) + 3*t**2*(self.trolley[0]-self.controlPt2[0])
			y_change = 3*(1-t)**2*(self.controlPt1[1]-self.origin[1])+ 6*(1-t)*t*(self.controlPt2[1]-self.controlPt1[1]) + 3*t**2*(self.trolley[1]-self.controlPt2[1])
			angle_value = math.atan2(y_change, x_change)
			self.bpath.append([x_value, y_value, angle_value])
			t += self.timeStep

		rospy.loginfo("Curve generated")
		rospy.loginfo(self.bpath)

	def compute_error(self, current_time, current_pose):
		target_pose = self.bpath[current_time/total_time*bpath.length]
		err_x = (target_pose[0] - current_pose[0]) * math.cos(target_pose[2]) + (target_pose[1] - current_pose[1])*math.sin(target_pose[2]);
		err_y = (target_pose[0] - current_pose[0]) * math.sin(target_pose[2]) + (target_pose[1] - current_pose[1])*math.cos(target_pose[2]);
		err_theta = target_pose[2] - current_pose[2]
		
		err = [err_x,err_y,err_theta]
		return err


class Robot(Path_Planner):
	def __init__(self):
		super(Robot,self).__init__()
		self.rate=rospy.Rate(10)
		self.current_pose = [0,0,0]
		self.start_time = time.time()
		rospy.loginfo("Time set")
		self.elapsed_time = 0

	def vel_publish(self, left_vel = 0.2, right_vel = 0.2): #publish movements
		pub = rospy.Publisher("/cmd_vel", Wheels, queue_size = 30)
		msg = Wheels()
		msg.left = left_vel
		msg.right = right_vel
		while True:
	    		connections = pub.get_num_connections()
	    		if connections > 0:
				pub.publish(msg)
				rospy.loginfo("Publishing movement")
				rospy.sleep(0.05)

				break
	    		else:
				self.rate.sleep()

	def ekf_sub(self):
		rospy.Subscriber("robot_pose_ekf/odom_combined", PoseWithCovarianceStamped, self.get_current_pose)
		rospy.sleep(0.05)

	def get_current_pose(self, msg):
		self.elapsed_time = time.time() - self.start_time
		self.current_pose[0] = msg.pose.pose.position.x
		self.current_pose[1] = msg.pose.pose.position.y
		
		(roll, ptich,yaw) = euler_from_quaternion(msg.pose.pose.orientation)
		self.current_pose[2] = yaw

	def gen_vel(self):
		left_vel = 0
		rigt_vel = 0
		return left_vel,right_vel

	
	def is_docked(self):
		if abs(self.current_pose[0] - self.trolley[0]) < 0.3 and abs(self.current_pose[1] - self.trolley[1]) < 0.3 and abs(self.current_pose[2] - self.trolley[2]) < 0.1:
			return True

		else:
			return False



	
