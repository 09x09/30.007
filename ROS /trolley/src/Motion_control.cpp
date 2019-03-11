#include <ros/ros.h>
#include <trolley/Wheels.h>
#include <stdlib.h>
#include <iostream>

/* Declare and set robot params */

const float BASERADIUS = 0.1; //in meters
std::string TURNDIR = "left";  //left or right
const float TURNRADIUS = 0.5; //if turn radius = 0 then robot goes straight at minimum velocity
const float MINWHEELVEL = 0.1; //in meters per second
const float RUNTIME = 10; //in seconds

/* Rest of code */
float leftWheelVel;
float rightWheelVel;

int main(int argc, char**argv)
{
	ros::init(argc, argv, "Vel_Publisher");
	ros::NodeHandle nh;
	
	ros::Publisher vel_pub = nh.advertise<trolley::Wheels>("/cmd_vel",100);
	ros::Rate loop_rate(5);

	if(TURNRADIUS == 0) {
		leftWheelVel = MINWHEELVEL;
		rightWheelVel = MINWHEELVEL;
	
	} else {
		if(TURNDIR == "left")	{
			leftWheelVel = MINWHEELVEL;
			rightWheelVel = MINWHEELVEL*(1+1/(BASERADIUS*TURNRADIUS));
		
		} else {
			rightWheelVel = MINWHEELVEL;
			leftWheelVel = 	MINWHEELVEL*(1+1/(BASERADIUS*TURNRADIUS));
		}

	}

	std::cout << "Left wheel velocity" << leftWheelVel;
	std::cout << "Right Wheel velocity" << rightWheelVel;

	while(ros::ok())
	{
		trolley::Wheels msg;
		
		msg.left = leftWheelVel;
		msg.right = rightWheelVel;

		vel_pub.publish(msg);

		ros::spinOnce();
		loop_rate.sleep();
	
	}	

	return 0;
}



