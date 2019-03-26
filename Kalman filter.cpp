#include <iostream>
#include "Eigen/Core"
// # include "ros.h"
//# include  "sensor_msgs/Imu.h"

using namespace std;
using namespace Eigen;

float imuData[3][50] = {};
float encoderData[50][3] = {};

/*tracks 6 states: x,y,theta, v_x, v_y, omega*/
VectorXd state(6); 
VectorXd predState(6); 
VectorXd prevState

/*2 inputs: v_left, v_right*/
VectorXd commands; 

/*6x6 matrix*/
MatrixXd covariance = MatrixXd::Identity(6,6); 
MatrixXd predConfidence = MatrixXd::Identity(6, 6);
MatrixXd confidence = MatrixXd::Identity(6, 6);
MatrixXd prevConfidence = MatrixXd::Identity(6, 6);
MatrixXd sensorModel = MatrixXd::Identity(6, 6);
MatrixXd sensorConfidence = MatrixXd::Identity(6, 6);
MatrixXd measurements = MatrixXd::Identity(6, 6);

/*temp matrices*/
MatrixXd A;
MatrixXd B;
MatrixXd K;

/*Covariance variables*/

void initMatrix() {
	state << 0,0 0,0,0,0;
	predState << 0,0,0,0,0,0;
	commands << 0,0,0,0,0,0;


	

	prevState = state

}

void kalmanFilter() {
	predState = A * prevState + B * commands;
	predConfidence = A * prevConfidence*Transpose(A) + covariance;

	K = predConfidencep * Transpose(sensorModel)*Inverse(sensorModel*predConfidence*Transpose(sensorModel) + sensorConfidence);
	state = predState + K * (measurements - sensorModel * predState);
	confidence = (1-K*H)*predConfidence
}

void encoderCallback{

}

void imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
	//store data in variables
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "listener");
	ros::NodeHandle sensorListener;
	ros::subscriber sub = sensorListener.subscribe("imu", 100, imuCallback);
	ros::subscriber sub2 = sensorListener.subscribe("encoder", 100, encoderCallback);

	kalmanFilter();
	    	  
}
