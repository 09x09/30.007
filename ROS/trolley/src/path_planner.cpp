#include "Eigen/Core"
#include <iostream>
#include <string>
#include <fstream>

using namespace std;
using namespace Eigen;

float stepSize = 0.01;
int targetTime = 50;
MatrixXf path(int(1/stepSize)+2,3);
Vector3f origin;
Vector3f trolley;
Vector2f controlPt1;
Vector2f controlPt2;

Vector3f currentPose;
Vector3f targetPose;

float currentTime;
float errorX, errorY, errorTheta;

void generatePath(float x, float y, float theta) {
	origin << 0, 0, 0;
	trolley << x, y, theta;
	controlPt1 << 0, 5;

	if (theta < 0) {
		controlPt2 << x + 2, 2* atan(theta) + y;
	}
	else {
		controlPt2 << x - 2, 2 * atan(theta) + y;
	}
	
	float xValue, yValue, xChange, yChange, angle;
	int counter = 1;
	for (float t = 0; t < 1; t = t + stepSize) {
		xValue = pow((1 - t) , 3) * origin(0) + 3 *pow( (1 - t) , 2) * t*controlPt1(0) + 3 * (1 - t)*pow(t , 2) * controlPt2(0) + pow(t , 3) * trolley(0);
		yValue = pow((1 - t) , 3) * origin(1) + 3 * pow((1 - t), 2) * t*controlPt1(1) + 3 * (1 - t)*pow(t, 2) * controlPt2(1) + pow(t, 3) * trolley(1);
		xChange = 3 * pow((1 - t) , 2) * (controlPt1(0) - origin(0)) + 6 * (1 - t)*t*(controlPt2(0) - controlPt1(0)) + 3 * pow(t , 2) * (trolley(0) - controlPt2(0));
		yChange = 3 * pow((1 - t) , 2) * (controlPt1(1) - origin(1)) + 6 * (1 - t)*t*(controlPt2(1) - controlPt1(1)) + 3 * pow(t , 2) * (trolley(1) - controlPt2(1));
		angle = atan2(yChange , xChange);
		path(counter,0) = xValue;
		path(counter,1) = yValue;
		path(counter,2) = angle;
		counter += 1;
	}

}


void computeError() {
	targetPose(0) = path(0, currentTime);
	targetPose(1) = path(1, currentTime);
	targetPose(2) = path(2, currentTime);

	errorX = (targetPose(0) - currentPose(0)) * cos(targetPose(2)) + (targetPose(1) - currentPose(1))*sin(targetPose(2));
	errorY = (targetPose(0) - currentPose(0)) * sin(targetPose(2)) + (targetPose(1) - currentPose(1))*cos(targetPose(2));
	errorTheta = targetPose(2) - currentPose(2);

}




void main() {
	printf("Start\n");
	generatePath(3, 3, 0.1);
	ofstream mf;
	mf.open("path.txt");
	mf << path;
	mf.close();

}