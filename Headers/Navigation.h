// add header guards
#pragma once
#include <iostream>
class Navigation {
	/*	distance (meters)
		turn_angle(radians)
	*/
	public:
		void moveForward(float distance) {
			std::cout << "Moving forward " << distance << " m\n";
		};

		void turnLeft(float turn_angle) {
			std::cout << "Turning left " << turn_angle << "radian\n";
		};

		void turnRight(float turn_angle) {
			std::cout << "Turning right " << turn_angle << "radian\n";
		}
};
