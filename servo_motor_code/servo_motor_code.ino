#include <Servo.h>
#include <ros.h>
#include <std_msgs/Float64.h>

ros::NodeHandle servos;


Servo servoLeft;
Servo servoRight;

int pinLeft = 3;
int pinRight = 2;
int turnAngle = 90;

void lowerServo() { //angle in degrees 
  //adds a delay to slow down the servo motion
  for (int pos = 0; pos <= turnAngle; pos++) {
    servoLeft.write(pos);
    servoRight.write(pos);
    delay(10);
  }
}

void raiseServo() { //angle in degrees 
  //adds a delay to slow down the servo motion
  for (int pos = 0; pos <= turnAngle; pos++) {
    servoLeft.write(turnAngle - pos);
    servoRight.write(pos-turnAngle);
    delay(10);
  }
}

void callback(const std_msgs::Float64 &msg){ 
  if(msg.data == 0) {
    servos.loginfo("down");
    lowerServo();
    servos.loginfo("done");
  } 
  if(msg.data == 1){
    servos.loginfo("up");
    raiseServo();
    servos.loginfo("done");
  }
}

ros::Subscriber<std_msgs::Float64> sub("servo", callback);

void setup() {
  servos.initNode();
  servoLeft.attach(pinLeft);
  servoRight.attach(pinRight);
  servos.subscribe(sub);
  
}


void loop() {
  servos.spinOnce();
  delay(10);
}
