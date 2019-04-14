#include <Servo.h>
#include <ros.h>

ros::Nodehandle servos;
ros::Subscriber<std_msgs::Int32> sub("servo", callback);

Servo servoLeft;
Servo servoRight;

int pinLeft = 9;
int pinRight = 10;
int pos;
int turnAngle = 90;

void lowerServo() { //angle in degrees 
  //adds a delay to slow down the servo motion
  for (pos = 0; pos <= turnAngle; pos++) {
    servoLeft.write(pos);
    servoRight.write(pos);
    delay(10);
  }
}

void raiseServo() { //angle in degrees 
  //adds a delay to slow down the servo motion
  for (pos = 0; pos <= turnAngle; pos++) {
    servoLeft.write(turnAngle - pos);
    servoRight.write(turnAngle - pos);
    delay(10);
  }
}

void callback(&msg){ 
  if(msg.data == 1) {
    lowerServo()
  }

  else {
    raiseServo()
  }
}

void setup() {
  servos.initNode();
  servoLeft.attach(pinLeft);
  servoRight.attach(pinRight);
  servos.subscribe(sub)
  
}


void loop() {
  servos.spinOnce();
  delay(10);
}

