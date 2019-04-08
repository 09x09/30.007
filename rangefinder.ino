/* This example shows how to use continuous mode to take
range measurements with the VL53L0X. It is based on
vl53l0x_ContinuousRanging_Example.c from the VL53L0X API.

The range readings are in units of mm. */
//#include <ros.h>
//#include <std_msgs/Float32MultiArray.h>
#include <Wire.h>
#include <VL53L0X.h>

//ros::NodeHandle ranger;
//std_msgs::Float32MultiArray range;
//ros::Publisher rangefinder("rangefinder", &range);

VL53L0X left;
VL53L0X right;
void setup()
{
//  ranger.initNode();
//  ranger.advertise(rangefinder);
//  
  Serial.begin(9600);
  
  Wire.begin();
  Serial.println("00");
  left.init(true);
  Serial.println("01");
  left.setAddress((uint8_t)22);
  Serial.println("02");

  right.init(true);
  Serial.println("03");
  right.setAddress((uint8_t)25);
  Serial.println("04");

  Serial.println("addresses set");
  left.setTimeout(500);
  right.setTimeout(500);
  left.startContinuous();
  right.startContinuous();
}

void loop()
{
  int a = left.readRangeContinuousMillimeters();
  int b = right.readRangeContinuousMillimeters();
  Serial.println(a);
  if (left.timeoutOccurred()) { Serial.print(" TIMEOUT"); }
  Serial.println(b);
  if (right.timeoutOccurred()){Serial.print("TIMEOUT");}
//
//  range[0] = left.readRangeContinuousMillimeters();
//  range[1] = right.readRangeContinuousMillimeters();
//  rangefinder.publish( &range);
//  ranger.spinOnce();
  delay(300);
}
