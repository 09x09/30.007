//Libraries to include
#include "src/Encoder/Encoder.h"
#include "src/Encoder/MovingAverage.h"
#include "src/Imu/Imu.h"

#include <Wire.h>
#include <VL53L0X.h>

#include "src/Motor/Motor.h"
#include <Servo.h>
#include <SPI.h>
#include <MFRC522.h>
#include <ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Float64.h>

#include <trolley/Wheels.h>

//Motor stuff
Motor MotRight(6, 7, 8, 9);
Motor MotLeft(10, 11, 12, 13);

////Servo stuff
Servo servoLeft;
Servo servoRight;
int pinLeft = 28;
int pinRight = 5;
const int turnAngle = 90;

ros::NodeHandle trobot;

//Servo movement

// callback for servo
 void callback(const std_msgs::Float64 &msg)
 {
   trobot.loginfo("callback");
   if (msg.data == 0)
   {
      servoLeft.write(90);
      servoRight.write(0);
      trobot.loginfo("done");
   }

   if (msg.data == 1)
   {
      servoLeft.write(0);
      servoRight.write(90);
      trobot.loginfo("done");
   }
   
 }

//callback for motors
 void messageCb(const trolley::Wheels &msg)
 {
   int val_right = msg.right * (92 / 0.205) * 0.8933 + 9;
   int val_left = msg.left * (92 / 0.205) * 0.8933 + 9;
   
   if (msg.left < 0 && msg.right < 0) //reverse
   {
     val_right = -1 * val_right;
     val_left = -1 * val_left;
     MotRight.move(0, val_right);
     MotLeft.move(0, val_left);
   }

   if (msg.left > 0 && msg.right > 0) //forward
   {
     MotRight.move(val_right, 0);
     MotLeft.move(val_left, 0);
   }
   
   if (msg.left == 0 && msg.right == 0){
    MotRight.stop();
    MotLeft.stop();
   }
   
   if (msg.left > 0 && msg.right < 0)
   {
     val_right = -1 * val_right;
     MotRight.move(0, val_right);
     MotLeft.move(val_left, 0);
   }
   
   if (msg.left < 0 && msg.right > 0){
     val_left = -1 * val_left;
     MotRight.move(val_right, 0);
     MotLeft.move(0, val_left);
   }
 }
//ROS STUFF
////subscribers
 ros::Subscriber<std_msgs::Float64> servo("servo", &callback);
 ros::Subscriber<trolley::Wheels> cmd_vel("cmd_vel", &messageCb);

//publishers
 std_msgs::Float32MultiArray enc;
 std_msgs::Float32MultiArray imu_msg;
 std_msgs::Int32MultiArray range;
 std_msgs::Int32 rfid_msg;
ros::Publisher encoder("encoded", &enc);
ros::Publisher imuu("readings", &imu_msg);
ros::Publisher rangefinder("rangefinder", &range);
ros::Publisher rfidread("rfidread", &rfid_msg);

////Encoder stuff
struct EncoderPin
{
  int dt;
  int clk;
  EncoderPin(int dt, int clk) : dt(dt), clk(clk){};
};

const EncoderPin right(2, 3);
const EncoderPin left(31, 30);
const int samples = 10;
const int threshold = 3;
const float step_angle = PI / 12;
const float linear_dist = 0.05;

MovingAverage smooth_left(10, 1.7);
MovingAverage smooth_right(8, 2);
Encoder wheel_left(left.dt, left.clk, 3, 2500000, step_angle, linear_dist);
Encoder wheel_right(right.dt, right.clk, 3, 2500000, step_angle, linear_dist);
bool print_once_stop_flag = true;
bool print_right = true;
float linear_vel[2];


//IMU STUFF
int imu_counter = 0;
Imu emu;
unsigned long thrott = 0;
long last = 0;

//RANGEFINDER STUFF
VL53L0X left_side;
VL53L0X right_side;
 int Rleft = 27;
int Rright = 46;
long int distance[2];
//Time b4 entering main loop
long gtime = 0;


//RFID READER STUFF
MFRC522 rfid(53, 29); // Instance of the class

MFRC522::MIFARE_Key key;

// Init array that will store new NUID
byte nuidPICC[4];

void setup()
{
  //INITIALISE ROS COMMS
   trobot.initNode();
   trobot.advertise(rangefinder);
   trobot.advertise(imuu);
   trobot.advertise(encoder);
   trobot.advertise(rfidread);
   trobot.subscribe(servo);
   trobot.subscribe(cmd_vel);

  Serial.begin(57600);

  ////INITIALISE IMU
   emu.init();

  //INITIALISE RANGEFINDERS
  pinMode(Rleft, OUTPUT);
  pinMode(Rright, OUTPUT);
  digitalWrite(Rleft, LOW);
  digitalWrite(Rright, LOW);

  pinMode(Rleft, INPUT);
  left_side.init(true);
  left_side.setAddress((uint8_t)22);
  pinMode(Rright, INPUT);
  right_side.init(true);
  right_side.setAddress((uint8_t)25);

  left_side.setTimeout(50000);
  right_side.setTimeout(50000);
  left_side.startContinuous();
  right_side.startContinuous();

  ////INITIALISE ENCODER
  pinMode(left.dt, INPUT);
  pinMode(left.clk, INPUT);
  pinMode(right.dt, INPUT);
  pinMode(right.clk, INPUT);


  ////INITIALISE RFID READER
  SPI.begin();     // Init SPI bus
  rfid.PCD_Init(); // Init MFRC522
  for (byte i = 0; i < 6; i++)
  {
    key.keyByte[i] = 0xFF;
  }

  ////INTIALISE MOTOR
  MotRight.init();
  MotLeft.init();
  gtime = millis();

  ////INITIALISE SERVOS;
  servoLeft.attach(pinLeft);
  servoRight.attach(pinRight);
  servoLeft.write(90);
  servoRight.write(0);
  gtime = millis();
};

void loop()
{
 

  if (wheel_right.sample_encoder())
  {
    float rightenc = -1 * smooth_right.add_sample(wheel_right.lin_vel());
    if (rightenc < 3 && rightenc > -3){
      linear_vel[1] = rightenc;
    }
    print_right = true;
  }
  
  if (wheel_right.stopped() && print_right)
  {
    linear_vel[1] = 0;
    print_right = false;
  }
  
  if (wheel_left.sample_encoder())
  {
    float leftenc = smooth_left.add_sample(wheel_left.lin_vel());
    if(leftenc < 3 && leftenc > -3){
    linear_vel[0] = leftenc; 
    }
    print_once_stop_flag = true;
  }
  
  if (wheel_left.stopped() && print_once_stop_flag)
  {
    linear_vel[0] = 0;
    print_once_stop_flag = false;
  }
  
   enc.data_length = 2;
   enc.data = linear_vel;
   encoder.publish( &enc);
   trobot.spinOnce();

  ////READ ENCODER
    if (millis() > thrott + 15)
   {
     thrott = millis();

      //READ IMU
     emu.GyroCalc(imu_counter);
     emu.AccCalc(imu_counter);
     emu.MagCalc(imu_counter);

  //   // Serial.println("imu reading :");
//      Serial.println(emu.pitch[imu_counter]);
  //   //  Serial.println(emu.roll[imu_counter]);
  //   // Serial.println(emu.yaw[imu_counter]);
     imu_counter++;

  // //   //READ RANGEFINDER
     distance[0] = right_side.readRangeContinuousMillimeters();
     distance[1] = left_side.readRangeContinuousMillimeters();

       Serial.println("left: ");
       Serial.println(distance[1]);
       if (left_side.timeoutOccurred())
       {
         Serial.print(" TIMEOUT");
       }
       Serial.println("right: ");
       Serial.println(distance[0]);
       if (right_side.timeoutOccurred())
       {
         Serial.print("TIMEOUT");
       }

      // Publish rangefinder readings
       range.data_length = 2;
       range.data = distance;
       rangefinder.publish( &range);
       trobot.spinOnce();
   }

   if (imu_counter >= 10)
   {
     imu_counter = 0;
     emu.quaternion();
     float imu[37];
     emu.matrixCalc(imu);


//     //Publish imu readings
      imu_msg.data_length = 37;
      imu_msg.data = imu;
      imuu.publish(&imu_msg);
      trobot.spinOnce();
   }
   ////READ RFID
   // Look for new cards
   if (!rfid.PICC_IsNewCardPresent())
     return;

//   // Verify if the NUID has been readed
   if (!rfid.PICC_ReadCardSerial())
     return;

   if (rfid.uid.uidByte[0] != nuidPICC[0] ||
       rfid.uid.uidByte[1] != nuidPICC[1] ||
       rfid.uid.uidByte[2] != nuidPICC[2] ||
       rfid.uid.uidByte[3] != nuidPICC[3])
   {

//     // Store NUID into nuidPICC array
     for (byte i = 0; i < 4; i++)
     {
       nuidPICC[i] = rfid.uid.uidByte[i];

     }
     int nu = printDec(rfid.uid.uidByte, rfid.uid.size);

//     //// Publish rfid readings
  rfid_msg.data = nu;
      rfidread.publish( &rfid_msg );
      trobot.spinOnce();
     Serial.println(nu);
   }
   else
     Serial.println(F("Card read previously."));

//   // Halt PICC
   rfid.PICC_HaltA();

//   // Stop encryption on PCD
   rfid.PCD_StopCrypto1();

  trobot.spinOnce();

}
//Endloop

//Decode rfid
int printDec(byte *buffer, byte bufferSize)
{
  int number;
  for (byte i = 0; i < bufferSize; i++)
  {
    number += buffer[i];
  }

      return number;
  
}
