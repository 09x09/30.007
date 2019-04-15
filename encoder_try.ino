// #include <ros.h>
// #include <std_msgs/Float32MultiArray.h>
#include "src/Encoder/Encoder.h"
#include "src/Encoder/MovingAverage.h"
#include "src/imu.h"
// ros::NodeHandle encoder;
// std_msgs::Float32MultiArray arr ;
// ros::Publisher encoded("encoded", &arr);

struct EncoderPin {
  int dt;
  int clk;
  EncoderPin(int dt, int clk) : dt(dt), clk(clk) {};
};

// DT, CLK
const EncoderPin left(2, 3);

const int samples = 10;
const int threshold = 3;
const float step_angle = PI/12;
const float linear_dist = 0.05;

MovingAverage smooth_left(8, 2);
Encoder wheel_left(left.dt, left.clk, 3, 250000, step_angle, linear_dist);
int imu_counter = 0;
Imu emu;
void setup() { 
  // encoder.initNode();
  // encoder.advertise(encoded);

  pinMode(left.dt,  INPUT);
  pinMode(left.clk,  INPUT);
  
  Serial.begin (115200);
  emu.init();
};
long last = 0;
bool print_once_stop_flag = true;
unsigned long thrott = 0;
void loop() {
  float a;
  if (wheel_left.sample_encoder()) {
    Serial.print("Wheel Left: ");
    a = smooth_left.add_sample(wheel_left.lin_vel());
    Serial.print(a);
    Serial.print(", ");
    Serial.print(wheel_left.raw_tick().data.state);
    Serial.println("");
    print_once_stop_flag = true;
   
  }
  if (wheel_left.stopped() && print_once_stop_flag) {
    Serial.println("Stopped");
    // print_once_stop_flag = false;
  }
  // delay(10);
  if (millis() > thrott + 15) {
  emu.GyroCalc(imu_counter);
  emu.AccCalc(imu_counter);
  emu.MagCalc(imu_counter);
  imu_counter ++;
  thrott = millis();
  }
// //Publish imu readings
  if(imu_counter >= 10){
  Serial.print("this: ");
  Serial.println(micros() - last);
  last = micros();
    // Serial.println(micros());
    imu_counter = 0;
    float imu[37];
    emu.quaternion();
    imu[0] = emu.quat[0];
    imu[1] = emu.quat[1];
    imu[2] = emu.quat[2];
    imu[3] = emu.quat[3];

    //orientation covariance
    imu[4] = emu.variance(emu.roll);

    imu[5] = emu.covariance(emu.roll,emu.pitch);

    imu[6] = emu.covariance(emu.roll,emu.yaw);
    imu[7] = imu[5];
    imu[8] = emu.variance(emu.pitch);
    imu[9] = emu.covariance(emu.pitch, emu.yaw);
    imu[10] = imu[6];
    imu[11] = imu[9];
    imu[12] = emu.variance(emu.yaw);  
//  
//    //angular velocity 
    imu[13] = emu.gyrox[9];
    imu[14] = emu.gyroy[9];
    imu[15] = emu.gyroz[9];
  
//    //angular velocity covariance
    imu[16] = emu.variance(emu.gyrox);
    imu[17] = emu.covariance(emu.gyrox,emu.gyroy);
    imu[18] =  emu.covariance(emu.gyrox,emu.gyroz);
    imu[19] = imu[17];
    imu[20] = emu.variance(emu.gyroy);
    imu[21] = emu.covariance(emu.gyroy, emu.gyroz);
    imu[22] = imu[18];
    imu[23] = imu[21];
    imu[24] = emu.variance(emu.gyroz);
//      
//    //linear acceleration 
    imu[25] = emu.accx[9];
    imu[26] = emu.accy[9];
    imu[27] = emu.accz[9];
//  
//    //linear acceleration covariance
    imu[28] = emu.variance(emu.accx);
    imu[29] = emu.covariance(emu.accx,emu.accy);
    imu[30] = emu.covariance(emu.accx,emu.accz);
    imu[31] = imu[29];
    imu[32] = emu.variance(emu.accy);
    imu[33] = emu.covariance(emu.accy, emu.accz);
    imu[34] = imu[30];
    imu[35] = imu[33];
    imu[36] = emu.variance(emu.accz);
// //  
// ////    imu_msg.data_length = 37;
// ////    imu_msg.data = imu;
// ////    readings.publish(&imu_msg);
// ////    encoder.spinOnce();

// //  
  }
//     // encoded.publish(&arr);
//     // encoder.spinOnce();
//     // next_skip += skip;
//   // }
};
