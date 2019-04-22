#ifndef Emu
#define Emu

#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include "Wire.h"

#define GYRO 0x68 //  when AD0 is connected to GND ,gyro address is 0x68.
#define G_SMPLRT_DIV 0x15
#define G_DLPF_FS 0x16
#define G_INT_CFG 0x17
#define G_PWR_MGM 0x3E
#define G_TO_READ 8 // 2 bytes for each axis x, y, z

//ACCELEROMETER PINS
#define ADXAddress 0x53 // Device add in which is also included the 8th bit for selectting the mode, read in this case
#define Power_Register 0x2D
#define X_Axis_Register_DATAX0 0x32 // Hexadecima address for the DATAX0 internal register.
#define X_Axis_Register_DATAX1 0x33 // Hexadecima address for the DATAX1 internal register.
#define Y_Axis_Register_DATAY0 0x34
#define Y_Axis_Register_DATAY1 0x35
#define Z_Axis_Register_DATAZ0 0x36
#define Z_Axis_Register_DATAZ1 0x37

//MAGNETOMETER PINS
#define QMC5883L_ADDR 0x0D
#define Mode_Standby 0b00000000
#define Mode_Continuous 0b00000001

#define ODR_10Hz 0b00000000
#define ODR_50Hz 0b00000100
#define ODR_100Hz 0b00001000
#define ODR_200Hz 0b00001100

#define RNG_2G 0b00000000
#define RNG_8G 0b00010000

#define OSR_512 0b00000000
#define OSR_256 0b01000000
#define OSR_128 0b10000000
#define OSR_64 0b11000000

class Imu
{
private:
  uint8_t mag_add = QMC5883L_ADDR;
  void writeTo(int DEVICE, byte address, byte val);
  void readGyro(int DEVICE, byte address, int num, byte buff[]);

  uint8_t acc_add = ADXAddress;
  float readacc(uint8_t address, byte register1, byte register2, int num);

  void readMag();
  void getGyroscopeData(int result[3]);
  int length = 10;
  

public:
  Imu();
  float gyrox[10] = {0};
  float gyroy[10] = {0};
  float gyroz[10] = {0};
  float accx[10] = {0};
  float accy[10] = {0};
  float accz[10] = {0};
  float pitch[10] = {0};
  float yaw[10] = {0};
  float roll[10] = {0};
  float quat[4] = {0};
  float mx = 0;
  float my = 0;
  float mz = 0;
  int offsets[3] = {0};
  float accoff[3] = {0};
  void init();
  void setMode(uint16_t mode, uint16_t odr, uint16_t rng, uint16_t osr);
  float variance(float X[]);
  float covariance(float X[], float Y[]);
  void GyroCalc(int imu_counter);
  void AccCalc(int imu_counter);
  void MagCalc(int imu_counter);
  void quaternion();
  float matrixCalc(float matrix[]);
 
};
#endif