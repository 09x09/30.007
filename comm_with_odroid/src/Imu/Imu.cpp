
#include <Wire.h> // I2C library, gyroscope

#include "Imu.h"
#include "Arduino.h"

Imu::Imu(){};

void Imu::init()
{
  Wire.begin();
  writeTo(GYRO, G_PWR_MGM, 0x00);
  writeTo(GYRO, G_SMPLRT_DIV, 0x07); // EB, 50, 80, 7F, DE, 23, 20, FF
  writeTo(GYRO, G_DLPF_FS, 0x1E);    // +/- 2000 dgrs/sec, 1KHz, 1E, 19
  writeTo(GYRO, G_INT_CFG, 0x00);
  Serial.println("start calb") ;
    float tmpOffsets[3]={0,0,0};
    
    for (int i = 0; i< 2000;i++){
      // Serial.println("oij");
    delay(2);
    getGyroscopeData(offsets);
    tmpOffsets[0] += offsets[0];
    tmpOffsets[1] += offsets[1];
    tmpOffsets[2] += offsets[2];
    } 
    
  // Serial.println(tmpOffsets[0]);
  // Serial.println(millis());
  writeTo(acc_add, Power_Register, 8);
  for (int i = 1; i <= 100; i++){
    accoff[0] += readacc(acc_add, X_Axis_Register_DATAX0, X_Axis_Register_DATAX1, 2);
    accoff[1] += readacc(acc_add, Y_Axis_Register_DATAY0, Y_Axis_Register_DATAY1, 2);
    accoff[2] += readacc(acc_add, Z_Axis_Register_DATAZ0, Z_Axis_Register_DATAZ1, 2);
    }
    
  accoff[0] /=100;   
  accoff[1] /=100;
  accoff[2] /=100;
  
  writeTo(mag_add, 0x0B, 0x01);
  setMode(Mode_Continuous, ODR_200Hz, RNG_8G, OSR_512);
  Serial.println(millis());
};

void Imu::writeTo(int DEVICE, byte address, byte val)
{
  Wire.beginTransmission(DEVICE); //start transmission to ACC
  Wire.write(address);            // send register address
  Wire.write(val);                // send value to write
  Wire.endTransmission();         //end transmission
};

void Imu::quaternion()
{

  float cy = cos(yaw[9] * 0.5);
  float sy = sin(yaw[9] * 0.5);
  float cp = cos(pitch[9] * 0.5);
  float sp = sin(pitch[9] * 0.5);
  float cr = cos(roll[9] * 0.5);
  float sr = sin(roll[9] * 0.5);

  quat[0] = cy * cp * cr + sy * sp * sr;
  quat[1] = cy * cp * sr - sy * sp * cr;
  quat[2] = sy * cp * sr + cy * sp * cr;
  quat[3] = sy * cp * cr - cy * sp * sr;
};

void Imu::setMode(uint16_t mode, uint16_t odr, uint16_t rng, uint16_t osr)
{
  writeTo(mag_add, 0x09, mode | odr | rng | osr);
};

void Imu::readMag()
{
  Wire.beginTransmission(mag_add);
  Wire.write(0x00);
  Wire.endTransmission();
  Wire.requestFrom(mag_add, 6);
  int x = Wire.read();   //LSB  x
  x |= Wire.read() << 8; //MSB  x

  int y = Wire.read();   //LSB  z
  y |= Wire.read() << 8; //MSB z
  int z = Wire.read();   //LSB y
  z |= Wire.read() << 8; //MSB y
  mx = x;
  mx /= 3000;
  my = y;
  my /= 3000;
  mz = z;
  mz /= 3000;
  //  Serial.println(mx,7);
  //  Serial.println(my,7);
  //  Serial.println(mz,7);
};

void Imu::MagCalc(int imu_counter)
{

  readMag();

  pitch[imu_counter] = atan2(accx[imu_counter], sqrt(accy[imu_counter] * accy[imu_counter] + accz[imu_counter] * accz[imu_counter]));
  roll[imu_counter] = atan2(accy[imu_counter], sqrt(accx[imu_counter] * accx[imu_counter] + accz[imu_counter] * accz[imu_counter]));
  float mag_x = mx * cos(pitch[imu_counter]) + my * sin(roll[imu_counter]) * sin(pitch[imu_counter]) + mz * cos(roll[imu_counter]) * sin(pitch[imu_counter]);
  //  Serial.println(mag_x);
  float mag_y = my * cos(roll[imu_counter]) - mz * sin(roll[imu_counter]);
  //  Serial.println(mag_y);
  yaw[imu_counter] = (atan2(my, mx))*180/PI;
};

//ACCELEROMETER STUFF
float Imu::readacc(uint8_t address, byte register1, byte register2, int num)
{
  Wire.beginTransmission(address); // Begin transmission to the Sensor
  //Ask the particular registers for data
  Wire.write(register1);
  Wire.write(register2);
  Wire.endTransmission();         // Ends the transmission and transmits the data from the two registers
  Wire.requestFrom(address, num); // Request the transmitted two bytes from the two registers
  if (Wire.available() <= num)
  {                      //
    int a = Wire.read(); // Reads the data from the register
    int b = Wire.read();
    /* Converting the raw data of the X-Axis into X-Axis Acceleration
      - The output data is Two's complement
      - X0 as the least significant byte
      - X1 as the most significant byte */
    b = b << 8;

    float output = 9.81 * (a + b) / 256.0;

    return output; // Xa = output value from -1 to +1, Gravity acceleration acting on the X-Axis
  }
};

void Imu::AccCalc(int imu_counter)
{
  accx[imu_counter] = readacc(acc_add, X_Axis_Register_DATAX0, X_Axis_Register_DATAX1, 2)-accoff[0];
  accy[imu_counter] = readacc(acc_add, Y_Axis_Register_DATAY0, Y_Axis_Register_DATAY1, 2)-accoff[1];
  accz[imu_counter] = readacc(acc_add, Z_Axis_Register_DATAZ0, Z_Axis_Register_DATAZ1, 2)-accoff[2];
};

// //GYRO STUFF
void Imu::readGyro(int DEVICE, byte address, int num, byte buff[])
{
  Wire.beginTransmission(DEVICE); //start transmission to ACC
  Wire.write(address);            //sends address to read from
  Wire.endTransmission();         //end transmission

  Wire.beginTransmission(DEVICE); //start transmission to ACC
  Wire.requestFrom(DEVICE, num);  // request 6 bytes from ACC

  int i = 0;
  while (Wire.available()) //ACC may send less than requested (abnormal)
  {
    buff[i] = Wire.read(); // receive a byte
    i++;
  }
  Wire.endTransmission(); //end transmission
};

void Imu::getGyroscopeData(int result[3])
{
  int regAddress = 0x1B;
  int temp, x, y, z;
  byte buff[G_TO_READ];
  readGyro(GYRO, regAddress, G_TO_READ, buff); //read the gyro data from the ITG3200
  result[0] = ((buff[2] << 8) | buff[3]);
  result[1] = ((buff[4] << 8) | buff[5]);
  result[2] = ((buff[6] << 8) | buff[7]);
};

void Imu::GyroCalc(int imu_counter)
{

  int gyro[3];
  getGyroscopeData(gyro);
  gyrox[imu_counter] = ((gyro[0] - offsets[0]) / 14.375) * PI / 180;
  gyroy[imu_counter] = ((gyro[1] - offsets[1]) / 14.375) * PI / 180;
  gyroz[imu_counter] = ((gyro[2] - offsets[2]) / 14.375) * PI / 180;
};

float Imu::covariance(float X[], float Y[])
{
  float multiple[length];
  float meanmultiple;
  float meanX;
  float meanY;
  for (int i = 0; i < length; i++)
  {
    multiple[i] = X[i] * Y[i];
    meanmultiple += multiple[i];
    meanX += X[i];
    meanY += Y[i];
  }
  meanmultiple = meanmultiple / length;
  meanX = meanX / length;
  meanY = meanY / length;
  float covariance = meanmultiple - meanX * meanY;
  return covariance;
};

float Imu::variance(float X[])
{
  float mean = 0;
  float sum = 0;
  for (int i = 0; i < length; i++)
  {
    mean += X[i];
    sum = sum + X[i] * X[i];
  };
  mean = mean / length;
  float variance = (sum - (mean * mean * length)) / (length - 1);

  return variance;
};

float Imu::matrixCalc(float matrix[])
{
  matrix[0] = quat[0];
  matrix[1] = quat[1];
  matrix[2] = quat[2];
  matrix[3] = quat[3];

  //orientation covariance
  matrix[4] = variance(roll);

  matrix[5] = covariance(roll, pitch);

  matrix[6] = covariance(roll, yaw);
  matrix[7] = matrix[5];
  matrix[8] = variance(pitch);
  matrix[9] = covariance(pitch, yaw);
  matrix[10] = matrix[6];
  matrix[11] = matrix[9];
  matrix[12] = variance(yaw);
  //
  //    //angular velocity
  matrix[13] = gyrox[length - 1];
  matrix[14] = gyroy[length - 1];
  matrix[15] = gyroz[length - 1];

  //    //angular velocity covariance
  matrix[16] = variance(gyrox);
  matrix[17] = covariance(gyrox, gyroy);
  matrix[18] = covariance(gyrox, gyroz);
  matrix[19] = matrix[17];
  matrix[20] = variance(gyroy);
  matrix[21] = covariance(gyroy, gyroz);
  matrix[22] = matrix[18];
  matrix[23] = matrix[21];
  matrix[24] = variance(gyroz);
  //
  //    //linear acceleration
  matrix[25] = accx[length - 1];
  matrix[26] = accy[length - 1];
  matrix[27] = accz[length - 1];
  //
  //    //linear acceleration covariance
  matrix[28] = variance(accx);
  matrix[29] = covariance(accx, accy);
  matrix[30] = covariance(accx, accz);
  matrix[31] = matrix[29];
  matrix[32] = variance(accy);
  matrix[33] = covariance(accy, accz);
  matrix[34] = matrix[30];
  matrix[35] = matrix[33];
  matrix[36] = variance(accz);
};