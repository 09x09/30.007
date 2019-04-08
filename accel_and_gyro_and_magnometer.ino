
#include <ros.h>
#include <std_msgs/Float32MultiArray.h>

std_msgs::Float32MultiArray imu_msg;
ros::NodeHandle node;
ros::Publisher readings("readings", &imu_msg);
float imu[36]; //[quaternion, orientation cov, angular velocity, angular velocity cov, linear acc, linear acc cov]

#include <Wire.h> // I2C library, gyroscope

#define GYRO 0x68 //  when AD0 is connected to GND ,gyro address is 0x68.
#define G_SMPLRT_DIV 0x15
#define G_DLPF_FS 0x16
#define G_INT_CFG 0x17
#define G_PWR_MGM 0x3E
#define G_TO_READ 8 // 2 bytes for each axis x, y, z

#define SENSOR 0x53 // Device add in which is also included the 8th bit for selectting the mode, read in this case
#define Power_Register 0x2D
#define X_Axis_Register_DATAX0 0x32 // Hexadecima address for the DATAX0 internal register.
#define X_Axis_Register_DATAX1 0x33 // Hexadecima address for the DATAX1 internal register.
#define Y_Axis_Register_DATAY0 0x34
#define Y_Axis_Register_DATAY1 0x35
#define Z_Axis_Register_DATAZ0 0x36
#define Z_Axis_Register_DATAZ1 0x37

#define Magnetometer_mX0 0x03
#define Magnetometer_mX1 0x04
#define Magnetometer_mZ0 0x05
#define Magnetometer_mZ1 0x06
#define Magnetometer_mY0 0x07
#define Magnetometer_mY1 0x08

int mX0, mX1, mX_out;
int mY0, mY1, mY_out;
int mZ0, mZ1, mZ_out;
float heading, headingDegrees, headingFiltered, declination;
float Xm, Ym, Zm;
#define Magnetometer 0x1E //I2C 7bit address of HMC5883

float gyrox[50],gyroy[50],gyroz[50],accx[50],accy[50],accz[50],magx[50],magy[50],magz[50],x,y,z, hz,hx, hy,pitch[50],roll[50],yaw[50];
int offsets[3], DataReturned_x0, DataReturned_x1,DataReturned_y0, DataReturned_y1,DataReturned_z0, DataReturned_z1;

void initGyro(){
 writeTo(GYRO, G_PWR_MGM, 0x00);
 writeTo(GYRO, G_SMPLRT_DIV, 0x07); // EB, 50, 80, 7F, DE, 23, 20, FF
 writeTo(GYRO, G_DLPF_FS, 0x1E); // +/- 2000 dgrs/sec, 1KHz, 1E, 19
 writeTo(GYRO, G_INT_CFG, 0x00);
}

void calibrate(unsigned int totSamples, unsigned int sampleDelayMS,int*xyz){
  
  float tmpOffsets[]={0,0,0};
  for (int i = 0; i<totSamples;i++){
    delay(sampleDelayMS);
    getGyroscopeData(xyz);
    tmpOffsets[0] += xyz[0];
    tmpOffsets[1] += xyz[1];
    tmpOffsets[2] += xyz[2];
    } 
 }
void getGyroscopeData(int * result)
{
 /**************************************
 Gyro ITG-3200 I2C
 registers:
 temp MSB = 1B, temp LSB = 1C
 x axis MSB = 1D, x axis LSB = 1E
 y axis MSB = 1F, y axis LSB = 20
 z axis MSB = 21, z axis LSB = 22
 *************************************/
 int regAddress = 0x1B;
 int temp, x, y, z;
 byte buff[G_TO_READ];
 readFrom(GYRO, regAddress, G_TO_READ, buff); //read the gyro data from the ITG3200
 result[0] = ((buff[2] << 8) | buff[3]);
 result[1] = ((buff[4] << 8) | buff[5]);
 result[2] = ((buff[6] << 8) | buff[7]);
 }
 
void setup(){ 
 Serial.begin(57600);
 Wire.begin();
 node.initNode();
 node.advertise(readings);
 initGyro();
 calibrate(2500,2,offsets);
 writeTo(SENSOR,Power_Register,8);
 writeTo(Magnetometer,0x02,0x00);

}

int i =0;

void loop(){
  if(i == 0){Serial.println(millis());}
 if(i%50==0){
  Serial.println(millis());
  Serial.println("calc stuff now");
    float cy = cos(yaw[49] * 0.5);
    float sy = sin(yaw[49] * 0.5);
    float cp = cos(pitch[49] * 0.5);
    float sp = sin(pitch[49] * 0.5);
    float cr = cos(roll[49] * 0.5);
    float sr = sin(roll[49] * 0.5);

    float w = cy * cp * cr + sy * sp * sr;
    float x = cy * cp * sr - sy * sp * cr;
    float y = sy * cp * sr + cy * sp * cr;
    float z = sy * cp * cr - cy * sp * sr;
  
  //quaternion orientation 
  imu[0] = x;
  imu[1] = y;
  imu[2] = z;
  imu[3] = w;

  //orientation covariance
  imu[4] = variance(roll);
  imu[5] = covariance(roll,pitch);
  imu[6] = covariance(roll,yaw);
  imu[7] = imu[5];
  imu[8] = variance(pitch);
  imu[9] = covariance(pitch, yaw);
  imu[10] = imu[6];
  imu[11] = imu[9];
  imu[12] = variance(yaw);  

  //angular velocity 
  imu[13] = gyrox[49];
  imu[14] = gyroy[49];
  imu[15] = gyroz[49];

  //angular velocity covariance
  imu[16] = variance(gyrox);
  imu[17] = covariance(gyrox,gyroy);
  imu[18] =  covariance(gyrox,gyroz);
  imu[19] = imu[17];
  imu[20] = variance(gyroy);
  imu[21] = covariance(gyroy, gyroz);
  imu[22] = imu[18];
  imu[23] = imu[21];
  imu[24] = variance(gyroz);
    
  //linear acceleration 
  imu[25] = accx[49];
  imu[26] = accy[49];
  imu[27] = accz[49];

  //linear acceleration covariance
  imu[28] = variance(accx);
  imu[29] = covariance(accx,accy);
  imu[30] = covariance(accx,accz);
  imu[31] = imu[29];
  imu[32] = variance(accy);
  imu[33] = covariance(accy, accz);
  imu[34] = imu[30];
  imu[35] = imu[33];
  imu[36] = variance(accz);

  imu_msg.data_length = 37;
  imu_msg.data = imu;
  readings.publish(&imu_msg);
  node.spinOnce();
  i=0;
 }
// Serial.println(i);
 byte addr;
 int gyro[3];
 getGyroscopeData(gyro);
 gyro[0] -= offsets[0];
 gyro[1] -= offsets[1];
 gyro[2] -= offsets[2];
 hx = (gyro[0] / 14.375)*PI/180;
 hy = (gyro[1] / 14.375)*PI/180;
 hz = (gyro[2] / 14.375)*PI/180;
 gyrox[i] = hx;
 gyroy[i] = hy;
 gyroz[i] = hz;

readacc(X_Axis_Register_DATAX0,X_Axis_Register_DATAX1);

  Wire.requestFrom(SENSOR, 2);
  if (Wire.available() <= 2) {
    DataReturned_x0 = Wire.read();
    DataReturned_x1 = Wire.read();}
  readacc(Y_Axis_Register_DATAY0,Y_Axis_Register_DATAY1);
  Wire.requestFrom(SENSOR,2);
  if (Wire.available() <= 2) {
    DataReturned_y0 = Wire.read();
    DataReturned_y1 = Wire.read();}
  readacc(Z_Axis_Register_DATAZ0,Z_Axis_Register_DATAZ1);
  Wire.requestFrom(SENSOR,2);
  if (Wire.available() <= 2) {
    DataReturned_z0 = Wire.read();
    DataReturned_z1 = Wire.read();}

    DataReturned_x1 = DataReturned_x1 << 8;
    x = DataReturned_x0 + DataReturned_x1;

    DataReturned_y1 = DataReturned_y1 <<8;
    y = DataReturned_y0 + DataReturned_y1;
    DataReturned_z1 = DataReturned_z1 <<8;
    z = DataReturned_z0 + DataReturned_z1;
    x = x / 256.0;
    y = y/ 256.0;
    z = z / 256.0;
    accx[i] = x*9.81;
    accy[i] = y*9.81;
    accz[i] = z*9.81;

 // — — X-Axis
 mX0=readmag(Magnetometer,Magnetometer_mX1,1);
 mX1=readmag(Magnetometer,Magnetometer_mX0,1);
 mY0=readmag(Magnetometer,Magnetometer_mY1,1);
 mY1=readmag(Magnetometer,Magnetometer_mY0,1);
 mZ0=readmag(Magnetometer,Magnetometer_mZ1,1);
 mZ1=readmag(Magnetometer,Magnetometer_mZ0,1);

  mX1 = mX1 << 8;
  mX_out = mX0 + mX1; // Raw data
  magx[i] = mX_out * 0.00092; // Gauss unit
  mY1 = mY1 << 8;
  mY_out = mY0 + mY1;
  magx[i] = mY_out * 0.00092;
  // — — Z-Axis
  mZ1 = mZ1 << 8;
  mZ_out = mZ0 + mZ1;
  magz[i] = mZ_out * 0.00092;

 // heading = atan2(magy[i], magx[i]);
 // declination = 0.003782;
 // heading += declination;

 // if (heading < 0) heading += 2 * PI;

//  if (heading > 2 * PI)heading -= 2 * PI;
//headingDegrees = heading * 180 / PI; // The heading in Degrees unit
//headingFiltered = headingFiltered * 0.85 + headingDegrees * 0.15;
float mag_x = magx[i]*cos(pitch[i]) + magy[i]*sin(roll[i])*sin(pitch[i]) + magz[i]*cos(roll[i])*sin(pitch[i]);
float mag_y = magy[i] * cos(roll[i]) - magz[i] * sin(roll[i]);
yaw[i] = 180 * atan2(-mag_y,mag_x)/PI;
pitch[i] = 180 * atan2(accx[i], sqrt(accy[i]*accy[i] + accz[i]*accz[i]))/PI;
roll[i] = 180 * atan2(accy[i], sqrt(accx[i]*accx[i] + accz[i]*accz[i]))/PI;
i+=1;

  
  delay(50);
}
//---------------- Functions
//Writes val to address register on ACC
void writeTo(int DEVICE, byte address, byte val) {
  Wire.beginTransmission(DEVICE); //start transmission to ACC 
  Wire.write(address);        // send register address
  Wire.write(val);        // send value to write
  Wire.endTransmission(); //end transmission
}
//reads num bytes starting from address register on ACC in to buff array
 void readFrom(int DEVICE, byte address, int num, byte buff[]) {
 Wire.beginTransmission(DEVICE); //start transmission to ACC 
 Wire.write(address);        //sends address to read from
 Wire.endTransmission(); //end transmission
 
 Wire.beginTransmission(DEVICE); //start transmission to ACC
 Wire.requestFrom(DEVICE, num);    // request 6 bytes from ACC
 
 int i = 0;
 while(Wire.available())    //ACC may send less than requested (abnormal)
 { 
   buff[i] = Wire.read(); // receive a byte
   i++;
 }
 Wire.endTransmission(); //end transmission
}
void readacc(byte address1, byte address2){
  Wire.beginTransmission(SENSOR);
  Wire.write(address1);
  Wire.write(address2);
  Wire.endTransmission();}
  
byte readmag(int dev, byte adress, int num){
  Wire.beginTransmission(dev); // transmit to device
  Wire.write(adress);
  Wire.endTransmission();
  Wire.requestFrom(dev, 1);
  if (Wire.available() <= 1)
  { return Wire.read();
  }
}

float covariance(float X[], float Y[]){
  float multiple[50];
  float meanmultiple;
  float meanX;
  float meanY;
  float covariance; 
  for(int i=0;i<50;i++){
    meanX += X[i];
    meanY+=Y[i];
   }

  meanX=meanX/50;
  meanY = meanY/50;
  
  for(int j = 0; j < 50; j++) {
    covariance += (X[i] - meanX)*(Y[i] - meanY)/49;
  }
  
  return covariance;
 }
 
float variance(float X[]){
  float mean=0;
  float sum = 0;
  for(int i = 0; i<50;i++){
    mean += X[i];
    sum = sum+ X[i]*X[i]; 
//    Serial.print("Xi: ");
//    Serial.println(X[i],7);
}
  mean = mean/50;
  float variance = (sum-(mean*mean*50))/49;
//  Serial.print("mean: ");
//  Serial.println(mean,7);
//  Serial.print("sum: ");
//  Serial.println(sum,7);
//  Serial.print("variance: ");
//  Serial.println(variance,7);
  return variance;
}
