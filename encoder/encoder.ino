#include <ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <math.h>
ros::NodeHandle encoder;
std_msgs::Float32MultiArray arr ;
float arr1 [24];
ros::Publisher encoded("encoded", &arr);

int encoder0PinA_left = 14; //CLK
int encoder0PinB_left = 15; //DT
int encoder0PinA_right = 20; //CLK
int encoder0PinB_right = 21; //DT

float encoder_timing_left[2] = {0,0};
float encoder_timing_right[2] = {0,0};

const int smooth_length = 9;
float smooth_data_left_angular [smooth_length] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
float smooth_data_left_linear [smooth_length] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
float smooth_data_right_angular [smooth_length] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
float smooth_data_right_linear [smooth_length] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
float smooth_decay [smooth_length] = {34, 21, 13, 8, 5, 3, 2, 1, 1};
const float smooth_div = smooth_decay[0] + smooth_decay[1] + smooth_decay[2] + smooth_decay[3] + smooth_decay[4] + smooth_decay[5] + smooth_decay[6] + smooth_decay[7] + smooth_decay[8];

int smooth_pos_left_angular = 0;
int smooth_pos_left_linear = 0;

int smooth_pos_right_angular = 0;
int smooth_pos_right_linear = 0; 

float left_angular;
float left_linear;
float right_angular;
float right_linear;


void setup() 
{ 
  encoder.initNode();
  encoder.advertise(encoded);
  pinMode (encoder0PinA_left,INPUT);
  pinMode (encoder0PinB_left,INPUT);
  pinMode (encoder0PinA_right,INPUT);
  pinMode (encoder0PinB_right,INPUT);
  
  Serial.begin (9600);
  encoder_timing_left[0] = millis()*0.001;
  encoder_timing_left[1] = millis()*0.001;
  
  encoder_timing_right[0] = millis()*0.001;
  encoder_timing_right[1] = millis()*0.001;
  Serial.begin(57600);
}

bool enc_a_left [2] = {false, false};
bool enc_b_left = false;

int enc_state_left() {
  enc_a_left[1] = enc_a_left[0];
  enc_b_left;
  
  enc_a_left[0] = digitalRead(encoder0PinA_left);
  enc_b_left = digitalRead(encoder0PinB_left);

  if (!enc_a_left[1] && enc_a_left[0]) {
    return enc_b_left == 0 ? -1 : 1;
  }
  return 0;
}

bool enc_a_right [2] = {false, false};
bool enc_b_right = false;

int enc_state_right() {
  enc_a_right[1] = enc_a_right[0];
  enc_b_right;
  
  enc_a_right[0] = digitalRead(encoder0PinA_right);
  enc_b_right = digitalRead(encoder0PinB_right);

  if (!enc_a_right[1] && enc_a_right[0]) {
    return -1*(enc_b_right == 0 ? -1 : 1);  
  }
  return 0;
}

float angle = PI/12;

int es_hist_left [5] = {1, 1, 1, 1, 1}; 
int es_hist_right [5] = {1, 1, 1, 1, 1}; 

void es_push_left(int x) {
  for (int i = 5; i > 0; i--) {
    es_hist_left[i] = es_hist_left[i -1];
  }
  es_hist_left[0] = x;
}

void es_push_right(int x) {
  for (int i = 5; i > 0; i--) {
    es_hist_right[i] = es_hist_right[i -1];
  }
  es_hist_right[0] = x;
}

void es_unshift_left() {
  for (int i = 0; i < 4; i++) {
    es_hist_left[i] = es_hist_left[i + 1];
  }
}

void es_unshift_right() {
  for (int i = 0; i < 4; i++) {
    es_hist_right[i] = es_hist_right[i + 1];
  }
}

void loop() {   
  int es_left = enc_state_left();
  int es_right = enc_state_right();
  float vl, vr, vc, wc, nextAngle, newPosX, newPosY; 
  if (es_left != 0) {
    es_push_left(es_left);
    if ((es_hist_left[2] == es_hist_left[0]) && (es_hist_left[1] != es_hist_left[0])) {
      es_hist_left[1] = es_hist_left[0];
      es_unshift_left();
      
    } else {
      encoder_timing_left[0] = encoder_timing_left[1];
      encoder_timing_left[1] = millis()*0.001;
  
      float diff_tm = encoder_timing_left[1] - encoder_timing_left[0];
      float left_angvel = (es_hist_left[1] * angle) / diff_tm;
      float left_linvel = 0.05 * (es_hist_left[1] * angle) / diff_tm;
      //left_angular = weighted_moving_average_left_angular(left_angvel);
      left_linear = weighted_moving_average_left_linear(left_linvel);
      vl = left_linear;
      Serial.println(left_linear);
    }
  }
   if (es_right != 0) {
    es_push_right(es_right);
    if ((es_hist_right[2] == es_hist_right[0]) && (es_hist_right[1] != es_hist_right[0])) {
      es_hist_right[1] = es_hist_right[0];
      es_unshift_right();
      
    } else {
      encoder_timing_right[0] = encoder_timing_right[1];
      encoder_timing_right[1] = millis()*0.001;
      float diff_tm_right = encoder_timing_right[1] - encoder_timing_right[0];
      float right_angvel = (es_hist_right[1] * angle) / diff_tm_right;
      float right_linvel = 0.05 * (es_hist_right[1] * angle) / diff_tm_right;
      //right_angular = weighted_moving_average_right_angular(right_angvel);
      right_linear = weighted_moving_average_right_linear(right_linvel);
      vr = right_linear;
      Serial.println(right_linear);
    }
  }

    arr1[0] = vl;
    arr1[1] = vr;    
    arr.data_length = 2;
    arr.data = arr1;
    encoded.publish( &arr);
    encoder.spinOnce();
  //delay(30);
}


float weighted_moving_average_left_angular(float reading) {
  smooth_data_left_angular[smooth_pos_left_angular] = reading;
  float wma = 0;
  for (int i = 0; i < smooth_length; i++) {
    wma += smooth_data_left_angular[(i + smooth_pos_left_angular) % smooth_length] * smooth_decay[i];
  }
  wma = wma / smooth_div;
  smooth_pos_left_angular += 8;
  smooth_pos_left_angular %= smooth_length;
  return wma;
}

float weighted_moving_average_left_linear(float reading) {
  smooth_data_left_linear[smooth_pos_left_linear] = reading;
  float wma = 0;
  for (int i = 0; i < smooth_length; i++) {
    wma += smooth_data_left_linear[(i + smooth_pos_left_linear) % smooth_length] * smooth_decay[i];
  }
  wma = wma / smooth_div;
  smooth_pos_left_linear += 8;
  smooth_pos_left_linear %= smooth_length;
  return wma;
}

float weighted_moving_average_right_angular(float reading) {
  smooth_data_right_angular[smooth_pos_right_angular] = reading;
  float wma = 0;
  for (int i = 0; i < smooth_length; i++) {
    wma += smooth_data_right_angular[(i + smooth_pos_right_angular) % smooth_length] * smooth_decay[i];
  }
  wma = wma / smooth_div;
  smooth_pos_right_angular += 8;
  smooth_pos_right_angular %= smooth_length;
  return wma;
}

float weighted_moving_average_right_linear(float reading) {
  smooth_data_right_linear[smooth_pos_right_linear] = reading;
  float wma = 0;
  for (int i = 0; i < smooth_length; i++) {
    wma += smooth_data_right_linear[(i + smooth_pos_right_linear) % smooth_length] * smooth_decay[i];
  }
  wma = wma / smooth_div;
  smooth_pos_right_linear += 8;
  smooth_pos_right_linear %= smooth_length;
  return wma;
}

float covariance(float X[], float Y[]){
  float multiple[50];
  float meanmultiple;
  float meanX;
  float meanY;
  float covariance; 
  for(int i=0;i<30;i++){
    meanX += X[i];
    meanY+=Y[i];
   }

  meanX=meanX/30;
  meanY = meanY/30;
  
  for(int j = 0; j < 50; j++) {
    covariance += (X[j] - meanX)*(Y[j] - meanY)/29;
  }
  
  return covariance;
 }
 
float variance(float X[]){
  float mean=0;
  float sum = 0;
  for(int i = 0; i<30;i++){
    mean += X[i];
    sum = sum+ X[i]*X[i]; 
//    Serial.print("Xi: ");
//    Serial.println(X[i],7);
}
  mean = mean/30;
  float variance = (sum-(mean*mean*50))/29;
//  Serial.print("mean: ");
//  Serial.println(mean,7);
//  Serial.print("sum: ");
//  Serial.println(sum,7);
//  Serial.print("variance: ");
//  Serial.println(variance,7);
  return variance;
}
