#include <ros.h>

#include <C:\Users\asus\Desktop\Design\motor_movement\Wheels.h>

void messageCb( const std_msgs::Wheels & msg ){
  int left_speed = round( ( msg.left / max_speed ) * 255 );
  int right_speed = round( ( msg.right / max_speed ) * 255 );  
  if( left_speed > right_speed ){
    turn_right( left_speed, right_speed );
  }
  elif( left_speed < right_speed ){
    turn_left( left_speed, right_speed );
  }
  elif( left_speed = right_speed){
    if( left_speed < 0 ){
      backward( left_speed * -1, right_speed * -1 );
    }
    else{
      forward( left_speed, right_speed);
    }
  }
}
ros::NodeHandle nh;
ros::Subscriber<std_msgs::Wheels> sub("cmd_vel", &messageCb );

//MOTOR 1
int E1 = 5;
int M1 = 4;
 
//MOTOR 2
int E2 = 6;
int M2 = 7;

//maximum speed of wheel
int max_speed = 255;


void forward( left, right ){
  analogWrite( E1, left );
  digitalWrite( M1, HIGH );
  analogWrite( E2, left );
  digitalWrite( M2, HIGH );
  
}

void backward( left, right ){
  analogWrite( E1, left );
  digitalWrite( M1, LOW );
  analogWrite( E2, left );
  digitalWrite( M2, LOW );
}

void turn_right( left, right ){
  analogWrite( E1, left);
  digitalWrite(M1, HIGH);
  analogWrite( E2, left);
  digitalWrite(M2, LOW);
}

void turn_left( left, right ){
  analogWrite( E1, left);
  digitalWrite(M1, LOW);
  analogWrite( E2, left);
  digitalWrite(M2, HIGH);;
}

void stop( void ){
  digitalWrite( E1, LOW );
  digitalWrite( E2, LOW );
  }
}


void setup() {
  pinMode(E1, OUTPUT);
  pinMode(E2, OUTPUT);
  pinMode(M1, OUTPUT);
  pinMode(M2, OUTPUT);
  nh.initNode();
  nh.subscribe(sub);
}

void loop() {
  nh.spin();
}
