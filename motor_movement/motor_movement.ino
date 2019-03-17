#include <ros.h>
#include <trolley/Wheels.h>
#include <Arduino.h>





//MOTOR 1

int E1 = 5;

int M1 = 4;

 

//MOTOR 2

int E2 = 6;

int M2 = 7;



//maximum speed of wheel

int max_speed = 255;

ros::NodeHandle nh;


void messageCb( const trolley::Wheels & msg ){

  int left_speed = round( ( msg.left / max_speed ) * 255 );

  int right_speed = round( ( msg.right / max_speed ) * 255 );  

  if( left_speed > right_speed ){

    turn_right( left_speed, right_speed );

  }

  else if( left_speed < right_speed ){

    turn_left( left_speed, right_speed );

  }

  else if( left_speed = right_speed){

    if( left_speed < 0 ){

      backward( left_speed * -1, right_speed * -1 );

    }

    else{

      forward( left_speed, right_speed);

    }

  }

}



ros::Subscriber<trolley::Wheels> sub("cmd_vel", &messageCb );



void forward( int left, int right ){

  analogWrite( E1, left );

  digitalWrite( M1, HIGH );

  analogWrite( E2, right );

  digitalWrite( M2, HIGH );

  

}



void backward( int left, int right ){

  analogWrite( E1, left );

  digitalWrite( M1, LOW );

  analogWrite( E2, right );

  digitalWrite( M2, LOW );

}



void turn_right( int left, int right ){

  analogWrite( E1, left);

  digitalWrite(M1, HIGH);

  analogWrite( E2, right);

  digitalWrite(M2, LOW);

}



void turn_left( int left, int right ){

  analogWrite( E1, left);

  digitalWrite(M1, LOW);

  analogWrite( E2, right);

  digitalWrite(M2, HIGH);

}



void stop( void ){

  digitalWrite( E1, LOW );

  digitalWrite( E2, LOW );

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

  nh.spinOnce();

  delay(1);

}
