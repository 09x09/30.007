#include "Motor.h"
#include "Arduino.h"

Motor::Motor(int r_en, int r_pwm, int l_en, int l_pwm)
{
  R_EN = r_en;
  R_PWM = r_pwm;
  L_EN = l_en;
  L_PWM = l_pwm;
};

void Motor::init()
{
  pinMode(R_EN, OUTPUT);
  pinMode(R_PWM, OUTPUT);
  pinMode(L_EN, OUTPUT);
  pinMode(L_PWM, OUTPUT);
};

void Motor::move(char a, char b) //Move backward, b<a
{
  analogWrite(R_PWM, a); //PWM Speed Control
  digitalWrite(R_EN, HIGH);
  analogWrite(L_PWM, b);
  digitalWrite(L_EN, HIGH);
};

void Motor::stop(void) //Stop
{
  digitalWrite(R_EN, LOW);
  digitalWrite(L_EN, LOW);

};