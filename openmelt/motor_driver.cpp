//this module handles interfacing to the motors

#include "arduino.h"
#include "melty_config.h"
#include "motor_driver.h"
#include <Servo.h>

//motor_X_on functions are used for the powered phase of each rotation
//motor_X_coast functions are used for the unpowered phase of each rotation
//motor_X_off functions are used for when the robot is spun-down



void motor_on(float throttle_percent, int motor_pin) {

  if (THROTTLE_TYPE == BINARY_THROTTLE) {
    digitalWrite(motor_pin, HIGH);
  }
  if (THROTTLE_TYPE == FIXED_PWM_THROTTLE) {
    analogWrite(motor_pin, PWM_MOTOR_ON);
  }
  if (THROTTLE_TYPE == Brushless_Fixed_PWM_Throttle) {
      if (motor_pin = MOTOR_PIN1)
      {
        BrushlessMotor1.write(180);
      }
      else if (motor_pin == MOTOR_PIN2)
      {
        BrushlessMotor2.write(180);
      }

//If DYNAMIC_PWM_THROTTLE - PWM is scaled between PWM_MOTOR_COAST and PWM_MOTOR_ON
//Applies over range defined by DYNAMIC_PWM_THROTTLE_PERCENT_MAX - maxed at PWM_MOTOR_ON above this
  if (THROTTLE_TYPE == DYNAMIC_PWM_THROTTLE) {
    float throttle_pwm = PWM_MOTOR_COAST + ((throttle_percent / DYNAMIC_PWM_THROTTLE_PERCENT_MAX) * (PWM_MOTOR_ON - PWM_MOTOR_COAST));
    if (throttle_pwm > PWM_MOTOR_ON) throttle_pwm = PWM_MOTOR_ON;
    analogWrite(motor_pin, throttle_pwm);
  }
}

void motor_1_on(float throttle_percent) {
  motor_on(throttle_percent, MOTOR_PIN1);
}

void motor_2_on(float throttle_percent) {
  motor_on(throttle_percent, MOTOR_PIN2);
}

void motor_coast(int motor_pin) {
  if (THROTTLE_TYPE == FIXED_PWM_THROTTLE || THROTTLE_TYPE == DYNAMIC_PWM_THROTTLE) {
    analogWrite(motor_pin, PWM_MOTOR_COAST);
  }
  if (THROTTLE_TYPE == BINARY_THROTTLE) {
    digitalWrite(motor_pin, LOW);  //same as "off" for brushed motors
  }
  if (THROTTLE_TYPE == Brushless_Fixed_PWM_Throttle) {
      if (motor_pin = MOTOR_PIN1)
      {
        BrushlessMotor1.write(0);
      }
      else if (motor_pin == MOTOR_PIN2)
      {
        BrushlessMotor2.write(0);
      }  
}

void motor_1_coast() {
  motor_coast(MOTOR_PIN1);
}

void motor_2_coast() {
  motor_coast(MOTOR_PIN2);
}

void motor_off(int motor_pin) {
  if (THROTTLE_TYPE == FIXED_PWM_THROTTLE || THROTTLE_TYPE == DYNAMIC_PWM_THROTTLE) {
    analogWrite(motor_pin, PWM_MOTOR_OFF);
  }
  if (THROTTLE_TYPE == BINARY_THROTTLE) {
    digitalWrite(motor_pin, LOW);  //same as "off" for brushed motors
  }
  if (THROTTLE_TYPE == Brushless_Fixed_PWM_Throttle) {
      if (motor_pin = MOTOR_PIN1)
      {
        BrushlessMotor1.write(0);
      }
      else if (motor_pin = MOTOR_PIN1)
      {
        BrushlessMotor2.write(0);
      }
  }  
}

void motor_1_off() {
  motor_off(MOTOR_PIN1);
}

void motor_2_off() {
  motor_off(MOTOR_PIN2);
}

void motors_off() {
  motor_1_off();
  motor_2_off();
}

void init_motors() {
  if (THROTTLE_TYPE == Brushless_Fixed_PWM_Throttle){
    servo BrushlessMotor1;
    servo BrushlessMotor1;
    BrushlessMotor1.attach(MOTOR_PIN1,Brushless_PWM_Off,Brushless_PWM_On);
    BrushlessMotor2.attach(MOTOR_PIN2,Brushless_PWM_Off,Brushless_PWM_On);
    BrushlessMotor1.write(0);
    BrushlessMotor2.write(0);
    // at the time of comment the code does not make the distinction between servo objects
    // effectively making single wheel drive right now will add later -Dirac 5/16/2025
    mills(2000);
  }
  else 
  {
    pinMode(MOTOR_PIN1, OUTPUT);
    pinMode(MOTOR_PIN2, OUTPUT);
  }
  motors_off();
  
}
