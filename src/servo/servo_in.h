#ifndef servo_in_h__
#define servo_in_h__

#include <Arduino.h>
#include <Servo.h>

#define R_Pin 39
#define Y_Pin 40
#define Z_Pin 41
#define E_Pin 42

#define R_Angle_Default 90
#define Y_Angle_Default 90
#define Z_Angle_Default 90
#define E_Angle_Default 90

void Servo_init();
void Servo_active_one();
void Servo_zero();
void Servo_Shake_R();
void demo();

#endif
