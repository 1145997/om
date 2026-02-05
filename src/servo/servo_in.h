#ifndef servo_in_h__
#define servo_in_h__

#include <Arduino.h>
#include <Servo.h>

#define Y_Pin 16
#define Z_Pin 17
#define E_Pin 18

#define Y_Angle_Default 90
#define Z_Angle_Default 90
#define E_Angle_Default 90

void Servo_init();
void Servo_active_one();

#endif
