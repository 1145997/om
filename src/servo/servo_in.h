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
// void Servo_active_one();
// void Servo_zero();
// void Servo_Shake_R();
// void demo();

// 播放器维护（必须在 loop 里反复调用）
void Servo_Update();

// 控制
void Servo_Stop();
bool Servo_IsBusy();

// 动作触发（外部只调用这些）
void Servo_PlayZero();
void Servo_PlayShakeR();
void Servo_PlayWave();
void Servo_PlayDemo();

#endif
