#ifndef servo_in_h__
#define servo_in_h__

#include <Arduino.h>
#include <Servo.h>

#define A_Pin 36
#define B_Pin 37
#define C_Pin 38
#define R_Pin 39
#define Y_Pin 40
#define Z_Pin 41
#define E_Pin 42


#define R_Angle_Default 90
#define Y_Angle_Default 90
#define Z_Angle_Default 90
#define E_Angle_Default 90
#define A_Angle_Default 90
#define B_Angle_Default 90
#define C_Angle_Default 90




#define KF(r,y,z,e,a,b,c,ms)  { { (r),(y),(z),(e),(a),(b),(c) }, (uint16_t)(ms) }
#define SEQ_LEN(x) (int)(sizeof(x)/sizeof((x)[0]))


void Servo_init();

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
void Servo_act_test();
void Servo_act_test1();

void Servo_PlayNodAck();
void Servo_PlayScan();
void Servo_PlayWaveHi();
void Servo_PlayConfused();
void Servo_PlayProudOK();




#endif