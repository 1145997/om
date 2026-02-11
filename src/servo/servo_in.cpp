#include "servo_in.h"


Servo E; 
Servo Y; 
Servo Z; 
Servo R;

int curR = R_Angle_Default;
int curY = Y_Angle_Default;
int curZ = Z_Angle_Default;

int clampServo(int a){
  if(a < 0) return 0;
  if(a > 180) return 180;
  return a;
}

void moveToSmooth(Servo &s, int &cur, int target, int stepDelayMs, int stepSize){
  target = clampServo(target);
  if(stepSize < 1) stepSize = 1;

  while(cur != target){
    if(cur < target) cur += stepSize;
    else             cur -= stepSize;

    // 防止越过
    if(abs(cur - target) < stepSize) cur = target;

    s.write(cur);
    delay(stepDelayMs);
  }
}


void Servo_Default(){
  R.write(R_Angle_Default);
  E.write(E_Angle_Default);
  Y.write(Y_Angle_Default);
  Z.write(Z_Angle_Default);
  delay(1000);
}



void Servo_zero(){
  R.write(R_Angle_Default-80);
  Y.write(Y_Angle_Default-80);
  Z.write(Z_Angle_Default-80);
  E.write(E_Angle_Default+80);
}

void Servo_init(){
  R.attach(R_Pin);
  E.attach(E_Pin);
  Y.attach(Y_Pin);
  Z.attach(Z_Pin);
  delay(50);
  Servo_Default();
}

void Servo_active_one(){
  Y.write(20);
  Z.write(20);
  E.write(E_Angle_Default-45);
}

void Servo_Shake_R(){
  curR = R_Angle_Default;
  R.write(curR);
  delay(100);

  for(int i=0;i<3;i++){
    moveToSmooth(R,curR, 70, 10, 1);  // 右：每10ms走1度
    delay(150);
    moveToSmooth(R,curR, 30, 10, 1);  // 左回
    delay(150);
  }

  Servo_Default();
}

void demo(){
  moveToSmooth(R, curR, 120, 10, 1);
  moveToSmooth(Y, curY, 60,  10, 1);
  moveToSmooth(Z, curZ, 150, 8,  2);
}

