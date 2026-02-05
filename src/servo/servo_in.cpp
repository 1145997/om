#include "servo_in.h"


Servo E; 
Servo Y; 
Servo Z; 

void Servo_init(){
  E.attach(E_Pin);
  Y.attach(Y_Pin);
  Z.attach(Z_Pin);
  delay(50);
  E.write(E_Angle_Default);
  Y.write(Y_Angle_Default);
  Z.write(Z_Angle_Default);
  delay(1000);
}

void Servo_active_one(){
  Y.write(Y_Angle_Default+20);
  Z.write(Z_Angle_Default-20);
  E.write(E_Angle_Default-45);
}

