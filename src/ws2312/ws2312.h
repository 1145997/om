#ifndef ws2312_h
#define ws2312_h

#include <Arduino.h>
#include <driver/rmt.h>
#include <math.h>
#define NUM_LEDS 200 // 定义LED灯珠数量
#define DATA_PIN 6 // 定义数据引脚

void ws2312_init() ;
void ws2312_test() ;



#endif