#include <Arduino.h>
#include "uart/uart_in.h"
#include "webservo/web.h"
#include "servo/servo_in.h"
#include "ws2312/ws2312.h"
#include "ASR/ASR_module.h"


#define bootraid  115200
#define TX 38
#define RX 39


ASR_MOUDLE asr;

uint8_t result = 0;

extern Servo E; 
extern Servo Y; 
extern Servo Z; 

const int LED_PIN = 2; // 举例

void setup() {
  Serial.begin(115200);                     // 调试串口
  pinMode(LED_PIN, OUTPUT);
  Servo_init();
  // wifi_init();
  ws2312_init();
  // 初始化语音 UART
  asr.ASR_init();
  Serial.println("UART In ready");
}

void loop() {
  result = asr.rec_recognition();  //返回识别结果，即识别到的词条编号
  if(result != 0)
  {
    if(result == 0x01)
    {
      Servo_active_one();
      Serial.println("test1");
    }else if(result == 0x02)
    {
      ws2312_test();
      Serial.println("test2");
    }else if(result == 0x03)
    {
      digitalWrite(LED_PIN, HIGH);
      Serial.println("test3");
    }else if(result == 0x04)
    {
      Serial.println("test4");
    }else if(result == 0x09)
    {
      Serial.println("stop");
    }
  }
}



