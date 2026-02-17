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

extern WS2812FX_RMT strip;

const int LED_PIN_1 = 35; // 举例
const int LED_PIN_2 = 36; // 举例

void setup() {
  Serial.begin(115200);                     // 调试串口
  pinMode(LED_PIN_1, OUTPUT);
  pinMode(LED_PIN_2, OUTPUT);
  Servo_init();
  ws2812_init();
  // wifi_init();
  // 初始化语音 UART
  asr.ASR_init();
  Serial.println("UART In ready");
}

void loop() {
    Servo_Update();  // 必须常驻

    strip.tick();
    strip.show();

    result = asr.rec_recognition();  //返回识别结果，即识别到的词条编号
    if(result != 0){
      if(result == 0x01) {
        Servo_PlayShakeR();
        Serial.println("1");
      }
      else if(result == 0x02) {
        Servo_PlayZero();
        Serial.println("2");
      }
      else if(result == 0x03) {
        Servo_PlayDemo();
        Serial.println("3");
      }
      else if(result == 0x04) {
        Servo_act_test1();
        Serial.println("4");
      }
      else if(result == 0x05) {
        Servo_PlayZero();
        Serial.println("2");
      }
      else if(result == 0x06) {
        Servo_PlayDemo();
        Serial.println("3");
      }
      else if(result == 0x07) {
        Servo_PlayProudOK();
        Serial.println("4");
      }
      else if(result == 0x08) {
        Servo_PlayConfused();
        Serial.println("4");
      }
      else if(result == 0x09) {
        Servo_PlayWaveHi();
        Serial.println("2");
      }
      else if(result == 0x0A) {
        Servo_PlayScan();
        Serial.println("3");
      }
      else if(result == 0x0B) {
        Servo_PlayNodAck();
        Serial.println("4");
      }
    }
}



