#include <Arduino.h>
#include "uart_in.h"
#include "webservo/web.h"
#include "servo/servo_in.h"
#include "ws2312/ws2312.h"



#define bootraid  115200
#define TX 38
#define RX 39

extern Servo E; 
extern Servo Y; 
extern Servo Z; 

const int LED_PIN = 2; // 举例

void setup() {
  Serial.begin(115200);                     // 调试串口
  pinMode(LED_PIN, OUTPUT);
  Servo_init();
  wifi_init();
  ws2312_init();
  // 初始化语音 UART
  uartInInit(bootraid, TX, RX); // 波特率、RX、TX
  Serial.println("UART In ready");
}

void loop() {
  uint16_t cmd;
  // 1. 检查是否接收到了协议帧
  if (readCmdFromUART(cmd)) {
    Serial.print("Recv cmd: 0x");
    Serial.println(cmd, HEX);

    // 2. 将命令解析为动作编号
    uint8_t act = parseAction(cmd);
    switch (act)
    {
    case test_1 : digitalWrite(2, HIGH); 
      break;
    case test_2 : ws2312_test();
      break;
    case test_3 : Servo_active_one();
    
    default:
      break;
    }
  }
}



