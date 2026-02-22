#include <Arduino.h>
#include "uart/uart_in.h"
#include "webservo/web.h"
#include "servo/servo_in.h"
#include "ws2812/ws2812.h"
#include "ASR/ASR_module.h"
#include "sys/sys.h"


#define bootraid  115200
#define TX 38
#define RX 39

#define Leaser_pin 14

ASR_MOUDLE asr;
uint8_t result = 0;


void setup() {
  Serial.begin(115200);                     // 调试串口
  Servo_init();
  ws2812_init();
  sys_init();
  // wifi_init();
  // 初始化语音 UART
  asr.ASR_init();
  Serial.println("UART In ready");
}

void loop() {
    Servo_Update();  // 必须常驻
    ws2812_is_running();
    sys_service();

    result = asr.rec_recognition();  //返回识别结果，即识别到的词条编号
    if(result != 0){
      if(result == 0x01) {
        Servo_act_firest();
        biz_pulse_led(Leaser_pin);
        ws2812_demo1();
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
        Servo_act_air_warning(); 
        Serial.println("5");
      }
      else if(result == 0x06) { 
        Servo_act_report_crash(); 
        Serial.println("4");
      }
      else if(result == 0x07) { 
        Servo_act_gas_wave_need_cores_map(); 
        Serial.println("4");
      }
      else if(result == 0x08) { 
        Servo_act_dismantle_god_myth(); 
        Serial.println("4");
      }
      else if(result == 0x09) { 
        Servo_act_blow_the_box_fast(); 
        Serial.println("4");
      }
      else if(result == 0x0B) { 
        Servo_act_emergency_oxygen(); 
        Serial.println("4");
      }
      else if(result == 0x0C) { 
        Servo_act_ai_party_dizzy(); 
        Serial.println("4");
      }
      else if(result == 0x0D) { 
        Servo_act_party_glitch_spasm(); 
        Serial.println("4");
      }
      else if(result == 0x0E) { 
        Servo_act_point_3_knobs_20s(); 
        biz_pulse_led(Leaser_pin);
        Serial.println("4");
      }
      else if(result == 0x0F) { 
        Servo_act_doubt_not_sure_6s(); 
        Serial.println("4");
      }
      else if(result == 0x10) { 
        Servo_act_nav_abandoned_port(); 
        Serial.println("4");
      }
      else if(result == 0x11) { 
        Servo_act_nervous_apology_6s(); 
        Serial.println("4");
      }
      else if(result == 0x12) { 
        Servo_act_accuse_god_15s(); 
        Serial.println("4");
      }
      else if(result == 0x13) { 
        Servo_act_point_power_source_2s(); 
        Serial.println("4");
      }
      else if(result == 0x14) { 
        Servo_act_overload_need2_override_urgent_15s(); 
        Serial.println("4");
      }
    }
}



