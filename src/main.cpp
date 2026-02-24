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
#define Radar_pin 13

ASR_MOUDLE asr;
uint8_t result = 0;


void setup() {
  Serial.begin(115200);                     // 调试串口
  Servo_init();
  ws2812_init();
  ws2812_staute_green();
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
        biz_start_air_warning(); // 对应空气预警
        Serial.println("5");
      }

      else if(result == 0x06) { 
        Servo_act_report_crash(); 
        biz_start_crash_report(); // 对应坠毁报告
        Serial.println("6");
      }

      else if(result == 0x07) { 
        Servo_act_gas_wave_need_cores_map(); 
        biz_start_gas_wave_scan(Leaser_pin); // 对应气体波形扫描（已包含引脚操作）
        Serial.println("7");
      }

      else if(result == 0x08) { 
        Servo_act_dismantle_god_myth(); 
        biz_start_dismantle_myth(); // 对应拆解谜团
        Serial.println("8");
      }

      else if(result == 0x09) { 
        Servo_act_blow_the_box_fast(); 
        biz_start_blow_box(); // 对应直接炸箱子
        Serial.println("9");
      }

      else if(result == 0x0B) { 
        Servo_act_emergency_oxygen(); 
        biz_start_emergency_oxygen(); // 对应应急制氧
        Serial.println("11");
      }

      else if(result == 0x0C) { 
        Servo_act_ai_party_dizzy(); 
        biz_start_ai_party_dizzy(); // 对应派对/眩晕
        Serial.println("12");
      }

      else if(result == 0x0D) { 
        Servo_act_party_glitch_spasm(); 
        biz_start_glitch_spasm(); // 对应抽风乱甩（内部包含彩虹灯效）
        Serial.println("13");
      }

      else if(result == 0x0E) { 
        Servo_act_point_3_knobs_20s(); 
        biz_start_point_knobs(); // 对应旋钮分析提示
        biz_pulse_led(Leaser_pin); 
        Serial.println("14");
      }

      else if(result == 0x0F) { 
        Servo_act_doubt_not_sure_6s(); 
        biz_start_doubt(); // 对应怀疑状态
        Serial.println("15");
      }

      else if(result == 0x10) { 
        Servo_act_nav_abandoned_port(); 
        biz_start_nav_port(); // 对应导航废弃港口
        Serial.println("16");
      }

      else if(result == 0x11) { 
        Servo_act_nervous_apology_6s();
        biz_start_nervous_apology(Radar_pin); // 对应紧张道歉
        Serial.println("17");
      }

      else if(result == 0x12) { 
        Servo_act_accuse_god_15s(); 
        biz_start_accusation(); // 对应指责情绪
        Serial.println("18");
      }

      else if(result == 0x13) { 
        Servo_act_point_power_source_2s(); 
        biz_start_point_power(); // 对应电力装置指向
        Serial.println("19");
      }

      else if(result == 0x14) { 
        Servo_act_overload_need2_override_urgent_15s(); 
        biz_start_overload_override(); // 对应装置超载覆写
        Serial.println("20");
      }
  }
}


