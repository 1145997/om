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
        // 台词：警告，空气中化合物超标。
        Serial.println("5");
      }

      else if(result == 0x06) { 
        Servo_act_report_crash(); 
        // 台词：委托显示浮空岛动力愈况而下……整座岛屿将坠毁。委托人：涡轮总督。
        Serial.println("4");
      }

      else if(result == 0x07) { 
        Servo_act_gas_wave_need_cores_map(); 
        biz_scan_sig(Leaser_pin);
        biz_start_scan();
        // 台词：正在分析，检测到未知气体波形，需要三枚核心构件进一步解析成分。
        Serial.println("4");
      }

      else if(result == 0x08) { 
        Servo_act_dismantle_god_myth(); 
        // 台词：我认为，如果他们的神明真的存在……所谓神谕不过是维护不当产生的摩擦声。
        Serial.println("4");
      }

      else if(result == 0x09) { 
        Servo_act_blow_the_box_fast(); 

        // 台词：我的算法分析显示，与其费劲解密，不如直接把箱子炸了。
        Serial.println("4");
      }

      else if(result == 0x0B) { 
        Servo_act_emergency_oxygen(); 
        // 台词：警告，未知气体浓度加剧，已启用应急制氧。
        Serial.println("4");
      }

      else if(result == 0x0C) { 
        Servo_act_ai_party_dizzy(); 
        // 台词：他的意思是……机箱的主理人在商业街开了一场超棒的AI派对！（晕晕的）
        Serial.println("4");
      }

      else if(result == 0x0D) { 
        biz_err_2_start();
        biz_err_2();
        biz_err_2_end();
        Servo_act_party_glitch_spasm(); 
        // 台词：超棒的派对哦吼吼吼吼吼锟斤拷！（抽风乱甩）
        Serial.println("4");
      }

      else if(result == 0x0E) { 
        Servo_act_point_3_knobs_20s(); 
        // 台词：正在分析+分析完毕，请按指示操作三个旋钮。
        biz_pulse_led(Leaser_pin); // 激光提示
        Serial.println("4");
      }

      else if(result == 0x0F) { 
        Servo_act_doubt_not_sure_6s(); 
        // 台词：我看未必。
        Serial.println("4");
      }

      else if(result == 0x10) { 
        Servo_act_nav_abandoned_port(); 
        // 台词：收到。导航到废弃港口。
        Serial.println("4");
      }

      else if(result == 0x11) { 
        Servo_act_nervous_apology_6s(); 
        // 台词：额……我说我不是故意的，你信吗？
        Serial.println("4");
      }

      else if(result == 0x12) { 
        Servo_act_accuse_god_15s(); 
        // 台词：万机之神……你所谓的进化，就是让全岛的人呼吸你的尾气吗？
        Serial.println("4");
      }

      else if(result == 0x13) { 
        Servo_act_point_power_source_2s(); 
        // 台词：那个装置正在源源不断地给他供能。
        Serial.println("4");
      }

      else if(result == 0x14) { 
        Servo_act_overload_need2_override_urgent_15s(); 
        // 台词：检测到装置超载运行……我需要剩余两个核心部件，手动覆写。
        Serial.println("4");
      }
    }
}



