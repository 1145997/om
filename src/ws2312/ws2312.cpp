#include "ws2312.h"


CRGB leds[NUM_LEDS]; // 创建一个CRGB数组来存储LED状态
void ws2312_init() {
   FastLED.addLeds<WS2812B, DATA_PIN, GRB>(leds, NUM_LEDS); // 初始化LED灯带
   FastLED.setBrightness(120); // 设置亮度
}
void ws2312_test() {
   for (int i = 0; i < NUM_LEDS; i++) {
       leds[i] = CRGB::Green; // 将所有LED设置为绿色
   }
   FastLED.show(); // 更新LED显示
   delay(1000); // 延迟1秒
   for (int i = 0; i < NUM_LEDS; i++) {
       leds[i] = CRGB::Black; // 关闭所有LED
   }
   FastLED.show(); // 更新LED显示
   delay(1000); // 延迟1秒
}