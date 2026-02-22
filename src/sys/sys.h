#pragma once
#include <Arduino.h>

// ===== 对外 API =====
void sys_init();
void sys_service();

// 示例业务函数：触发某个引脚以 5Hz 翻转 10 次（你可以按需改/加）
void biz_pulse_led(uint8_t pin);