#pragma once
#include <Arduino.h>
#include "../ws2812/ws2812.h"


// ===== 对外 API =====
void sys_init();
void sys_service();

// 示例业务函数：触发某个引脚以 5Hz 翻转 10 次（你可以按需改/加）
void biz_pulse_led(uint8_t pin);
void biz_scan_sig(uint8_t pin);

void biz_start_scan();

void biz_err_2_start(void);
void biz_err_2(void);
void biz_err_2_end(void);
