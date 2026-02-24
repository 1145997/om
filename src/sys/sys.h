#pragma once
#include <Arduino.h>
#include "../ws2812/ws2812.h"


// sys调用
typedef void (*SysFunc)();

struct SysStep {
  SysFunc   func;
  uint32_t  offset_ms;   // 相对 job 开始时间
};

struct SysJob {
  const SysStep* steps;
  uint16_t count;
};

#define JOB_COUNT(arr) (uint16_t)(sizeof(arr)/sizeof(arr[0]))




// ===== 对外 API =====
void sys_init();
void sys_service();

// 示例业务函数：触发某个引脚以 5Hz 翻转 10 次（你可以按需改/加）
void biz_pulse_led(uint8_t pin) ;


/* --- 业务逻辑接口 (Business Logic Interfaces) --- */

// 警报与故障类
void biz_start_air_warning(void);          // 空气预警流程
void biz_start_crash_report(void);         // 崩溃/灾难报告
void biz_start_glitch_spasm(void);         // 故障抽风/过载效果

// 检测与分析类
void biz_start_gas_wave_scan(uint8_t pin); // 气体波形扫描 (需传入GPIO引脚)
void biz_start_point_knobs(void);          // 旋钮指向/分析
void biz_start_point_power(void);          // 电力指向/提示

// 系统应急类
void biz_start_emergency_oxygen(void);    // 应急制氧
void biz_start_overload_override(void);   // 过载覆写流程

// 情绪与状态表达类
void biz_start_dismantle_myth(void);      // 拆解谜团/思考
void biz_start_blow_box(void);             // 蓄力爆炸效果
void biz_start_ai_party_dizzy(void);       // AI派对/眩晕
void biz_start_doubt(void);                // 怀疑/黄色呼吸
void biz_start_nervous_apology(uint8_t pin); // 紧张道歉 (需传入GPIO引脚)
void biz_start_accusation(void);           // 指责/战斗情绪

// 导航类
void biz_start_nav_port(void);             // 导航端口灯效










