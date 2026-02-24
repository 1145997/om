#include "ws2812.h"

WS2812FX ws2812fx = WS2812FX(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

void ws2812_is_running(){
      ws2812fx.service();
}

/* =========================
 * 初始化：把每个分区注册成一个 segment
 * 建议在 ws2812fx.init() 之后、start() 之前调用一次
 * ========================= */
static inline void ws2812_PartitionsInit() {
  for (uint8_t i = 0; i < WS2812_PART_COUNT; i++) {
    if (!g_parts[i].enabled) continue;

    // 给每个分区一个默认静态关灯/低亮也行，这里默认关
    ws2812fx.setSegment(
      (uint8_t)i,
      (uint16_t)g_parts[i].start,
      (uint16_t)g_parts[i].end,
      (uint8_t)FX_MODE_STATIC,
      (uint32_t)0x000000,
      (uint16_t)1000,
      (uint8_t)NO_OPTIONS
    );
  }
}

/* =========================
 * 对外统一 API：改变某个分区为某个模式
 * ========================= */
static inline bool ws2812_Change(ws2812_partition_t part, ws2812_mode_t mode) {
  if (part >= WS2812_PART_COUNT) return false;
  if (mode >= WS2812_MODE_COUNT) return false;
  if (!g_parts[part].enabled)    return false;

  const ws2812_partition_def_t &p = g_parts[part];
  const ws2812_mode_def_t &m = g_modes[mode];

  ws2812fx.setSegment(
    (uint8_t)part,
    p.start,
    p.end,
    m.fx_mode,
    m.color,
    m.speed,
    m.options
  );

  return true;
}

void ws2812_init(){
    ws2812fx.init();
    ws2812fx.setBrightness(100);
    ws2812_PartitionsInit();
    ws2812fx.start();
}



//***********业务区**************

void ws2812_demo1(){
    ws2812_Change(WS2812_PART_B, WS2812_MODE_GREEN_FLOW);
    ws2812_Change(WS2812_PART_A, WS2812_MODE_RED_BLINK);
}



//关闭
void ws2812_part_off(ws2812_partition_t part)
{
  ws2812_Change(part, WS2812_MODE_OFF);
}

/*控制区*/
/*全常亮*/
void ws2812_staute_green() {
  for (uint8_t i = 0; i < WS2812_PART_COUNT; i++) {
    ws2812_Change((ws2812_partition_t)i, WS2812_MODE_GREEN_SOLID);
  }
}

void ws2812_staute_yello() {
  for (uint8_t i = 0; i < WS2812_PART_COUNT; i++) {
    ws2812_Change((ws2812_partition_t)i, WS2812_MODE_YELLOW_SOLID);
  }
}

void ws2812_staute_red() {
  for (uint8_t i = 0; i < WS2812_PART_COUNT; i++) {
    ws2812_Change((ws2812_partition_t)i, WS2812_MODE_RED_SOLID);
  }
}

/*全闪*/
void ws2812_all_blink_red() {
  for (uint8_t i = 0; i < WS2812_PART_COUNT; i++) {
    ws2812_Change((ws2812_partition_t)i, WS2812_MODE_RED_BLINK);
  }
}

void ws2812_all_blink_yellow() {
  for (uint8_t i = 0; i < WS2812_PART_COUNT; i++) {
    ws2812_Change((ws2812_partition_t)i, WS2812_MODE_YELLOW_BLINK);
  }
}

void ws2812_all_blink_green() {
  for (uint8_t i = 0; i < WS2812_PART_COUNT; i++) {
    ws2812_Change((ws2812_partition_t)i, WS2812_MODE_GREEN_BLINK);
  }
}

/*全呼吸*/
void ws2812_all_breath_red() {
  for (uint8_t i = 0; i < WS2812_PART_COUNT; i++) {
    ws2812_Change((ws2812_partition_t)i, WS2812_MODE_RED_BREATH);
  }
}

void ws2812_all_breath_yellow() {
  for (uint8_t i = 0; i < WS2812_PART_COUNT; i++) {
    ws2812_Change((ws2812_partition_t)i, WS2812_MODE_YELLOW_BREATH);
  }
}

void ws2812_all_breath_green() {
  for (uint8_t i = 0; i < WS2812_PART_COUNT; i++) {
    ws2812_Change((ws2812_partition_t)i, WS2812_MODE_GREEN_BREATH);
  }
}
/*全流水*/
void ws2812_all_flow_red() {
  for (uint8_t i = 0; i < WS2812_PART_COUNT; i++) {
    ws2812_Change((ws2812_partition_t)i, WS2812_MODE_RED_FLOW);
  }
}

void ws2812_all_flow_yellow() {
  for (uint8_t i = 0; i < WS2812_PART_COUNT; i++) {
    ws2812_Change((ws2812_partition_t)i, WS2812_MODE_YELLOW_FLOW);
  }
}

void ws2812_all_flow_green() {
  for (uint8_t i = 0; i < WS2812_PART_COUNT; i++) {
    ws2812_Change((ws2812_partition_t)i, WS2812_MODE_GREEN_FLOW);
  }
}





//预设


void ws2812_scan_sig(){
    ws2812_Change(WS2812_PART_C, WS2812_MODE_GREEN_BREATH);
    ws2812_Change(WS2812_PART_B, WS2812_MODE_GREEN_BREATH);
    ws2812_Change(WS2812_PART_A, WS2812_MODE_GREEN_BREATH);
}

void ws2812_scan_sig_worning(){
    ws2812_Change(WS2812_PART_C, WS2812_MODE_RED_BLINK);
    ws2812_Change(WS2812_PART_B, WS2812_MODE_RED_BLINK);
    ws2812_Change(WS2812_PART_A, WS2812_MODE_RED_BLINK);
}

void ws2812_scan_sig_worning_end(){
    ws2812_Change(WS2812_PART_C, WS2812_MODE_RED_BLINK);
    ws2812_Change(WS2812_PART_B, WS2812_MODE_RED_BLINK);
    ws2812_Change(WS2812_PART_A, WS2812_MODE_RED_BLINK);
}


void ws2812_err_2() {
  for (uint8_t i = 0; i < WS2812_PART_COUNT; i++) {
    if (!g_parts[i].enabled) continue;

    ws2812fx.setSegment(
        (uint8_t)i,
        (uint16_t)g_parts[i].start,
        (uint16_t)g_parts[i].end,
        (uint8_t)FX_MODE_RAINBOW_CYCLE,
        (uint32_t)0,
        (uint16_t)1200,
        (uint8_t)NO_OPTIONS
    );
  }
}



void fx_detecting() {
  ws2812_Change(WS2812_PART_A, WS2812_MODE_YELLOW_BREATH);
  ws2812_Change(WS2812_PART_B, WS2812_MODE_YELLOW_FLOW);
  ws2812_Change(WS2812_PART_C, WS2812_MODE_GREEN_SOLID);
  ws2812_Change(WS2812_PART_D, WS2812_MODE_GREEN_FLOW);
  ws2812_Change(WS2812_PART_E, WS2812_MODE_YELLOW_BREATH);
}

void fx_detect_done() {
  ws2812_Change(WS2812_PART_A, WS2812_MODE_GREEN_SOLID);
  ws2812_Change(WS2812_PART_B, WS2812_MODE_GREEN_FLOW);
  ws2812_Change(WS2812_PART_C, WS2812_MODE_GREEN_BREATH);
  ws2812_Change(WS2812_PART_D, WS2812_MODE_GREEN_FLOW);
  ws2812_Change(WS2812_PART_E, WS2812_MODE_GREEN_SOLID);
}

void fx_emergency() {
  ws2812_Change(WS2812_PART_A, WS2812_MODE_RED_BLINK);
  ws2812_Change(WS2812_PART_B, WS2812_MODE_RED_BLINK);
  ws2812_Change(WS2812_PART_C, WS2812_MODE_RED_BREATH);
  ws2812_Change(WS2812_PART_D, WS2812_MODE_YELLOW_BLINK);
  ws2812_Change(WS2812_PART_E, WS2812_MODE_RED_BLINK);
}

void fx_glitch_overload() {
  ws2812_Change(WS2812_PART_B, WS2812_MODE_YELLOW_BLINK);
  ws2812_Change(WS2812_PART_A, WS2812_MODE_YELLOW_BLINK);
  ws2812_Change(WS2812_PART_C, WS2812_MODE_RED_BREATH);
  ws2812_Change(WS2812_PART_D, WS2812_MODE_GREEN_SOLID);
  ws2812_Change(WS2812_PART_E, WS2812_MODE_YELLOW_BLINK);
}

void fx_battle() {
  ws2812_Change(WS2812_PART_A, WS2812_MODE_RED_BREATH);
  ws2812_Change(WS2812_PART_B, WS2812_MODE_RED_FLOW);
  ws2812_Change(WS2812_PART_C, WS2812_MODE_RED_SOLID);
  ws2812_Change(WS2812_PART_D, WS2812_MODE_YELLOW_FLOW);
  ws2812_Change(WS2812_PART_E, WS2812_MODE_RED_BLINK);
}

// 覆写进行中
void fx_override_running() {
  ws2812_Change(WS2812_PART_A, WS2812_MODE_YELLOW_BREATH);
  ws2812_Change(WS2812_PART_B, WS2812_MODE_YELLOW_BREATH);
  ws2812_Change(WS2812_PART_C, WS2812_MODE_YELLOW_BREATH);
  ws2812_Change(WS2812_PART_D, WS2812_MODE_YELLOW_FLOW);
  ws2812_Change(WS2812_PART_E, WS2812_MODE_YELLOW_BREATH);
}

// 解除完成（稳定）
void fx_override_done() {
  ws2812_Change(WS2812_PART_A, WS2812_MODE_GREEN_SOLID);
  ws2812_Change(WS2812_PART_B, WS2812_MODE_GREEN_SOLID);
  ws2812_Change(WS2812_PART_C, WS2812_MODE_GREEN_SOLID);
  ws2812_Change(WS2812_PART_D, WS2812_MODE_GREEN_SOLID);
  ws2812_Change(WS2812_PART_E, WS2812_MODE_GREEN_SOLID);
}


