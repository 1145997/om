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






