#ifndef __ws2812_h__
#define __ws2812_h__

#include <WS2812FX.h>

#define LED_COUNT 15
#define LED_PIN 35




/* =========================
 * 分区范围（你已有）
 * 注意：WS2812FX 灯珠下标通常是 0..LED_COUNT-1
 * 你这里从 1 开始没问题，但要确保你物理/逻辑一致
 * ========================= */
#define Partition_A_Start 0
#define Partition_A_End   3
#define Partition_B_Start 4
#define Partition_B_End   6
#define Partition_C_Start 7
#define Partition_C_End   9
#define Partition_D_Start 10
#define Partition_D_End   12
#define Partition_E_Start 13
#define Partition_E_End   15

/* =========================
 * 分区 ID（可扩展：新增分区就在这里加）
 * ========================= */
typedef enum {
  WS2812_PART_A = 0,
  WS2812_PART_B,
  WS2812_PART_C,
  WS2812_PART_D,
  WS2812_PART_E,

  WS2812_PART_COUNT,   // 分区数量
  WS2812_PART_INVALID = 0xFF
} ws2812_partition_t;

typedef struct {
  uint16_t start;
  uint16_t end;
  bool     enabled;
} ws2812_partition_def_t;

/* =========================
 * 分区表（可扩展：新增分区就加一行）
 * ========================= */
static ws2812_partition_def_t g_parts[WS2812_PART_COUNT] = {
  { Partition_A_Start, Partition_A_End, true },
  { Partition_B_Start, Partition_B_End, true },
  { Partition_C_Start, Partition_C_End, true },
  { Partition_D_Start, Partition_D_End, true },
  { Partition_E_Start, Partition_E_End, true },
};

/* =========================
 * 模式（12种：红黄绿 × 呼吸/常亮/流水/闪烁）
 * ========================= */
typedef enum {
  WS2812_MODE_RED_BREATH = 0,
  WS2812_MODE_RED_SOLID,
  WS2812_MODE_RED_FLOW,
  WS2812_MODE_RED_BLINK,

  WS2812_MODE_YELLOW_BREATH,
  WS2812_MODE_YELLOW_SOLID,
  WS2812_MODE_YELLOW_FLOW,
  WS2812_MODE_YELLOW_BLINK,

  WS2812_MODE_GREEN_BREATH,
  WS2812_MODE_GREEN_SOLID,
  WS2812_MODE_GREEN_FLOW,
  WS2812_MODE_GREEN_BLINK,

  WS2812_MODE_COUNT,
  WS2812_MODE_INVALID = 0xFF
} ws2812_mode_t;

typedef struct {
  uint8_t  fx_mode;     // WS2812FX 的 FX_MODE_*
  uint32_t color;       // 0xRRGGBB
  uint16_t speed;       // WS2812FX speed
  uint8_t  options;     // NO_OPTIONS 等
} ws2812_mode_def_t;

/* =========================
 * 模式表（你要改速度/效果，只改这里）
 * ========================= */
static const ws2812_mode_def_t g_modes[WS2812_MODE_COUNT] = {
  // RED
  { FX_MODE_BREATH,         0xFF0000, 1500, NO_OPTIONS }, // RED_BREATH
  { FX_MODE_STATIC,         0xFF0000, 1000, NO_OPTIONS }, // RED_SOLID
  { FX_MODE_RUNNING_LIGHTS, 0xFF0000,  900, NO_OPTIONS }, // RED_FLOW
  { FX_MODE_BLINK,          0xFF0000,  500, NO_OPTIONS }, // RED_BLINK

  // YELLOW
  { FX_MODE_BREATH,         0xFFFF00, 1500, NO_OPTIONS }, // YELLOW_BREATH
  { FX_MODE_STATIC,         0xFFFF00, 1000, NO_OPTIONS }, // YELLOW_SOLID
  { FX_MODE_RUNNING_LIGHTS, 0xFFFF00,  900, NO_OPTIONS }, // YELLOW_FLOW
  { FX_MODE_BLINK,          0xFFFF00,  500, NO_OPTIONS }, // YELLOW_BLINK

  // GREEN
  { FX_MODE_BREATH,         0x00FF00, 1500, NO_OPTIONS }, // GREEN_BREATH
  { FX_MODE_STATIC,         0x00FF00, 1000, NO_OPTIONS }, // GREEN_SOLID
  { FX_MODE_RUNNING_LIGHTS, 0x00FF00,  900, NO_OPTIONS }, // GREEN_FLOW
  { FX_MODE_BLINK,          0x00FF00,  500, NO_OPTIONS }, // GREEN_BLINK
};




void ws2812_init();
void ws2812_is_running();
void ws2812_Change();
//***********业务区**************
void ws2812_demo1();

#endif
