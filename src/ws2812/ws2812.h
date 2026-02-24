#ifndef __ws2812_h__
#define __ws2812_h__

#include <WS2812FX.h>

#define LED_COUNT 50
#define LED_PIN 35




/* =========================
 * 分区范围（你已有）
 * 注意：WS2812FX 灯珠下标通常是 0..LED_COUNT-1
 * 你这里从 1 开始没问题，但要确保你物理/逻辑一致
 * ========================= */
#define Partition_A_Start 0
#define Partition_A_End   10
#define Partition_B_Start 11
#define Partition_B_End   20
#define Partition_C_Start 21
#define Partition_C_End   30
#define Partition_D_Start 31
#define Partition_D_End   40
#define Partition_E_Start 41
#define Partition_E_End   50

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

  WS2812_MODE_OFF,
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
  { FX_MODE_BREATH,         0xFF0000, 200, NO_OPTIONS }, // RED_BREATH
  { FX_MODE_STATIC,         0xFF0000, 1000, NO_OPTIONS }, // RED_SOLID
  { FX_MODE_RUNNING_LIGHTS, 0xFF0000,  900, NO_OPTIONS }, // RED_FLOW
  { FX_MODE_BLINK,          0xFF0000,  500, NO_OPTIONS }, // RED_BLINK

  // YELLOW
  { FX_MODE_BREATH,         0xFFFF00, 200, NO_OPTIONS }, // YELLOW_BREATH
  { FX_MODE_STATIC,         0xFFFF00, 1000, NO_OPTIONS }, // YELLOW_SOLID
  { FX_MODE_RUNNING_LIGHTS, 0xFFFF00,  900, NO_OPTIONS }, // YELLOW_FLOW
  { FX_MODE_BLINK,          0xFFFF00,  500, NO_OPTIONS }, // YELLOW_BLINK

  // GREEN
  { FX_MODE_BREATH,         0x00FF00, 200, NO_OPTIONS }, // GREEN_BREATH
  { FX_MODE_STATIC,         0x00FF00, 1000, NO_OPTIONS }, // GREEN_SOLID
  { FX_MODE_RUNNING_LIGHTS, 0x00FF00,  900, NO_OPTIONS }, // GREEN_FLOW
  { FX_MODE_BLINK,          0x00FF00,  500, NO_OPTIONS }, // GREEN_BLINK

  { FX_MODE_STATIC, 0x000000, 1000, NO_OPTIONS }, // OFF
};




void ws2812_init();
void ws2812_is_running();
void ws2812_Change();
//***********业务区**************
void ws2812_demo1();
void ws2812_scan_sig();
void ws2812_scan_sig_worning();


void ws2812_err_2();


/* =========================
 * 全部常亮
 * ========================= */
void ws2812_staute_red(void);
void ws2812_staute_yello(void);
void ws2812_staute_green(void);

/* =========================
 * 全部呼吸
 * ========================= */
void ws2812_all_breath_red(void);
void ws2812_all_breath_yellow(void);
void ws2812_all_breath_green(void);

/* =========================
 * 全部闪烁
 * ========================= */
void ws2812_all_blink_red(void);
void ws2812_all_blink_yellow(void);
void ws2812_all_blink_green(void);

/* =========================
 * 全部流水
 * ========================= */
void ws2812_all_flow_red(void);
void ws2812_all_flow_yellow(void);
void ws2812_all_flow_green(void);



void fx_detecting() ;
void fx_detect_done() ;
void fx_emergency() ;
void fx_glitch_overload() ;
void fx_battle();
void fx_override_running();
void fx_override_done();







#endif
