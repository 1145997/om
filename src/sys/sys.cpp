#include "sys.h"

// ========================
// 内部：IO 非阻塞任务系统
// ========================

namespace {

// 你可以按需要调整并发任务数
constexpr uint8_t MAX_IO_TASKS = 8;

struct IoFlipTask {
  bool     active = false;
  uint8_t  pin = 0;

  // 频率翻转：一个“翻转”就是一次电平切换（HIGH<->LOW）
  // times: 翻转次数（例如 10 次 = 切换 10 次）
  uint16_t remaining_toggles = 0;

  uint32_t half_period_ms = 0;   // 半周期毫秒（翻转间隔）
  uint32_t next_due_ms = 0;      // 下次执行时间点
};

IoFlipTask g_tasks[MAX_IO_TASKS];

// 立即翻转电平（非阻塞）
static inline void io_trigger(uint8_t pin) {
  // ESP32 Arduino：digitalRead 对 OUTPUT 引脚可读回当前电平（通常OK）
  digitalWrite(pin, !digitalRead(pin));
}

// 投递一个“持续翻转任务”
// pin: 引脚
// frequency_hz: 翻转波形的频率（例如 2Hz 表示一个完整方波周期 0.5s）
// times: 翻转次数（电平切换次数）
static bool io_Continuous_flipping(uint8_t pin, float frequency_hz, uint16_t times) {
  if (frequency_hz <= 0.0f || times == 0) return false;

  // half_period = 1000ms / (2 * frequency)
  // 例如 2Hz：周期 500ms，半周期 250ms（每 250ms 翻转一次）
  uint32_t half_period = (uint32_t)(1000.0f / (2.0f * frequency_hz));
  if (half_period == 0) half_period = 1;

  // 找空槽位
  for (uint8_t i = 0; i < MAX_IO_TASKS; i++) {
    if (!g_tasks[i].active) {
      g_tasks[i].active = true;
      g_tasks[i].pin = pin;
      g_tasks[i].remaining_toggles = times;
      g_tasks[i].half_period_ms = half_period;
      g_tasks[i].next_due_ms = millis(); // 立刻允许第一次翻转
      return true;
    }
  }
  return false; // 没位置了
}

// 调度：每次 sys_service() 调用都跑一次
static void io_service() {
  uint32_t now = millis();

  for (uint8_t i = 0; i < MAX_IO_TASKS; i++) {
    auto &t = g_tasks[i];
    if (!t.active) continue;

    // 到点就翻转一次
    // 用有符号差值写法，避免 millis 溢出问题
    if ((int32_t)(now - t.next_due_ms) >= 0) {
      io_trigger(t.pin);

      if (t.remaining_toggles > 0) t.remaining_toggles--;

      if (t.remaining_toggles == 0) {
        t.active = false;
      } else {
        t.next_due_ms += t.half_period_ms; // 走固定节拍，不“漂移”
      }
    }
  }
}

} // namespace


// ========================
// 对外：系统层
// ========================

void sys_init() {
  // 这里放所有系统初始化：IO、串口、传感器、网络……
  // IO 示例：你也可以不在这里统一 pinMode，而在业务函数里设置
  // pinMode(2, OUTPUT);

  // 清空任务
  for (auto &t : g_tasks) t = IoFlipTask{};
}

void sys_service() {
  // 所有系统轮询都丢这里：IO 调度、状态机、定时器驱动……
  io_service();
}

// ========================
// 示例业务函数：只“投递任务”
// ========================

void biz_pulse_led(uint8_t pin) {
  pinMode(pin, OUTPUT);
  // 以 5Hz 翻转 10 次（约 1 秒：5Hz 的方波周期 200ms，半周期 100ms，10 次翻转=1s）
  // 你可以按你的“次数/速度”概念改参数
  io_Continuous_flipping(pin, 1.0f, 10);
}