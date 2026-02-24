#include "sys.h"

// ========================
// 内部：IO 非阻塞任务系统
// ========================

namespace {

  // ===== 延时任务系统 =====

static constexpr uint8_t MAX_DELAY_TASKS = 8;

struct DelayTask {
    bool used = false;
    bool active = false;

    uint32_t trigger_time = 0;
    void (*callback)() = nullptr;
};

static DelayTask g_delay_tasks[MAX_DELAY_TASKS];

// 注册一个延时调用
static void sys_delay_call(void (*func)(), uint32_t delay_ms)
{
    for(int i=0;i<MAX_DELAY_TASKS;i++)
    {
        if(!g_delay_tasks[i].used)
        {
            g_delay_tasks[i].used = true;
            g_delay_tasks[i].active = true;
            g_delay_tasks[i].callback = func;
            g_delay_tasks[i].trigger_time = millis() + delay_ms;
            return;
        }
    }
}







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

  // 新增：延迟启动的时间点
  uint32_t start_time_ms = 0;
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
static bool io_Continuous_flipping(uint8_t pin, float frequency_hz, uint16_t times, uint32_t delay_ms) {
  if (frequency_hz <= 0.0f || times == 0) return false;

  uint32_t half_period = (uint32_t)(1000.0f / (2.0f * frequency_hz));
  if (half_period == 0) half_period = 1;

  uint32_t now = millis();

  for (uint8_t i = 0; i < MAX_IO_TASKS; i++) {
    if (!g_tasks[i].active) {
      auto &t = g_tasks[i];
      t.active = true;
      t.pin = pin;
      t.remaining_toggles = times;
      t.half_period_ms = half_period;

      // 延迟启动
      t.start_time_ms = now + delay_ms;

      // “下一次到期”从启动点开始（到点就立刻翻转一次）
      t.next_due_ms = t.start_time_ms;

      return true;
    }
  }
  return false;
}

// 调度：每次 sys_service() 调用都跑一次
static void io_service() {
  uint32_t now = millis();

  for(int i=0;i<MAX_DELAY_TASKS;i++){
        auto &t = g_delay_tasks[i];
        if(!t.used || !t.active) continue;

        if((int32_t)(now - t.trigger_time) >= 0)
        {
            t.active = false;
            if(t.callback) t.callback();
        }
    }

  for (uint8_t i = 0; i < MAX_IO_TASKS; i++) {
    auto &t = g_tasks[i];
    if (!t.active) continue;

    // 还没到启动时间，先不动
    if ((int32_t)(now - t.start_time_ms) < 0) {
      continue;
    }

    if ((int32_t)(now - t.next_due_ms) >= 0) {
      io_trigger(t.pin);

      if (t.remaining_toggles > 0) t.remaining_toggles--;

      if (t.remaining_toggles == 0) {
        t.active = false;
      } else {
        t.next_due_ms += t.half_period_ms; // 固定节拍
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
  io_Continuous_flipping(pin, 1.0f, 10,2000);
}

void biz_scan_sig(uint8_t pin){
    pinMode(pin, OUTPUT);
    io_Continuous_flipping(pin, 5.0f, 100 ,2000);
}



// 触发函数
void biz_start_scan()
{
    ws2812_scan_sig();
    sys_delay_call(ws2812_scan_sig_worning, 7000);   // 2秒后执行
    sys_delay_call(ws2812_staute_green,10000);
}


void biz_err_2_start(){
  // 初始黄色
sys_delay_call(ws2812_all_blink_yellow, 100);

// 交替序列 (t+100 步长)
sys_delay_call(ws2812_all_blink_green,  200); // 绿色
sys_delay_call(ws2812_all_blink_yellow, 300); // 黄色
sys_delay_call(ws2812_all_blink_green,  400); // 绿色
sys_delay_call(ws2812_all_blink_yellow, 500); // 黄色
sys_delay_call(ws2812_all_blink_green,  600); // 绿色
sys_delay_call(ws2812_all_blink_yellow, 700); // 黄色 (结束)
}

void biz_err_2(){
    sys_delay_call(ws2812_err_2, 800); 
}

void biz_err_2_end(){
    sys_delay_call(ws2812_staute_yello,2000);
}