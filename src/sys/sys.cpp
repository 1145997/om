#include "sys.h"



// ========================
// 非阻塞任务系统
// ========================

namespace {

static constexpr uint8_t MAX_RUNNING_JOBS = 6;

struct JobRuntime {
  bool active = false;
  const SysStep* steps = nullptr;
  uint16_t count = 0;
  uint16_t cursor = 0;       // 下一个要执行的 step 下标
  uint32_t start_ms = 0;     // job 启动时刻
};

static JobRuntime g_jobs[MAX_RUNNING_JOBS];

// 返回 job_id，失败返回 -1
int sys_job_start(const SysJob& job)
{
  for (int i = 0; i < MAX_RUNNING_JOBS; i++) {
    if (!g_jobs[i].active) {
      g_jobs[i].active   = true;
      g_jobs[i].steps    = job.steps;
      g_jobs[i].count    = job.count;
      g_jobs[i].cursor   = 0;
      g_jobs[i].start_ms = millis();
      return i;
    }
  }
  return -1;
}

void sys_job_cancel(int job_id)
{
  if (job_id < 0 || job_id >= MAX_RUNNING_JOBS) return;
  g_jobs[job_id] = JobRuntime{};
}

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


static void sys_delay_service(uint32_t now) {   
  for(int i=0;i<MAX_DELAY_TASKS;i++){
        auto &t = g_delay_tasks[i];
        if(!t.used || !t.active) continue;

        if((int32_t)(now - t.trigger_time) >= 0)
        {
            t.active = false;
            if(t.callback) t.callback();
        }
    } 
  }   // 你第一段



static void sys_ioflip_service(uint32_t now) { 
  
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
}  // 你第二段

static void sys_job_service(uint32_t now) { 
  
  for (int i = 0; i < MAX_RUNNING_JOBS; i++) {
    auto &jr = g_jobs[i];
    if (!jr.active) continue;

    // 连续执行：如果当前时间已经超过了多个 step 的时间点，就一次补齐执行
    while (jr.cursor < jr.count) {
      const SysStep &st = jr.steps[jr.cursor];
      uint32_t due = jr.start_ms + st.offset_ms;

      if ((int32_t)(now - due) < 0) break; // 还没到

      if (st.func) st.func();
      jr.cursor++;
    }

    // 跑完自动结束
    if (jr.cursor >= jr.count) {
      jr.active = false;
    }
  }
 }     // 你第三段
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
  uint32_t now = millis();
  sys_delay_service(now);
  sys_ioflip_service(now);
  sys_job_service(now);
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



//================================================================================
//================================================================================
//================================================================================
//================================================================================

static const SysStep JOB_0x05_AIR_WARNING_STEPS[] = {
  { ws2812_all_breath_yellow,  0     },  // 预警：开始检测
  { fx_detecting,              200   },  // 检测氛围
  { fx_emergency,              2500  },  // 升级：警报
  { ws2812_all_blink_red,      2600  },  // 强烈闪红
  { ws2812_all_breath_yellow,  5200  },  // 降级：回到警戒黄
  { ws2812_staute_green,       8000  },  // 结束：回绿
};
static const SysJob JOB_0x05_AIR_WARNING = { JOB_0x05_AIR_WARNING_STEPS, JOB_COUNT(JOB_0x05_AIR_WARNING_STEPS) };

void biz_start_air_warning()
{
  sys_job_start(JOB_0x05_AIR_WARNING);
}

//================================================================================
//================================================================================
//================================================================================
//================================================================================




static const SysStep JOB_0x06_CRASH_REPORT_STEPS[] = {
  { fx_battle,            0     },   // 灾难/战斗氛围
  { ws2812_all_flow_red,  200   },
  { fx_emergency,         1600  },   // 警报叠加
  { ws2812_all_blink_red, 1700  },
  { ws2812_all_blink_yellow, 4200 }, // “系统抖动/临界”
  { ws2812_staute_red,    6500  },   // 维持红常亮（持续紧张）
};
static const SysJob JOB_0x06_CRASH_REPORT = { JOB_0x06_CRASH_REPORT_STEPS, JOB_COUNT(JOB_0x06_CRASH_REPORT_STEPS) };

void biz_start_crash_report()
{
  sys_job_start(JOB_0x06_CRASH_REPORT);
}
//================================================================================
//================================================================================
//================================================================================
//================================================================================

void biz_scan_sig(uint8_t pin){
    pinMode(pin, OUTPUT);
    io_Continuous_flipping(pin, 5.0f, 100 ,2000);
}


static const SysStep JOB_0x07_GAS_WAVE_SCAN_STEPS[] = {
  { ws2812_scan_sig,          0     }, // A/B/C 绿呼吸扫描
  { fx_detecting,             300   }, // 检测氛围
  { ws2812_scan_sig_worning,  7000  }, // 7s：波形异常警告
  { fx_emergency,             7100  },
  { fx_detect_done,           10000 }, // 10s：分析完成/回绿
  { ws2812_staute_green,      11000 },
};
static const SysJob JOB_0x07_GAS_WAVE_SCAN = { JOB_0x07_GAS_WAVE_SCAN_STEPS, JOB_COUNT(JOB_0x07_GAS_WAVE_SCAN_STEPS) };
void biz_start_gas_wave_scan(uint8_t pin)
{
  biz_scan_sig(pin);
  sys_job_start(JOB_0x07_GAS_WAVE_SCAN);
}

//================================================================================
//================================================================================
//================================================================================
//================================================================================




static const SysStep JOB_0x08_DISMANTLE_MYTH_STEPS[] = {
  { ws2812_all_breath_yellow, 0     }, // 思考/怀疑
  { fx_glitch_overload,       1200  }, // 机械摩擦/噪声感
  { ws2812_all_breath_yellow, 2500  },
  { ws2812_staute_green,      5000  }, // 结论稳定
};
static const SysJob JOB_0x08_DISMANTLE_MYTH = { JOB_0x08_DISMANTLE_MYTH_STEPS, JOB_COUNT(JOB_0x08_DISMANTLE_MYTH_STEPS) };
void biz_start_dismantle_myth()
{
  sys_job_start(JOB_0x08_DISMANTLE_MYTH);
}
//================================================================================
//================================================================================
//================================================================================
//================================================================================



static const SysStep JOB_0x09_BLOW_BOX_STEPS[] = {
  { ws2812_all_breath_red,  0     }, // 蓄力
  { ws2812_all_flow_red,    1500  }, // 冲击感
  { ws2812_all_blink_red,   3200  }, // “爆”
  { ws2812_staute_red,      3800  }, // 余震
  { ws2812_staute_green,    6500  }, // 回常态
};
static const SysJob JOB_0x09_BLOW_BOX = { JOB_0x09_BLOW_BOX_STEPS, JOB_COUNT(JOB_0x09_BLOW_BOX_STEPS) };
void biz_start_blow_box()
{
  sys_job_start(JOB_0x09_BLOW_BOX);
}

//================================================================================
//================================================================================
//================================================================================
//================================================================================




static const SysStep JOB_0x0B_EMERGENCY_OXYGEN_STEPS[] = {
  { ws2812_all_breath_yellow, 0     },  // 进入应急流程
  { fx_emergency,             1800  },  // 浓度加剧
  { ws2812_all_blink_red,      1900  },
  { fx_override_running,       4200  },  // 启用应急制氧/覆写运行感
  { fx_override_done,          9500  },  // 稳定
  { ws2812_staute_green,       10500 },
};
static const SysJob JOB_0x0B_EMERGENCY_OXYGEN = { JOB_0x0B_EMERGENCY_OXYGEN_STEPS, JOB_COUNT(JOB_0x0B_EMERGENCY_OXYGEN_STEPS) };
void biz_start_emergency_oxygen()
{
  sys_job_start(JOB_0x0B_EMERGENCY_OXYGEN);
}


//================================================================================
//================================================================================
//================================================================================
//================================================================================



static const SysStep JOB_0x0C_AI_PARTY_DIZZY_STEPS[] = {
  { ws2812_err_2,              0     }, // 彩虹cycle：派对感
  { ws2812_all_flow_yellow,    2500  }, // 眩晕/摇摆
  { ws2812_all_breath_yellow,  5000  }, // 迷糊收束
  { ws2812_staute_green,       8000  },
};
static const SysJob JOB_0x0C_AI_PARTY_DIZZY = { JOB_0x0C_AI_PARTY_DIZZY_STEPS, JOB_COUNT(JOB_0x0C_AI_PARTY_DIZZY_STEPS) };
void biz_start_ai_party_dizzy()
{
  sys_job_start(JOB_0x0C_AI_PARTY_DIZZY);
}
//================================================================================
//================================================================================
//================================================================================
//================================================================================

static const SysStep JOB_0x0D_GLITCH_SPASM_STEPS[] = {
  { fx_glitch_overload,     0     },  // 过载故障氛围
  { ws2812_err_2,            900   },  // 彩虹抽风
  { ws2812_all_blink_yellow, 3200  },  // 乱码黄闪
  { ws2812_all_blink_red,    4800  },  // 危险红闪
  { ws2812_staute_yello,     6500  },  // 余震黄
  { ws2812_staute_green,     9000  },  // 回绿
};
static const SysJob JOB_0x0D_GLITCH_SPASM = { JOB_0x0D_GLITCH_SPASM_STEPS, JOB_COUNT(JOB_0x0D_GLITCH_SPASM_STEPS) };
void biz_start_glitch_spasm()
{
  sys_job_start(JOB_0x0D_GLITCH_SPASM);
}


//================================================================================
//================================================================================
//================================================================================
//================================================================================


static const SysStep JOB_0x0E_POINT_KNOBS_STEPS[] = {
  { fx_detecting,    0    },
  { fx_detect_done,  6000 },  // 20s 舵机动作里，灯先 6s 给到“分析完成”提示
  { ws2812_staute_green, 8000 },
};
static const SysJob JOB_0x0E_POINT_KNOBS = { JOB_0x0E_POINT_KNOBS_STEPS, JOB_COUNT(JOB_0x0E_POINT_KNOBS_STEPS) };

void biz_start_point_knobs()
{
  sys_job_start(JOB_0x0E_POINT_KNOBS);
}
//================================================================================
//================================================================================
//================================================================================
//================================================================================


static const SysStep JOB_0x0F_DOUBT_STEPS[] = {
  { ws2812_all_breath_yellow, 0    },
  { ws2812_staute_green,      3000 },
};
static const SysJob JOB_0x0F_DOUBT = { JOB_0x0F_DOUBT_STEPS, JOB_COUNT(JOB_0x0F_DOUBT_STEPS) };

void biz_start_doubt()
{
  sys_job_start(JOB_0x0F_DOUBT);
}

//================================================================================
//================================================================================
//================================================================================
//================================================================================


static const SysStep JOB_0x10_NAV_PORT_STEPS[] = {
  { ws2812_all_flow_green, 0    },
  { ws2812_staute_green,   5000 },
};
static const SysJob JOB_0x10_NAV_PORT = { JOB_0x10_NAV_PORT_STEPS, JOB_COUNT(JOB_0x10_NAV_PORT_STEPS) };

void biz_start_nav_port()
{
  sys_job_start(JOB_0x10_NAV_PORT);
}


//================================================================================
//================================================================================
//================================================================================
//================================================================================

void biz_err_3(uint8_t pin) {
  pinMode(pin, OUTPUT);
  io_Continuous_flipping(pin, 1.0f, 1,2000);
}

static const SysStep JOB_0x11_NERVOUS_APOLOGY_STEPS[] = {
  { ws2812_all_blink_yellow, 0    },
  { ws2812_all_breath_yellow,1800 },
  { ws2812_staute_green,     6000 },
};
static const SysJob JOB_0x11_NERVOUS_APOLOGY = { JOB_0x11_NERVOUS_APOLOGY_STEPS, JOB_COUNT(JOB_0x11_NERVOUS_APOLOGY_STEPS) };

void biz_start_nervous_apology(uint8_t pin)
{
  biz_err_3(pin);
  sys_job_start(JOB_0x11_NERVOUS_APOLOGY);
}

//================================================================================
//================================================================================
//================================================================================
//================================================================================

static const SysStep JOB_0x12_ACCUSATION_STEPS[] = {
  { ws2812_all_breath_red,  0    },
  { fx_battle,              1200 },
  { ws2812_all_blink_red,   4500 },
  { ws2812_staute_red,      6000 },
};
static const SysJob JOB_0x12_ACCUSATION = { JOB_0x12_ACCUSATION_STEPS, JOB_COUNT(JOB_0x12_ACCUSATION_STEPS) };

void biz_start_accusation()
{
  sys_job_start(JOB_0x12_ACCUSATION);
}

//================================================================================
//================================================================================
//================================================================================
//================================================================================

static const SysStep JOB_0x13_POINT_POWER_STEPS[] = {
  { ws2812_all_blink_yellow, 0    }, // 提示/指向
  { ws2812_all_flow_yellow,  800  },
  { ws2812_staute_green,     2500 },
};
static const SysJob JOB_0x13_POINT_POWER = { JOB_0x13_POINT_POWER_STEPS, JOB_COUNT(JOB_0x13_POINT_POWER_STEPS) };

void biz_start_point_power()
{
  sys_job_start(JOB_0x13_POINT_POWER);
}

//================================================================================
//================================================================================
//================================================================================
//================================================================================

static const SysStep JOB_0x14_OVERLOAD_OVERRIDE_STEPS[] = {
  { fx_emergency,         0     },
  { ws2812_all_blink_red,  200  },
  { fx_override_running,   2500 },
  { fx_override_done,      12000 },
  { ws2812_staute_green,   13000 },
};
static const SysJob JOB_0x14_OVERLOAD_OVERRIDE = { JOB_0x14_OVERLOAD_OVERRIDE_STEPS, JOB_COUNT(JOB_0x14_OVERLOAD_OVERRIDE_STEPS) };

void biz_start_overload_override()
{
  sys_job_start(JOB_0x14_OVERLOAD_OVERRIDE);
}



















