#include "servo_in.h"


Servo E; 
Servo Y; 
Servo Z; 
Servo R;

//****************************************//
struct AxisState {
  int D;   // default / origin
  int o;   // offset from origin
};

int clampi(int v, int lo, int hi){
  if(v < lo) return lo;
  if(v > hi) return hi;
  return v;
}

// 把 (D, o) 转成舵机真实角度，并把 o 校正到合法范围（纯数学）
int axisAngle(AxisState &ax){
  int a = clampi(ax.D + ax.o, 0, 180);
  ax.o = a - ax.D;          // 关键：clamp 后回写，状态不漂
  return a;
}

// 绝对设置偏移：o := targetOffset
int axisSetOffset(AxisState &ax, int targetOffset){
  ax.o = targetOffset;
  return axisAngle(ax);
}

// 增量更新偏移：o := o + delta
int axisAddOffset(AxisState &ax, int delta){
  ax.o += delta;
  return axisAngle(ax);
}

// 复位到原点：o := 0
int axisReset(AxisState &ax){
  ax.o = 0;
  return axisAngle(ax);
}

AxisState Rax{R_Angle_Default, 0};
AxisState Yax{Y_Angle_Default, 0};
AxisState Zax{Z_Angle_Default, 0};
AxisState Eax{E_Angle_Default, 0};

//****************************************//
int clampOffset(const AxisState &ax, int o){
  return clampi(o, -ax.D, 180 - ax.D);
}
/**
 * @brief  平滑移动舵机到指定“偏移位置”（以默认角度为原点）
 *
 * @param  s              舵机对象引用（例如 R / Y / Z / E）
 *
 * @param  ax             轴状态结构体引用（包含：
 *                        D = 默认角度（原点）
 *                        o = 当前偏移量）
 *                        函数会实时更新 ax.o
 *
 * @param  targetOffset   目标偏移量（单位：度）
 *                        实际输出角度 = ax.D + targetOffset
 *                        合法范围：[-ax.D , 180 - ax.D]
 *                        超出范围会自动 clamp
 *
 * @param  stepDelayMs    每一步之间的延时（毫秒）
 *                        越小运动越快
 *                        推荐范围：5~15ms
 *
 * @param  stepSize       每一步移动的偏移步长（单位：度）
 *                        越小越平滑
 *                        推荐值：1~2
 *
 * @note   本函数基于“数学偏移模型”
 *         角度计算公式：angle = D + o
 *         全程使用 offset 计算，不直接操作绝对角度
 *
 * @warning 如果 stepSize 过大可能导致运动不够平滑
 *          如果 stepDelayMs 过小可能导致舵机抖动
 */
void moveToSmooth_Blocking(Servo &s, AxisState &ax, int targetOffset, int stepDelayMs, int stepSize){
  if(stepSize < 1) stepSize = 1;

  targetOffset = clampOffset(ax, targetOffset);

  while(ax.o != targetOffset){
    int diff = targetOffset - ax.o;

    if(abs(diff) <= stepSize) ax.o = targetOffset;
    else                      ax.o += (diff > 0 ? stepSize : -stepSize);

    s.write(axisAngle(ax));
    delay(stepDelayMs);
  }
}



void Servo_Default(){
  R.write(R_Angle_Default);
  E.write(E_Angle_Default);
  Y.write(Y_Angle_Default);
  Z.write(Z_Angle_Default);
  axisReset(Rax);
  axisReset(Eax);
  axisReset(Yax);
  axisReset(Zax);
  delay(1000);
}

void Servo_zero(){
  R.write(axisSetOffset(Rax, -80));
  Y.write(axisSetOffset(Yax, -80));
  Z.write(axisSetOffset(Zax, -80));
  E.write(axisSetOffset(Eax, +80));
}

void Servo_init(){
  R.attach(R_Pin);
  E.attach(E_Pin);
  Y.attach(Y_Pin);
  Z.attach(Z_Pin);
  delay(50);
  Servo_Default();
}

inline void writeRYZ(){
  R.write(axisAngle(Rax));
  Y.write(axisAngle(Yax));
  Z.write(axisAngle(Zax));
}


//*****************动作****************//
//*****************动作****************//
//*****************动作****************//

struct Step3 {
  int rOff;        // R 目标偏移
  int yOff;        // Y 目标偏移
  int zOff;        // Z 目标偏移
  uint16_t durMs;  // 这一段用时（毫秒）
};

const Step3 act_wave[] = {
  {  0,   0,   0,  300},   // 回到中位（偏移0）
  {-25,  10,  -5,  600},
  { 15, -15,  10,  600},
  {-40,   0,  20,  700},
  {  0,   0,   0,  500},   // 收回
};

const Step3 act_shakeR[] = {
  {  0, 0, 0, 100},   // 起始回中
  { 70, 0, 0, 400},   // 到 70
  { 40, 0, 0, 400},   // 回 30（即 70->40）
  { 70, 0, 0, 400},
  { 40, 0, 0, 400},
  { 70, 0, 0, 400},
  { 40, 0, 0, 400},
  {  0, 0, 0, 300},   // 收回
};

const Step3 act_zero[] = {
  { -80, -80, -80, 600 },
};

const Step3 act_demo[] = {
  { 120,  60, 150, 800 },
  {   0,   0,   0, 600 },
};




//***********************************//
//***********************************//
//***********************************//


//*****************播放器************//

float easeInOut(float t){ return t*t*(3.0f - 2.0f*t); }

struct MoveRuntime3 {
  bool running = false;
  const Step3* seq = nullptr;
  int n = 0;
  int idx = 0;

  uint32_t segStartMs = 0;

  int rStart=0, yStart=0, zStart=0;
  int rDelta=0, yDelta=0, zDelta=0;
};

MoveRuntime3 player;


int clampOffsetSafe(const AxisState &ax, int o, int minOff, int maxOff){
  int lo = max(-ax.D, minOff);
  int hi = min(180 - ax.D, maxOff);
  return clampi(o, lo, hi);
}

/**
 * @brief  播放器核心更新函数（非阻塞）
 *
 * @return true   当前仍在播放动作
 * @return false  当前没有播放或已播放完成
 *
 * @details
 * 该函数基于 millis() 推进当前动作序列的时间。
 * 每次调用会：
 *   1. 计算当前段的插值进度 u (0~1)
 *   2. 通过缓动函数 easeInOut(u) 得到平滑比例
 *   3. 根据起点 + delta 插值计算新的 offset
 *   4. 写入舵机
 *   5. 若当前段结束，自动切换到下一段
 *
 * 该函数必须在 loop() 中持续调用，否则动作会停止。
 *
 * 运动模型：
 *   offset = start + delta * ease(t)
 *
 * 安全机制：
 *   使用 clampOffsetSafe 限制每轴偏移范围，避免机械顶死。
 *
 * 状态机逻辑：
 *   running == false 时立即返回
 *   idx >= n 时自动结束播放
 */
bool updateSequence(){
  if(!player.running) return false;

  const Step3 &st = player.seq[player.idx];
  uint32_t now = millis();
  uint32_t el = now - player.segStartMs;

  uint16_t dur = st.durMs;
  float u = (dur == 0) ? 1.0f : (float)el / (float)dur;
  if(u > 1.0f) u = 1.0f;

  float e = easeInOut(u);

  // 插值计算 offset
  int ro = player.rStart + (int)(player.rDelta * e);
  int yo = player.yStart + (int)(player.yDelta * e);
  int zo = player.zStart + (int)(player.zDelta * e);

  Rax.o = clampOffsetSafe(Rax, ro, -60, +60);
  Yax.o = clampOffsetSafe(Yax, yo, -60, +60);
  Zax.o = clampOffsetSafe(Zax, zo, -60, +60);

  writeRYZ();

  // 段结束：切下一段
  if(u >= 1.0f){
    player.idx++;
    if(player.idx >= player.n){
      player.running = false;   // ✅ 整套执行完毕
      return false;
    }

    // 下一段准备
    player.segStartMs = now;

    player.rStart = Rax.o; player.yStart = Yax.o; player.zStart = Zax.o;

    int rt = clampOffset(Rax, player.seq[player.idx].rOff);
    int yt = clampOffset(Yax, player.seq[player.idx].yOff);
    int zt = clampOffset(Zax, player.seq[player.idx].zOff);

    player.rDelta = rt - player.rStart;
    player.yDelta = yt - player.yStart;
    player.zDelta = zt - player.zStart;
  }

  return true;
}


/**
 * @brief  启动播放一套动作序列
 *
 * @param  seq  指向动作数组（Step3）
 * @param  n    动作段数量
 *
 * @details
 * 该函数初始化播放器状态：
 *   - 设置当前动作序列
 *   - 重置段索引 idx
 *   - 记录当前 offset 作为起点
 *   - 计算第一段的 delta
 *   - 立即输出第一帧（避免启动延迟）
 *
 * 若需要抢占旧动作，应在调用前执行 stopSequence()。
 *
 * 注意：
 *   该函数不会做防抖或忙碌判断。
 *   建议通过 playSequence() 统一入口调用。
 */
void beginSequence(const Step3* seq, int n){
  player.running = true;
  player.seq = seq;
  player.n = n;
  player.idx = 0;
  player.segStartMs = millis();

  // 当前段起点 = 当前位置
  player.rStart = Rax.o; player.yStart = Yax.o; player.zStart = Zax.o;

  // 当前段终点 = 第0段目标
  int rt = clampOffset(Rax, seq[0].rOff);
  int yt = clampOffset(Yax, seq[0].yOff);
  int zt = clampOffset(Zax, seq[0].zOff);

  player.rDelta = rt - player.rStart;
  player.yDelta = yt - player.yStart;
  player.zDelta = zt - player.zStart;

  updateSequence();
}

/**
 * @brief  强制停止当前动作播放
 *
 * @details
 * 仅修改播放器状态，不改变舵机当前姿态。
 * 若希望停止后回到默认姿态，请调用 Servo_PlayZero()。
 */
void stopSequence(){
  player.running = false;
}

/**
 * @brief  查询当前是否正在播放动作
 *
 * @return true   正在播放
 * @return false  空闲
 *
 * @details
 * 可用于：
 *   - 防止重复触发
 *   - 实现非抢占模式
 *   - 等待当前动作播放完成
 */
bool isSequenceRunning(){
  return player.running;
}

/**
 * @brief  内部统一播放入口
 *
 * @param  seq        动作数组
 * @param  n          动作段数量
 * @param  interrupt  是否抢占当前动作（默认 true）
 *
 * @details
 * 该函数封装 stop + begin 逻辑，
 * 用于所有对外动作接口的统一入口。
 *
 * interrupt == true:
 *     立即停止当前动作并开始新动作（抢占模式）
 *
 * interrupt == false:
 *     若当前正在播放，则忽略本次触发
 */
static void playSequence(const Step3* seq, int n, bool interrupt=true){
  if(interrupt) stopSequence();
  beginSequence(seq, n);
}


/**
 * @brief  播放器维护函数（必须在 loop() 中持续调用）
 *
 * @details
 * 该函数负责驱动当前动作序列的插值计算与舵机输出。
 * 内部基于 millis() 进行非阻塞时间推进。
 *
 * 若当前没有动作在播放，则函数内部会立即返回。
 *
 * @note
 * 必须在主循环中高频调用，否则动作会停止在当前帧。
 *
 * @example
 * void loop(){
 *   Servo_Update();
 * }
 */

void Servo_Update(){
  updateSequence();
}

/**
 * @brief  强制停止当前动作序列
 *
 * @details
 * 立即终止当前播放器状态，不会自动回到默认姿态。
 * 若需要回中，请在外部调用 Servo_PlayZero() 或其他动作。
 *
 * @note
 * 该函数不会改变当前舵机位置，只是停止插值推进。
 */
void Servo_Stop(){
  stopSequence();
}

/**
 * @brief  查询播放器是否正在执行动作
 *
 * @return true   当前有动作正在播放
 * @return false  当前处于空闲状态
 *
 * @details
 * 可用于：
 * - 防止重复触发动作
 * - 判断是否允许切换新动作
 * - 实现“非抢占式”控制逻辑
 */
bool Servo_IsBusy(){
  return isSequenceRunning();
}

/**
 * @brief  动作触发时间防抖变量
 *
 * @details
 * 记录上一次成功触发动作的时间戳（毫秒）。
 * 用于防止语音识别等连续触发导致动作反复重启。
 */
static uint32_t lastTrigMs = 0;


/**
 * @brief  动作触发防抖检测
 *
 * @return true   允许触发新动作
 * @return false  触发间隔过短，被抑制
 *
 * @details
 * 通过 millis() 计算两次触发的时间差。
 * 若间隔小于 300ms，则认为是重复触发，不执行。
 *
 * 适用于语音识别、按键抖动、串口重复数据等场景。
 *
 * @note
 * 如需调整灵敏度，可修改 300ms 阈值。
 */
static bool canTrigger(){
  uint32_t now = millis();
  if(now - lastTrigMs < 300) return false; // 300ms 防抖
  lastTrigMs = now;
  return true;
}


//*********************注册动作函数*********************************//



void Servo_PlayZero(){
  if(!canTrigger()) return;
  playSequence(act_zero, sizeof(act_zero)/sizeof(act_zero[0]));
}

void Servo_PlayShakeR(){
  if(!canTrigger()) return;
  playSequence(act_shakeR, sizeof(act_shakeR)/sizeof(act_shakeR[0]));
}

void Servo_PlayWave(){
  if(!canTrigger()) return;
  playSequence(act_wave, sizeof(act_wave)/sizeof(act_wave[0]));
}

void Servo_PlayDemo(){
  if(!canTrigger()) return;
  playSequence(act_demo, sizeof(act_demo)/sizeof(act_demo[0]));
}
