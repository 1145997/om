#include "servo_in.h"


Servo E; 
Servo Y; 
Servo Z; 
Servo R;
Servo A;
Servo B;
Servo C;

//****************************************//
struct AxisState {
  int D;   // default / origin
  int o;   // offset from origin
};

AxisState Aax{A_Angle_Default, 0};
AxisState Bax{B_Angle_Default, 0};
AxisState Cax{C_Angle_Default, 0};
AxisState Rax{R_Angle_Default, 0};
AxisState Yax{Y_Angle_Default, 0};
AxisState Zax{Z_Angle_Default, 0};
AxisState Eax{E_Angle_Default, 0};




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

//归零姿态
void Servo_zero(){
  R.write(axisSetOffset(Rax, -80));
  Y.write(axisSetOffset(Yax, -80));
  Z.write(axisSetOffset(Zax, -80));
  E.write(axisSetOffset(Eax,-20));
  A.write(axisReset(Aax));
  B.write(axisReset(Bax));
  C.write(axisReset(Cax));
  delay(500);
  R.write(axisReset(Rax));
}


void Servo_Default(){
  R.write(R_Angle_Default);
  E.write(E_Angle_Default);
  Y.write(Y_Angle_Default);
  Z.write(Z_Angle_Default);
  A.write(A_Angle_Default);
  B.write(B_Angle_Default);
  C.write(C_Angle_Default);

  axisReset(Rax);
  axisReset(Eax);
  axisReset(Yax);
  axisReset(Zax);
  axisReset(Aax);
  axisReset(Bax);
  axisReset(Cax);

  delay(1000);
  Servo_zero();
  delay(500);
}



void Servo_init(){
  R.attach(R_Pin);
  E.attach(E_Pin);
  Y.attach(Y_Pin);
  Z.attach(Z_Pin);
  A.attach(A_Pin);
  B.attach(B_Pin);
  C.attach(C_Pin);
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
//{  R,   Y,   Z,  time}

enum { AX_R, AX_Y, AX_Z, AX_E, AX_A, AX_B, AX_C, AX_N };

struct Step3 {
  int rOff;        // R 目标偏移
  int yOff;        // Y 目标偏移
  int zOff;        // Z 目标偏移
  uint16_t durMs;  // 这一段用时（毫秒）
};

struct StepN {
  int16_t off[AX_N];
  uint16_t durMs;
};

// static StepN tmpSeq[64]; // 够用就行（按你最大动作段数调）
// static int   tmpN = 0;

// static const StepN* convert3ToN(const Step3* seq3, int n3){
//   if(n3 > 64) n3 = 64;
//   tmpN = n3;

//   for(int i=0;i<n3;i++){
//     // 全清零
//     for(int k=0;k<AX_N;k++) tmpSeq[i].off[k] = 0;

//     // 只填 R/Y/Z（顺序必须与你 enum/axes 对齐）
//     tmpSeq[i].off[AX_R] = seq3[i].rOff;
//     tmpSeq[i].off[AX_Y] = seq3[i].yOff;
//     tmpSeq[i].off[AX_Z] = seq3[i].zOff;

//     tmpSeq[i].durMs = seq3[i].durMs;
//   }
//   return tmpSeq;
// }



// { R,  Y,  Z,  E,  A,  B,  C,  durMs }


// wave：R/Y/Z 三轴，约 2.7s
const StepN act_wave[] = {
  //        R   Y   Z   E   A   B   C   ms
  KF(       0,  0,  0,  0,  0,  0,  0, 300), // 回中
  KF(     -25, 10, -5,  0,  0,  0,  0, 600),
  KF(      15,-15, 10,  0,  0,  0,  0, 600),
  KF(     -40,  0, 20,  0,  0,  0,  0, 700),
  KF(       0,  0,  0,  0,  0,  0,  0, 500), // 收回
};


// shakeR：R轴，约 2.4s
const StepN act_shakeR[] = {
  KF(  0,0,0,0,0,0,0, 100),
  KF( 70,0,0,0,0,0,0, 400),
  KF( 40,0,0,0,0,0,0, 400),
  KF( 70,0,0,0,0,0,0, 400),
  KF( 40,0,0,0,0,0,0, 400),
  KF( 70,0,0,0,0,0,0, 400),
  KF( 40,0,0,0,0,0,0, 400),
  KF(  0,0,0,0,0,0,0, 300),
};


const StepN act_test[] = {
  KF(   0,   0,   0, 0,0,0,0, 100), // 起始回中
  KF(  70,   0,   0, 0,0,0,0, 400), // 到 70
  KF(  40,   0,   0, 0,0,0,0, 400), // 回 30（即 70->40）
  KF(  70,   0,   0, 0,0,0,0, 400),
  KF(  40, -15,   0, 0,0,0,0, 400),
  KF(  70,  60,   0, 0,0,0,0, 400),
  KF(  40,   0,   0, 0,0,0,0, 400),
  KF(  29,   0,   0, 0,0,0,0, 300),
  KF(  20,   0,   0, 0,0,0,0, 300),
  KF(  50,   0,  60, 0,0,0,0, 300),
  KF( -60,   0,  50, 0,0,0,0, 300),
  KF(  20,   0,   0, 0,0,0,0, 300),
};


const StepN zhizhidiandian[] = {
  // --- 预备 ---
  KF(   0,  -80,  -80, 0,0,0,0, 200),
  KF( -15,   65,   15, 0,0,0,0, 500),
  KF( -10,   55,   25, 0,0,0,0, 500),

  // --- 指点循环 ---
  KF(  18,   55,   18, 0,0,0,0, 220),
  KF(  10,   55,   26, 0,0,0,0, 580),

  KF(  18,   55,   18, 0,0,0,0, 220),
  KF(  10,   55,   26, 0,0,0,0, 580),

  // --- 换方向 ---
  KF(  28,   45,   25, 0,0,0,0, 600),
  KF(  22,   45,   30, 0,0,0,0, 600),

  // --- 再点 ---
  KF(  30,   45,   22, 0,0,0,0, 240),
  KF(  22,   45,   32, 0,0,0,0, 560),

  KF(  30,   45,   22, 0,0,0,0, 240),
  KF(  22,   45,   32, 0,0,0,0, 560),

  // --- 收回 ---
  KF(   0,   30,   40, 0,0,0,0, 400),
  KF(   0,  -80,  -80, 0,0,0,0, 200),
};


const StepN act_nod_ack[] = {
  KF(  0, 40, 60, 0,0,0,0, 300),
  KF(  5, 55, 50, 0,0,0,0, 250),
  KF(  0, 40, 60, 0,0,0,0, 350),
  KF( -5, 55, 50, 0,0,0,0, 250),
  KF(  0, 38, 60, 0,0,0,0, 350),
  KF(  8, 55, 48, 0,0,0,0, 250),
  KF(  0, 35, 60, 0,0,0,0, 500),
  KF(  0,  0, 60, 0,0,0,0, 600),
  KF(  0,  0,  0, 0,0,0,0, 400),
};


const StepN act_scan_look[] = {
  KF(  0, 35, 60, 0,0,0,0, 400),
  KF(-35, 40, 62, 0,0,0,0, 700),
  KF(-15, 32, 58, 0,0,0,0, 600),
  KF(-45, 38, 60, 0,0,0,0, 700),
  KF(  0, 30, 58, 0,0,0,0, 500),
  KF( 35, 40, 62, 0,0,0,0, 700),
  KF( 15, 32, 58, 0,0,0,0, 600),
  KF( 45, 38, 60, 0,0,0,0, 700),
  KF(  0, 30, 60, 0,0,0,0, 500),
  KF(  0,  0,  0, 0,0,0,0, 800),
};


const StepN act_wave_hi[] = {
  KF(  0, 45, 60, 0,0,0,0, 500),
  KF( 30, 45, 60, 0,0,0,0, 350),
  KF(-20, 45, 60, 0,0,0,0, 350),
  KF( 30, 45, 60, 0,0,0,0, 350),
  KF(-20, 45, 60, 0,0,0,0, 350),
  KF( 30, 45, 60, 0,0,0,0, 350),
  KF(-20, 45, 60, 0,0,0,0, 350),
  KF( 10, 35, 60, 0,0,0,0, 500),
  KF(  0,  0,  0, 0,0,0,0, 800),
};


const StepN act_proud_ok[] = {
  KF(  0, 50, 60, 0,0,0,0, 500),
  KF( 15, 55, 58, 0,0,0,0, 220),
  KF(  5, 48, 60, 0,0,0,0, 380),
  KF( 18, 55, 58, 0,0,0,0, 220),
  KF(  0, 45, 60, 0,0,0,0, 450),
  KF( 10, 52, 60, 0,0,0,0, 400),
  KF(  0,  0,  0, 0,0,0,0, 900),
};


const StepN act_confused[] = {
  KF(  0, 30, 60, 0,0,0,0, 400),
  KF(  0, 20, 50, 0,0,0,0, 500),
  KF( 18, 28, 58, 0,0,0,0, 450),
  KF(-18, 28, 58, 0,0,0,0, 450),
  KF( 12, 30, 60, 0,0,0,0, 450),
  KF(-12, 30, 60, 0,0,0,0, 450),
  KF( 25, 40, 55, 0,0,0,0, 500),
  KF(  0, 30, 60, 0,0,0,0, 500),
  KF(  0,  0,  0, 0,0,0,0, 800),
};

// zero：回到你的“归零姿态”（注意：这些是 offset）
const StepN act_zero[] = {
  KF(  0,-80,-80, 80, 0,0,0, 600),
};

const StepN act_demo[] = {
  KF( 120, 60,150, 0,0,0,0, 800),
  KF(   0,  0,  0, 0,0,0,0, 600),
};

const StepN act_firest[] = {
  KF(  -10, -80, -80, 0,0,0,0, 400),
  KF(  0, -60, -60, 0,0,0,0, 1000),
  KF(  0, 30, 30, 0,0,0,0, 2000),
  KF(  0, 30, 30, 0,0,0,0, 2000),
  KF(  0, 30, 30, 0,0,0,0, 2000),
  KF(  -10, -80, -80, 0,0,0,0, 400),
};



// **********************************************
// 动作：空气超标警告（约 5s）
const StepN act_air_warning[] = {

  // --- ① 扫描阶段 ---
  KF(   0,  30,  60, 0,0,0,0, 300),  // 抬起准备
  KF( -35,  35,  60, 0,0,0,0, 600),  // 向左扫描
  KF(  35,  35,  60, 0,0,0,0, 800),  // 向右扫描
  KF(   0,  28,  55, 0,0,0,0, 500),  // 回中并稍微下压

  // --- ② 发现异常（停顿）---
  KF(   0,  20,  50, 0,0,0,0, 600),  // 稍微后仰
  KF(   0,  25,  55, 0,0,0,0, 400),  // 稳住

  // --- ③ “虽然我是铁做的” ---
  KF(  15,  50,  65, 0,0,0,0, 600),  // 抬起+有点得意
  KF(   0,  45,  60, 0,0,0,0, 500),  // 稳一下

  // --- ④ “但你的肺可不是” ---
  KF(  25,  55,  40, 0,0,0,0, 500),  // 前伸（指人）
  KF(  10,  55,  45, 0,0,0,0, 400),  // 小点一下
  KF(   0,   0,   0, 0,0,0,0, 600),  // 收回
};

// 动作：浮空岛灾情播报（约 15s）
// 轴顺序：R Y Z E A B C
const StepN act_report_crash[] = {

  // --- ① 读取委托 / 扫描（0~3s）---
  KF(   0,  25,  55, 0,0,0,0, 400),  // 进入播报姿态
  KF( -25,  28,  58, 0,0,0,0, 800),  // 左侧读取
  KF(  25,  28,  58, 0,0,0,0, 900),  // 右侧读取
  KF(   0,  26,  56, 0,0,0,0, 900),  // 回中锁定

  // --- ② 动力愈况而下（3~6s）：慢慢“塌”下去 ---
  KF(   0,  18,  48, 0,0,0,0, 1200), // 下沉（像电量掉）
  KF(  -8,  16,  45, 0,0,0,0, 800),  // 微摇（不稳）
  KF(   0,  18,  48, 0,0,0,0, 1000), // 勉强回一点

  // --- ③ 毒气攀升（6~9s）：抬头+紧张扫描 ---
  KF(   0,  35,  65, 0,0,0,0, 700),  // 抬头（警觉）
  KF( -35,  38,  62, 0,0,0,0, 700),  // 左扫（“含量攀升”）
  KF(  35,  38,  62, 0,0,0,0, 700),  // 右扫
  KF(   0,  34,  64, 0,0,0,0, 900),  // 回中凝重停顿

  // --- ④ 没有应对手段（9~12s）：无奈+烦躁 ---
  KF(  18,  30,  58, 0,0,0,0, 700),  // 右偏（像“怎么办”）
  KF( -18,  30,  58, 0,0,0,0, 700),  // 左偏
  KF(   0,  28,  56, 0,0,0,0, 600),  // 回中（叹气感）

  // --- ⑤ 再这样下去将坠毁（12~14s）：急下沉+失稳 ---
  KF(  10,  14,  38, 0,0,0,0, 800),  // 明显下坠
  KF( -10,  12,  35, 0,0,0,0, 600),  // 抖一下（结构要散）
  KF(   0,  16,  40, 0,0,0,0, 600),  // 稳住（但很危险）

  // --- ⑥ 点名委托人：涡轮总督（14~15s）：指向定格 ---
  KF(  28,  40,  45, 0,0,0,0, 800),  // 指向 + 定格（点名）
};



//***********************************//
//***********************************//
//***********************************//


//*****************播放器************//



struct MoveRuntimeN {
  bool running = false;
  const StepN* seq = nullptr;
  int n = 0;
  int idx = 0;
  uint32_t segStartMs = 0;

  int16_t start[AX_N];   // 当前段起点 offset
  int16_t delta[AX_N];   // 当前段 delta = target - start
};

static MoveRuntimeN player;
static int16_t outOff[AX_N];   // 本帧输出 offset
















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




struct AxisCfg {
  Servo*     s;
  AxisState* ax;
  int16_t    minOff;   // 安全偏移下限
  int16_t    maxOff;   // 安全偏移上限
};

int clampOffsetSafe(const AxisState &ax, int o, int minOff, int maxOff){
  int lo = max(-ax.D, minOff);
  int hi = min(180 - ax.D, maxOff);
  return clampi(o, lo, hi);
}

void applyOffsets(const AxisCfg* cfg, int axisCount, const int16_t* outOff){
  for(int i=0; i<axisCount; i++){
    AxisState &a = *cfg[i].ax;
    a.o = clampOffsetSafe(a, outOff[i], cfg[i].minOff, cfg[i].maxOff);
    cfg[i].s->write(axisAngle(a));
  }
}




AxisCfg axes[AX_N] = {
  { &R, &Rax, -85, +85 },
  { &Y, &Yax, -85, +85 },
  { &Z, &Zax, -85, +85 },
  { &E, &Eax, -85, +85 },
  { &A, &Aax, -80, +80 },   // 夹爪/执行器按你机构设
  { &B, &Bax, -80, +80 },
  { &C, &Cax, -80, +80 },
};


static void prepareSegment(int idx){
  // start = 当前轴状态
  for(int i=0;i<AX_N;i++){
    player.start[i] = axes[i].ax->o;
  }

  // target = seq[idx].off，先按硬件极限 clampOffset 限制（不是安全范围）
  // 说明：clampOffset 只保证 [-D, 180-D]，不负责安全范围
  for(int i=0;i<AX_N;i++){
    int t = clampOffset(*axes[i].ax, player.seq[idx].off[i]);
    player.delta[i] = (int16_t)(t - player.start[i]);
  }
}

void writeAllCurrent(){
  for(int i=0;i<AX_N;i++){
    axes[i].s->write(axisAngle(*axes[i].ax));
  }
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

  const StepN &st = player.seq[player.idx];

  uint32_t now = millis();
  uint32_t el  = now - player.segStartMs;

  uint16_t dur = st.durMs;
  float u = (dur == 0) ? 1.0f : (float)el / (float)dur;
  if(u > 1.0f) u = 1.0f;

  float e = easeInOut(u);

  // 1) 插值计算 N 轴 offset
  for(int i=0;i<AX_N;i++){
    outOff[i] = (int16_t)(player.start[i] + (int)(player.delta[i] * e));
  }

  // 2) 安全夹紧 + 写舵机（你已经有 applyOffsets）
  applyOffsets(axes, AX_N, outOff);

  // 3) 段结束 -> 切换下一段
  if(u >= 1.0f){
    player.idx++;
    if(player.idx >= player.n){
      player.running = false;
      return false;
    }
    player.segStartMs = now;
    prepareSegment(player.idx);
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
void beginSequence(const StepN* seq, int n){
  player.running = true;
  player.seq = seq;
  player.n = n;
  player.idx = 0;
  player.segStartMs = millis();

  prepareSegment(0);
  updateSequence(); // 立刻输出第一帧
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
static void playSequence(const StepN* seq, int n, bool interrupt=true){
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
  playSequence(act_zero, SEQ_LEN(act_zero));
}

void Servo_PlayShakeR(){
  if(!canTrigger()) return;
  playSequence(act_shakeR, SEQ_LEN(act_shakeR));
  }

void Servo_PlayWave(){
  if(!canTrigger()) return;
  playSequence(act_wave, SEQ_LEN(act_wave));
  }

void Servo_PlayDemo(){
  if(!canTrigger()) return;
  playSequence(act_demo, SEQ_LEN(act_demo));
  }

void Servo_act_test(){
  if(!canTrigger()) return;
  playSequence(act_test, SEQ_LEN(act_test));
}

void Servo_act_test1(){
  if(!canTrigger()) return;
  playSequence(zhizhidiandian, SEQ_LEN(zhizhidiandian));
}

void Servo_PlayNodAck(){
  if(!canTrigger()) return;
  playSequence(act_nod_ack, SEQ_LEN(act_nod_ack));
}

void Servo_PlayScan(){
  if(!canTrigger()) return;
  playSequence(act_scan_look, SEQ_LEN(act_scan_look));
}

void Servo_PlayWaveHi(){
  if(!canTrigger()) return;
  playSequence(act_wave_hi, SEQ_LEN(act_wave_hi));
}

void Servo_PlayConfused(){
  if(!canTrigger()) return;
  playSequence(act_confused, SEQ_LEN(act_confused));
}

void Servo_PlayProudOK(){
  if(!canTrigger()) return;
  playSequence(act_proud_ok, SEQ_LEN(act_proud_ok));
}

void Servo_act_firest(){
  if(!canTrigger()) return;
  playSequence(act_report_crash, SEQ_LEN(act_report_crash));
}

void Servo_act_air_warning(){ 
  if(!canTrigger()) return;
  playSequence(act_air_warning, SEQ_LEN(act_air_warning)); 
}

void Servo_act_report_crash(){ 
  if(!canTrigger()) return;
  playSequence(act_report_crash, SEQ_LEN(act_report_crash)); 
}

















