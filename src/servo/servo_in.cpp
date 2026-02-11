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

// void Servo_active_one(){
//   Y.write(axisSetOffset(Yax, 20 - Yax.D));              // 把绝对角度20转成偏移
//   Z.write(axisSetOffset(Zax, 20 - Zax.D));
//   E.write(axisSetOffset(Eax, (E_Angle_Default-45) - Eax.D));
// }

// void Servo_Shake_R(){
//   R.write(axisReset(Rax));   // 同步：实际=默认，数学也=0偏移
//   delay(100);

//   for(int i=0;i<3;i++){
//     moveToSmooth_Blocking(R,Rax, 70, 10, 1);
//     delay(150);
//     moveToSmooth_Blocking(R,Rax, 30, 10, 1);  
//     delay(150);
//   }

//   Servo_Default();
// }

// void demo(){
//   moveToSmooth_Blocking(R, Rax, 120, 10, 1);
//   moveToSmooth_Blocking(Y, Yax, 60,  10, 1);
//   moveToSmooth_Blocking(Z, Zax, 150, 8,  2);
// }

inline void writeRYZ(){
  R.write(axisAngle(Rax));
  Y.write(axisAngle(Yax));
  Z.write(axisAngle(Zax));
}


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


void stopSequence(){
  player.running = false;
}

bool isSequenceRunning(){
  return player.running;
}


// servo_in.cpp 内部
static void playSequence(const Step3* seq, int n, bool interrupt=true){
  if(interrupt) stopSequence();
  beginSequence(seq, n);
}

void Servo_Update(){
  updateSequence();
}

void Servo_Stop(){
  stopSequence();
}

bool Servo_IsBusy(){
  return isSequenceRunning();
}

static uint32_t lastTrigMs = 0;

static bool canTrigger(){
  uint32_t now = millis();
  if(now - lastTrigMs < 300) return false; // 300ms 防抖
  lastTrigMs = now;
  return true;
}


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






