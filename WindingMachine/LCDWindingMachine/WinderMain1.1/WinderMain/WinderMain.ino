/********************************************************************
 * 绕线机一体固件（Winder Firmware）
 * 作者：jiangenze
 * 邮箱：jiangenze1985@hotmail.com
 * 日期：2025-05-01
 * 项目地址：https://github.com/shanhujiao1985/WindingMachine
 *
 * 使用协议（LICENSE）：
 * 本项目源码遵循 Creative Commons Attribution‑NonCommercial 4.0 
 * 国际许可协议（CC BY‑NC 4.0）
 * 
 * 允许自由复制、分发、修改与非商业性使用
 * 禁止任何形式的商业使用（包括销售、租赁、集成等）
 * 商业用途请联系作者获得授权
 * 
 * 协议详情：https://creativecommons.org/licenses/by-nc/4.0/
 *******************************************************************/

#include <Arduino.h>
#include <AccelStepper.h>
#include <OneButton.h>
#include <EEPROM.h>
#include <math.h>

/* ========================== 调试开关 ========================== */
#define DEBUG_LOG 0
#if DEBUG_LOG
#define DBG(x) Serial.print(x)
#define DBGLN(x) Serial.println(x)
#else
#define DBG(x)
#define DBGLN(x)
#endif

/* ========================== 机械常量 ========================== */
/**
 * STEP_ANGLE       ─ 单步角度（度）
 * MICROSTEPS       ─ 驱动细分
 * LEAD_SCREW_PITCH ─ 丝杠导程（mm / rev）
 * GEAR_RATIO       ─ Y 轴齿轮减速比（线轴转 / 步进电机转）
 */
#define STEP_ANGLE 1.8
#define MICROSTEPS_X 4 
#define MICROSTEPS_Y 1 
#define LEAD_SCREW_PITCH 1.0
#define GEAR_RATIO (1.0 / 4.0)

/* --------------------------- 引脚表 --------------------------- */
const int XDIR = 2;      // X 方向
const int XSTEP = 5;     // X 步进
const int YDIR = 3;      // Y 方向（绕线轴）
const int YSTEP = 6;     // Y 步进
const int XLIMIT = 9;    // X 原点限位
const int BREAKW = 10;   // 断线检测（低电平=断线）
const int CTLBTN = 12;   // 起停/暂停按钮
const int STOPBTN = 11;  // 停止/回零按钮（短按 Stop，长按 Home）
const int BUZZ = 13;     // 蜂鸣器
const int EN = 8;        // 步进电机使能（低电平有效）

/* ------------------------- 电机与按钮 ------------------------- */
AccelStepper sx(AccelStepper::DRIVER, XSTEP, XDIR);  // 横移丝杠
AccelStepper sy(AccelStepper::DRIVER, YSTEP, YDIR);  // 绕线主轴
OneButton btn(CTLBTN, true);                         // Start / Pause
OneButton btnStop(STOPBTN, true);                    // Stop / Home

/* ====================== 可调核心参数 ========================= */
/**
 * 所有参数均可通过 Nextion 屏幕配置
 * wireDia      ─ 线径 (mm)
 * totalTurns   ─ 总圈数 (0 = 无限)
 * layerW       ─ 单层宽度 (mm)
 * preTurns     ─ 预绕圈数（起始时先绕多少圈）
 * xZeroOff     ─ 零点偏移 (mm)
 * yMaxRPS      ─ 主轴最高转速 (rev/s)
 */
float wireDia = 1.8f;
uint32_t totalTurns = 0;
float layerW = 58.0f;
uint32_t preTurns = 5;
float xZeroOff = 5.0f;

float yMaxRPS = 2.0f;  // 绕线轴最快速度 (rev/s)
float xMaxRPS = 6.0f;  // 丝杆轴最快速度 (rev/s)
float homeRPS = 6.0f;  // 回零速度 (rev/s)
float accRATIO = 2.0f;   // 加速度 = 最高速度 × 2

/* ===================== 运行派生值 ============================ */
const int   STEPS_REV_X = int(360 / STEP_ANGLE * MICROSTEPS_X);  //X步进电机一圈的步数
const int   STEPS_REV_Y = int(360 / STEP_ANGLE * MICROSTEPS_Y);  //Y步进电机一圈的步数
const float STEPS_MM = STEPS_REV_X  / LEAD_SCREW_PITCH;          //X导轨每毫米的部署

long XSTEP_LINE = 0;   // 绕线轴一圈 X步进电机的步数
long YSTEP_TURN = 0;  // 绕线轴一圈 Y步进电机的步数
int TURNS_LAYER = 0;  // 每层可绕圈数

/* =================== 状态 / 运行时变量 ======================= */
enum St { IDLE,
          RUN,
          PAUSE };
volatile St st = IDLE;

long yLast = 0;                 // 上一次主轴步数
volatile uint32_t turnCnt = 0;  // 已绕圈数
bool layerMoving = false;
long layerStart = 0;  // 当前层起点
long layerEnd = 0;    // 当前层终点

uint32_t lastSentTurnCnt = 0;  // 上次上报的圈数

/* 断线检测计时变量 */
unsigned long breakStartMs = 0;
bool breakTriggered = false;

/* ================= EEPROM 存取相关常量 ======================= */
const float LAYER_TOTAL_LIMIT = 70.0f;  // layerW + xZeroOff ≤ 70
const uint16_t INIT_FLAG_ADDR = 512;
const uint8_t INIT_FLAG_VAL = 0xA5;
const int LANGUAGE_ADDR = INIT_FLAG_ADDR + 1;
const float PARAM_MIN[6] = { 0.10f, 0.0f,     1.0f,   0.0f,  0.0f,  0.2f };
const float PARAM_MAX[6] = { 2.00f, 9999.0f, 70.0f, 100.0f, 69.0f, 2.5f };
uint8_t paramDotPos[6] = { 2, 0, 1, 0, 1, 1 };
uint8_t currentConfig = 0;
short language = 0;

/* ------------------------- EEPROM 结构 ------------------------ */
struct ConfigSlot {
  float fWireDia;
  uint32_t iTotalTurns;
  float fLayerW;
  uint32_t iPreTurns;
  float fXZeroOff;
  float fYMaxRPS;
  uint8_t dotPos[6];
} __attribute__((packed));

constexpr size_t SLOT_SIZE = sizeof(ConfigSlot);
constexpr size_t EEPROM_BASE = 0;

/* ====================== Nextion 工具函数 ====================== */
inline void sendFF() {
  Serial.write(0xFF);
  Serial.write(0xFF);
  Serial.write(0xFF);
}

inline void sendValToNextion(uint8_t idx, uint32_t raw) {
  Serial.print(F("val"));
  Serial.print(idx);
  Serial.print(F(".val="));
  Serial.print(raw);
  sendFF();
}

inline void sendCount(uint32_t cnt) {
  Serial.print(F("count.val="));
  Serial.print(cnt);
  sendFF();
}

inline void loadLanguage(short _language) {
  Serial.print(F("lang.val="));
  Serial.print(_language - 1);
  sendFF();
  Serial.print(F("click btn_lang,1"));
  sendFF();
}

inline void sendBtnState() {
  switch (st) {
    case IDLE:
      Serial.print(F("btn_start.val=0"));
      sendFF();
      Serial.print(F("btn_stop.val=0"));
      sendFF();
      break;
    case RUN:
      Serial.print(F("btn_start.val=1"));
      sendFF();
      Serial.print(F("btn_stop.val=1"));
      sendFF();
      break;
    case PAUSE:
      Serial.print(F("btn_start.val=0"));
      sendFF();
      Serial.print(F("btn_stop.val=1"));
      sendFF();
      break;
  }
}

/**
 * sendCtrlOrder
 * @param objName Nextion 组件对象名
 * @param value   要写入的数值
 */
void sendCtrlOrder(const String objName, int value) {
  Serial.print(objName);
  Serial.print(" ");
  Serial.print(value);
  sendFF();
}

/* ======================= 工具函数 ============================ */
static inline float constrainF(float v, float lo, float hi) {
  return (v < lo) ? lo : (v > hi) ? hi
                                  : v;
}

inline void beepOnce() {
  digitalWrite(BUZZ, HIGH);
  delay(100);
  digitalWrite(BUZZ, LOW);
}
inline void M_ON() {
  digitalWrite(EN, LOW);
}  // 电机上电
inline void M_OFF() {
  digitalWrite(EN, HIGH);
}  // 电机释放


/* ======== X 轴短距离自适应限速 ======== */
const float SHORT_MM_MIN   = 1.0f;   // ≤1 mm 极慢
const float SHORT_MM_MAX   = 5.0f;  // ≥5 mm 恢复正常
const float SHORT_RATIO_MIN= 0.30f;  // 1 mm 时的速度系数
                                          // 10 mm 时系数线性涨到 1.0
void smartMoveX(long targetSteps)
{
  long  distSteps = labs(targetSteps - sx.currentPosition());
  float distMM    = distSteps / STEPS_MM;                 // 步 → mm

  /* 计算速度/加速度系数 */
  float ratio;
  if (distMM <= SHORT_MM_MIN)        ratio = SHORT_RATIO_MIN;
  else if (distMM >= SHORT_MM_MAX)   ratio = 1.0f;
  else {  // 线性插值
    float k = (distMM - SHORT_MM_MIN) / (SHORT_MM_MAX - SHORT_MM_MIN);
    ratio = SHORT_RATIO_MIN + k * (1.0f - SHORT_RATIO_MIN);
  }

  float vMax = homeRPS * STEPS_REV_X * ratio;   // 基础速度 × 系数
  float acc  = vMax * accRATIO;               // 对应加速度

  sx.setMaxSpeed(vMax);
  sx.setAcceleration(acc);
  sx.moveTo(targetSteps);

  while (sx.distanceToGo()) sx.run();
  sx.setCurrentPosition(targetSteps);
}


/* ------------------------------------------------------------- */
/*                       运行期计算函数                           */
/* ------------------------------------------------------------- */
void recalcDerived() {
  XSTEP_LINE   = long(wireDia * STEPS_MM + 0.5);        // 用 X 轴常量
  YSTEP_TURN  = long(STEPS_REV_Y * (1.0f/GEAR_RATIO));              // 用 Y 轴常量
  TURNS_LAYER = int(layerW / wireDia);
}

/* ------------------------------------------------------------- */
/*                          回零 (X)                              */
/* ------------------------------------------------------------- */
void homeX() {
  M_ON();
  sx.setMaxSpeed(homeRPS * STEPS_REV_X);
  sx.setAcceleration(homeRPS * STEPS_REV_X * accRATIO);
  sx.setSpeed(-homeRPS * STEPS_REV_X);  // 反向搜索限位
  while (digitalRead(XLIMIT)) sx.runSpeed();
  sx.setCurrentPosition(0);              // 到达限位，位置=0

  smartMoveX(long(xZeroOff * STEPS_MM)); // 自适应限速
  sx.setCurrentPosition(0);
  M_OFF();
}

/* ---------------- 只回到机械限位，不做偏移 ---------------- */
void homeXToLimit() {
  M_ON();
  sx.setMaxSpeed(homeRPS * STEPS_REV_X);
  sx.setAcceleration(homeRPS * STEPS_REV_X * accRATIO);
  sx.setSpeed(-homeRPS * STEPS_REV_X);      // 反向搜索限位
  while (digitalRead(XLIMIT)) sx.runSpeed();
  sx.setCurrentPosition(0);               // 机械零点 = 0
  M_OFF();
}

/* ------------------------------------------------------------- */
/*                      主轴圈数到位处理                          */
/* ------------------------------------------------------------- */
void arriveTurn() {
  long within = turnCnt % TURNS_LAYER;  // 当前层内的圈索引
  bool odd = (turnCnt / TURNS_LAYER) & 1;

  /* 每层第一圈：准备 X 方向移动 */
  // if (within == 0) {
  //   layerMoving = false;
  //   layerStart = odd ? (TURNS_LAYER - 1) * XSTEP_LINE : 0;
  //   layerEnd = odd ? 0 : (TURNS_LAYER - 1) * XSTEP_LINE;
  //   sx.moveTo(layerStart);
  // }

    /* 进入新层：立刻横移 */
    if (within == 0) {
        layerStart = odd ? (TURNS_LAYER - 1) * XSTEP_LINE : 0;
        layerEnd   = odd ? 0 : (TURNS_LAYER - 1) * XSTEP_LINE;
        sx.moveTo(layerEnd);      // ★直接去另一端
        layerMoving = true;       // 已经在移动
     }



  /* 入第二圈时启动横移 */
  if (within == 1 && !layerMoving) {
    sx.moveTo(layerEnd);
    layerMoving = true;
  }

  /* 达到设定总圈数则停止 */
  if (totalTurns && turnCnt >= totalTurns) {
    sendCount(turnCnt);
    lastSentTurnCnt = turnCnt;
    st = IDLE;
    M_OFF();
    sendBtnState();
    DBGLN("==> FINISH");
  }
}

/* ------------------------------------------------------------- */
/*                    从当前位置开始运行                          */
/* ------------------------------------------------------------- */
void runFromHere() {
 /* ---- 复位断线检测状态 ---- */
  breakStartMs   = 0;
  breakTriggered = false;

  recalcDerived();

  /* 计算 Y 与 X 的匹配速度 */
  float yRPS  = yMaxRPS;
  float ratio = (TURNS_LAYER - 1.0f) / TURNS_LAYER;   // 例如 31 / 32
  float xNeed = XSTEP_LINE * yRPS * ratio;            // ←★ 只改这行

  float xMax  = xMaxRPS * STEPS_REV_X;
  if (xNeed > xMax) {
    yRPS  = xMax / XSTEP_LINE / ratio;  // 要把同一个 ratio 除回去
    xNeed = xMax;
  }
  
  /* 主轴速度参数 */
  sy.setMaxSpeed(yRPS * YSTEP_TURN);
  sy.setAcceleration(yRPS * YSTEP_TURN * accRATIO);
  sy.setSpeed(yRPS * YSTEP_TURN);

  /* X 轴速度参数 */
  sx.setMaxSpeed(xNeed);
  sx.setAcceleration(homeRPS * STEPS_REV_X * accRATIO); //设置较高的启动速度，保证快速启动

  yLast = sy.currentPosition();
  arriveTurn();

  M_ON();
  st = RUN;
  sendBtnState();
  DBGLN("==> RUNNING");
}

/* ------------------------------------------------------------- */
/*                   预绕圈并移动到起始位置                        */
/* ------------------------------------------------------------- */
void prepareStartPos() {
  DBGLN("==> PREPARE START");

  turnCnt = 0;
  sendCount(0);
  lastSentTurnCnt = 0;
  recalcDerived();

  homeXToLimit();     

  long layer  = preTurns / TURNS_LAYER;
  int  within = preTurns % TURNS_LAYER;
  bool odd    = layer & 1;
  int  idx    = odd ? (TURNS_LAYER - 1 - within) : within;

  long workZero = long(xZeroOff * STEPS_MM);         
  long xPos     = workZero + long(idx) * XSTEP_LINE;   


  M_ON();
  smartMoveX(xPos);                      
  sx.setCurrentPosition(long(idx) * XSTEP_LINE);
  M_OFF();

  layerStart = odd ? (TURNS_LAYER - 1) * XSTEP_LINE : 0;
  layerEnd = odd ? 0 : (TURNS_LAYER - 1) * XSTEP_LINE;
  layerMoving = (within > 0);
  turnCnt = preTurns;
  sendCount(turnCnt);
  if (layerMoving) sx.moveTo(layerEnd);

  yLast = 0;
  sy.setCurrentPosition(0);
  beepOnce();
  sendCtrlOrder("beep", 200);
}

/* ------------------------------------------------------------- */
/*                     停止运行并清理状态                          */
/* ------------------------------------------------------------- */
void stopRun(const char* msg) {
  st = IDLE;
  M_OFF();
  sendBtnState();
  sendCtrlOrder("beep", 2000);  // 长响提示
  DBGLN(msg);
}

/* ------------------------------------------------------------- */
/*                      EEPROM 配置读写                           */
/* ------------------------------------------------------------- */
void saveConfig(uint8_t slot) {
  if (slot > 3) return;
  ConfigSlot cfg{ wireDia, totalTurns, layerW, preTurns, xZeroOff, yMaxRPS };
  memcpy(cfg.dotPos, paramDotPos, 6);
  EEPROM.put(EEPROM_BASE + slot * SLOT_SIZE, cfg);
}

bool loadConfig(uint8_t slot) {
  if (slot > 3) return false;

  ConfigSlot cfg;
  EEPROM.get(EEPROM_BASE + slot * SLOT_SIZE, cfg);
  if (cfg.fWireDia < PARAM_MIN[0] || cfg.fWireDia > PARAM_MAX[0]) return false;

  wireDia = cfg.fWireDia;
  totalTurns = cfg.iTotalTurns;
  layerW = cfg.fLayerW;
  preTurns = cfg.iPreTurns;
  xZeroOff = cfg.fXZeroOff;
  yMaxRPS = cfg.fYMaxRPS;
  memcpy(paramDotPos, cfg.dotPos, 6);

  /* 动态限制 layerW  */
  float dynMax = LAYER_TOTAL_LIMIT - xZeroOff;
  layerW = constrainF(layerW, PARAM_MIN[2], dynMax);

  currentConfig = slot;

  /* 更新屏幕数值 */
  for (uint8_t i = 1; i <= 6; ++i) {
    float v = 0;
    switch (i) {
      case 1: v = wireDia; break;
      case 2: v = totalTurns; break;
      case 3: v = layerW; break;
      case 4: v = preTurns; break;
      case 5: v = xZeroOff; break;
      case 6: v = yMaxRPS; break;
    }
    uint32_t raw = round(v * pow(10, paramDotPos[i - 1]));
    sendValToNextion(i, raw);
  }
  return true;
}

void initEEPROMIfNeeded() {
  if (EEPROM.read(INIT_FLAG_ADDR) == INIT_FLAG_VAL) return;
  for (uint8_t s = 0; s < 4; ++s) saveConfig(s);  // 四个槽写入默认值
  EEPROM.update(INIT_FLAG_ADDR, INIT_FLAG_VAL);

  EEPROM.update(LANGUAGE_ADDR, 0);
}

/* ------------------------------------------------------------- */
/*               状态机切换（Start / Pause / Stop / Home）        */
/* ------------------------------------------------------------- */
void setState(St newState) {
  if (newState == st) return;

  if (newState == RUN) {
    if (st == IDLE) {
      prepareStartPos();
    }
    runFromHere();
  } else if (newState == PAUSE && st == RUN) {
    st = PAUSE;
    M_OFF();
    sendBtnState();
    DBGLN("==> PAUSE");
  } else if (newState == IDLE) {
    stopRun("==> STOP");
  }
}

/* -------------------- 按钮回调入口 --------------------------- */
void onCtlBtn() {  // Start / Pause
  switch (st) {
    case IDLE:  // 第一次按：准备 → PAUSE
      saveConfig(currentConfig);
      prepareStartPos();
      st = PAUSE;
      sendBtnState();
      DBGLN("==> PAUSE (READY)");
      break;

    case PAUSE:  // 再按：开始 RUN
      setState(RUN);
      break;

    case RUN:  // 运行中：置 PAUSE
      setState(PAUSE);
      break;
  }
}

void onStopClick() {
  setState(IDLE);
}  // Stop
void onStopLong() {
  homeX();
  stopRun("==> HOME");
}  // Home

/* ------------------------------------------------------------- */
/*                      串口协议解析                               */
/* ------------------------------------------------------------- */
const uint8_t BUFFER_SIZE = 64;
uint8_t buffer[BUFFER_SIZE];
uint8_t bufferIndex = 0;

/**
 * parseBuffer
 * - 00 11 22 ... 77 88 99  字符串命令
 * - 00 11 23 ... 77 88 99  参数写入
 */
void parseBuffer() {
  if (bufferIndex < 9) return;

  /* ---------- 字符串命令 ---------- */
  if (buffer[0] == 0x00 && buffer[1] == 0x11 && buffer[2] == 0x22) {
    String cmd;
    for (uint8_t i = 3; i < bufferIndex - 3; ++i) cmd += char(buffer[i]);

    if (cmd == "Start" || cmd == "Pause") {
      if (cmd == "Start") saveConfig(currentConfig);
      onCtlBtn();
      return;
    }
    if (cmd == "Stop") {
      onStopClick();
      return;
    }
    if (cmd == "Origin") {
      turnCnt = 0;
      lastSentTurnCnt = turnCnt;
      sendCount(turnCnt);
      onStopLong();
      return;
    }

    if (cmd == "pageload") {
      loadConfig(currentConfig);
      loadLanguage(language);
      return;
    }

    if (cmd.startsWith("language")) {
      int idx = cmd.charAt(8) - '0';
      if (idx >= 0 && idx <= 2) {
        language = idx;
        EEPROM.update(LANGUAGE_ADDR, language);
      }
      return;
    }

    if (cmd.startsWith("Config")) {
      int slot = cmd.substring(6).toInt();
      loadConfig(slot);
    }
    return;
  }

  /* ---------- 参数写入 ---------- */
  if (buffer[0] == 0x00 && buffer[1] == 0x11 && buffer[2] == 0x23) {
    if (bufferIndex < 15) return;
    uint8_t idx = buffer[3] | (buffer[4] << 8) | (buffer[5] << 16) | (buffer[6] << 24);
    uint32_t raw = buffer[7] | (buffer[8] << 8) | (buffer[9] << 16) | (buffer[10] << 24);
    uint8_t dot = buffer[11];
    if (dot > 6) dot = 6;
    if (idx == 0 || idx > 6) return;

    float val = raw / pow(10, dot);
    val = constrainF(val, PARAM_MIN[idx - 1], PARAM_MAX[idx - 1]);

    if (idx == 3) {  // layerW
      float dynMax = LAYER_TOTAL_LIMIT - xZeroOff;
      val = constrainF(val, PARAM_MIN[2], dynMax);
      layerW = val;
    } else if (idx == 5) {  // xZeroOff
      xZeroOff = val;
      float dynMax = LAYER_TOTAL_LIMIT - xZeroOff;
      if (layerW > dynMax) {
        layerW = constrainF(layerW, PARAM_MIN[2], dynMax);
        uint32_t rawL = round(layerW * pow(10, paramDotPos[2]));
        sendValToNextion(3, rawL);
      }
    }

    switch (idx) {
      case 1: wireDia = val; break;
      case 2: totalTurns = val; break;
      case 4: preTurns = val; break;
      case 6: yMaxRPS = val; break;
    }
    paramDotPos[idx - 1] = dot;
    sendValToNextion(idx, round(val * pow(10, dot)));
  }
}

/* ------------------------------------------------------------- */
/*                   周期性状态上报（0.5s）                       */
/* ------------------------------------------------------------- */
unsigned long lastStatusMs = 0;
void periodicStatus() {
  if (millis() - lastStatusMs >= 300) {
    lastStatusMs += 300;
    if (st == RUN && turnCnt != lastSentTurnCnt) {
      sendCount(turnCnt);
      lastSentTurnCnt = turnCnt;
    }
  }
}

/* ------------------------------------------------------------- */
/*                            SETUP                               */
/* ------------------------------------------------------------- */
void setup() {
  Serial.begin(115200);

  pinMode(XLIMIT, INPUT_PULLUP);
  pinMode(BREAKW, INPUT_PULLUP);
  pinMode(BUZZ, OUTPUT);
  pinMode(EN, OUTPUT);
  M_OFF();

  btn.attachClick(onCtlBtn);
  btnStop.attachClick(onStopClick);
  btnStop.attachLongPressStart(onStopLong);

  recalcDerived();
  initEEPROMIfNeeded();

  uint8_t e = EEPROM.read(LANGUAGE_ADDR);
  language = (e >= 0 && e <= 2) ? e : 0;

  delay(1000);    // 等待屏幕上电
  loadConfig(0);  // 默认加载槽 0
  loadLanguage(language);
  sendBtnState();

  DBGLN("==> READY");
}

/* ------------------------------------------------------------- */
/*                             LOOP                               */
/* ------------------------------------------------------------- */
void loop() {
  /* 按钮处理 */
  btn.tick();
  btnStop.tick();

  /* 串口接收缓存 */
  while (Serial.available()) {
    uint8_t b = Serial.read();
    if (bufferIndex < BUFFER_SIZE) buffer[bufferIndex++] = b;
    if (bufferIndex >= 3 && buffer[bufferIndex - 3] == 0x77 && buffer[bufferIndex - 2] == 0x88 && buffer[bufferIndex - 1] == 0x99) {
      parseBuffer();
      bufferIndex = 0;
    }
  }

  /* 定时上报 */
  periodicStatus();

  /* ---------- 断线检测（需连续 LOW ≥100ms） ---------- */
  if (st == RUN) {
    if (digitalRead(BREAKW) == LOW) {
      if (breakStartMs == 0) {
        breakStartMs = millis();  // 首次检测断线
      } else if (millis() - breakStartMs >= 100 && !breakTriggered) {
        breakTriggered = true;  // 触发断线
        sendCount(turnCnt);
        stopRun("==> BREAK");
        return;  // 立即停止 loop
      }
    } else {
      breakStartMs = 0;  // 信号恢复
      breakTriggered = false;
    }
  }

  /* 电机运行 */
  if (st != RUN) return;

  sy.runSpeed();  // 主轴恒速
  sx.run();       // X 轴跟随

  long yPos = sy.currentPosition();
  if (yPos - yLast >= YSTEP_TURN) {
    yLast += YSTEP_TURN;
    ++turnCnt;
    arriveTurn();
  }
}
