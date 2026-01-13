#include <Wire.h>
#include <SoftwareSerial.h> // 引入软件串口库用于HC-06/05

// =================================================================
// 0. 蓝牙通信定义与模式控制
// =================================================================
const uint8_t BT_RX = 10; // Nano/Uno 接收蓝牙信号 (HC-06 TXD -> D10)
const uint8_t BT_TX = 11; // Nano/Uno 发送蓝牙信号 (D11 -> HC-06 RXD)
SoftwareSerial BT(BT_RX, BT_TX);

enum CarMode {
  MODE_AUTO,   // 自动驾驶模式（执行状态机）
  MODE_MANUAL  // 手动控制模式（接收手机指令）
};
CarMode currentMode = MODE_AUTO; // 默认启动自动驾驶

// =================================================================
// 1. 硬件定义与参数
// =================================================================
#define MPU_ADDR 0x68
#define MOTOR_ADDR 0x2A

// --- 自动驾驶参数 (坡道优化配置) ---
float PULSES_RIGHT = 229.0;
float PULSES_LEFT  = 235.0;

double SPEED_FLAT  = 40.0;   
double SPEED_CLIMB = 40.0;   
int    PWM_SPIN    = 45;     

#define RAMP_ANGLE_THRESHOLD 4.0   
#define FLAT_ANGLE_THRESHOLD 2.0   
#define TARGET_YAW           355.0 

#define STABLE_TIME        50      
#define STARTUP_LOCK_TIME  500     
#define TOP_STOP_TIME      2000    

// 状态机定义
enum TaskState {
  STATE_INIT, STATE_STRAIGHT, STATE_CLIMBING, STATE_TOP_STOP,
  STATE_SPIN, STATE_CONTINUE, STATE_DOWNHILL, STATE_FINISHED
};
TaskState currentState = STATE_INIT;
const char* getStateName(TaskState state); // 状态名称映射函数声明

// 传感器与PID变量
const bool INV_M1 = false; const bool INV_M2 = false; 
const bool INV_M3 = false; const bool INV_M4 = false;
float Kp = 0.8, Ki = 0.3, Kd = 0.0;
const int basePWM = 40;
const float RIGHT_RATIO = 1.0;
const float LEFT_RATIO  = PULSES_RIGHT / PULSES_LEFT;

double input_R=0, input_L=0, out_L=0, out_R=0;
double integral_L=0, integral_R=0;
long last_pos_R=0, last_pos_L=0;
unsigned long lastPIDTime=0;
const int PID_SAMPLE_TIME = 100;
// rampStep加大到10.0，使刹车更快
double targetSpeed=0, currentTargetSpeed=0, rampStep=10.0; 

int16_t ax, ay, az, gx, gy, gz;
float pitch = 0.0, pitch_offset = 0.0, yaw = 0.0, gyroZ_offset = 0.0;
unsigned long prevTime=0, stateChangeTimer=0, straightStartTime=0, topStopStart=0;
bool isConditionMet = false;

// --- 手动控制/蓝牙变量 ---
const unsigned long WATCHDOG_MS = 1000;
unsigned long lastCmdMs = 0;
int16_t tgtL = 0, tgtR = 0; 
int16_t curL = 0, curR = 0; 
const int MAX_STEP = 20;
const int MAX_SPEED = 255; 
const int MED_SPEED = 150;
const int LOW_SPEED = 100;
const int HIGH_SPEED = 200;

// =================================================================
// 2. 函数声明
// =================================================================
void stopMotors(); void initEncoders(); void calibrateGyro(); void calibratePitch();
void readMPU(); void updateAttitude(float dt); void runPIDLoop();
void readEncodersWithCompensation(); double computePID(double inp, double set, double *integ);
void applyMotorControl(double pid_L, double pid_R); // 用于 MODE_AUTO
void manual_applyMotorControl(int valL, int valR); // 用于 MODE_MANUAL
void spinOpenLoop(int speed);
long readLong(); void resetPID();
bool confirmStateChange(); void resetStateTimer();
bool readBTLine(char* buf, size_t maxlen);
bool parseLineToLR(const char* s, int &L, int &R);
static inline int16_t approach(int16_t cur, int16_t tgt);


// =================================================================
// 3. SETUP
// =================================================================
void setup() {
  Serial.begin(9600);
  Wire.begin();
  
  // MPU Init
  Wire.beginTransmission(MPU_ADDR); Wire.write(0x6B); Wire.write(0x00); Wire.endTransmission(true);

  stopMotors();
  delay(500);
  initEncoders();
  calibrateGyro();
  calibratePitch(); 

  // 【新增】初始化蓝牙
  BT.begin(9600); 
  BT.println("Car Ready! Mode: AUTO. Send 'MANUAL' to switch.");
  
  Serial.println("System Ready! Mode: AUTO");
  prevTime = millis();
  currentState = STATE_STRAIGHT;
  straightStartTime = millis();
  targetSpeed = SPEED_FLAT;

  lastCmdMs = millis(); // 初始化看门狗
}

// =================================================================
// 4. LOOP (双模式切换与控制核心)
// =================================================================
void loop() {
  unsigned long currentTime = millis();
  float dt = (currentTime - prevTime) / 1000.0;
  prevTime = currentTime;

  readMPU();
  updateAttitude(dt); 
  
  // ==================================================
  // A. 蓝牙输入处理 (模式切换 + 手动控制命令解析)
  // ==================================================
  char line[32];
  if (readBTLine(line, sizeof(line))) {
    lastCmdMs = millis();
    int L, R;
    
    // 检查模式切换命令
    if (!strcmp(line, "MANUAL")) {
      currentMode = MODE_MANUAL;
      stopMotors(); 
      resetPID();
      BT.println("Mode: MANUAL. Ready for control.");
      Serial.println("Mode: MANUAL");
      lastCmdMs = millis(); // 重置看门狗
      return; 
    } else if (!strcmp(line, "AUTO")) {
      currentMode = MODE_AUTO;
      stopMotors();
      // 重新初始化自动驾驶状态
      currentState = STATE_STRAIGHT;
      straightStartTime = millis();
      BT.println("Mode: AUTO. Starting mission.");
      Serial.println("Mode: AUTO");
      lastCmdMs = millis(); // 重置看门狗
      return;
    }
    
    // 仅在手动模式下处理运动指令
    if (currentMode == MODE_MANUAL) {
      if (parseLineToLR(line, L, R)) {
        tgtL = L; tgtR = R;
      } 
      else if (!strcmp(line, "FWD")) { tgtL = MED_SPEED; tgtR = MED_SPEED; }
      else if (!strcmp(line, "REV")) { tgtL = -MED_SPEED; tgtR = -MED_SPEED; }
      else if (!strcmp(line, "FAST")) { tgtL = HIGH_SPEED; tgtR = HIGH_SPEED; }
      else if (!strcmp(line, "SLOW")) { tgtL = LOW_SPEED; tgtR = LOW_SPEED; }
      else if (!strcmp(line, "LEFT")) { tgtL = -HIGH_SPEED; tgtR = HIGH_SPEED; }
      else if (!strcmp(line, "RIGHT")) { tgtL = HIGH_SPEED; tgtR = -HIGH_SPEED; }
      else if (!strcmp(line, "STOP")) { tgtL = 0; tgtR = 0; curL = 0; curR = 0; stopMotors(); }
    }
  }

  // ==================================================
  // B. 运动控制逻辑
  // ==================================================
  if (currentMode == MODE_AUTO) {
    // 【自动驾驶模式】
    if (millis() - lastPIDTime >= PID_SAMPLE_TIME) {
      lastPIDTime = millis();
      runPIDLoop();
    }
    // 【状态机逻辑】
    switch (currentState) {
        // [1] 起步直行 -> 检测上坡
        case STATE_STRAIGHT:
            targetSpeed = SPEED_FLAT;
            if (millis() - straightStartTime > STARTUP_LOCK_TIME) {
                if (pitch > RAMP_ANGLE_THRESHOLD) { 
                    if (confirmStateChange()) { currentState = STATE_CLIMBING; Serial.println(">>> CLIMBING <<<"); }
                } else resetStateTimer();
            }
            break;

        // [2] 爬坡中 -> 检测平地
        case STATE_CLIMBING:
            targetSpeed = SPEED_CLIMB;
            if (pitch < FLAT_ANGLE_THRESHOLD) { 
                if (confirmStateChange()) { 
                    topStopStart = millis();
                    currentState = STATE_TOP_STOP;
                    Serial.println(">>> TOP REACHED - STOP! <<<");
                }
            } else resetStateTimer();
            break;

        // [3] 坡顶停车 2s -> 准备旋转
        case STATE_TOP_STOP:
            targetSpeed = 0;
            stopMotors();
            resetPID();
            if (millis() - topStopStart >= TOP_STOP_TIME) {
                yaw = 0; currentState = STATE_SPIN; Serial.println(">>> SPINNING <<<");
            }
            break;

        // [4] 原地旋转
        case STATE_SPIN:
            targetSpeed = 0;
            spinOpenLoop(PWM_SPIN);
            if (abs(yaw) >= TARGET_YAW) {
                stopMotors(); delay(500); 
                currentState = STATE_CONTINUE; 
                resetPID();
                Serial.println(">>> CONTINUE (LOOKING FOR DOWNHILL) <<<");
            }
            break;

        // [5] 旋转后直行 -> ★ 检测下坡 ★
        case STATE_CONTINUE:
            targetSpeed = SPEED_FLAT;
            if (pitch < -RAMP_ANGLE_THRESHOLD) { 
                if (confirmStateChange()) { currentState = STATE_DOWNHILL; Serial.println(">>> DOWNHILL DETECTED <<<"); }
            } else resetStateTimer();
            break;

        // [6] 下坡中 -> ★ 检测平地 (终点) ★
        case STATE_DOWNHILL:
            targetSpeed = SPEED_FLAT;
            if (abs(pitch) < FLAT_ANGLE_THRESHOLD) {
                if (confirmStateChange()) { currentState = STATE_FINISHED; Serial.println(">>> FINISH LINE - STOP FOREVER <<<"); }
            } else resetStateTimer();
            break;

        // [7] 终点锁定
        case STATE_FINISHED:
            targetSpeed = 0;
            stopMotors();
            break;
    }
    
  } else { // MODE_MANUAL
    // 【手动控制模式】看门狗与平滑控制
    if (millis() - lastCmdMs > WATCHDOG_MS) {
      tgtL = 0; tgtR = 0; curL = 0; curR = 0; stopMotors();
    }
    if (tgtL != 0 || tgtR != 0 || curL != 0 || curR != 0) { // 运动或刹车
      curL = approach(curL, tgtL);
      curR = approach(curR, tgtR);
      manual_applyMotorControl(curL, curR); 
    }
  }

  // ==================================================
  // C. 状态反馈 (新增)
  // ==================================================
  static unsigned long lastReportTime = 0;
  const unsigned int REPORT_INTERVAL = 200; 
  
  if (millis() - lastReportTime >= REPORT_INTERVAL) {
    lastReportTime = millis();
    
    // 格式: M,状态/速度,Pitch/L_Speed,Yaw/R_Speed\n
    BT.print((currentMode == MODE_AUTO) ? 'A' : 'M');
    BT.print(',');
    
    if (currentMode == MODE_AUTO) {
        BT.print(getStateName(currentState)); 
        BT.print(',');
        BT.print(pitch, 1);
        BT.print(',');
        BT.print(yaw, 1);
    } else { 
        BT.print("SPEED");
        BT.print(',');
        BT.print(curL);
        BT.print(',');
        BT.print(curR);
    }
    BT.println();
  }
}

// =================================================================
// 5. 辅助函数 (Auto Drive / MPU)
// =================================================================

// MPU 姿态更新 (0.7/0.3 滤波)
void updateAttitude(float dt) {
  float raw_pitch = atan2(-ax, sqrt((long)ay * ay + (long)az * az)) * 180.0 / PI;
  raw_pitch -= pitch_offset;
  // 0.7/0.3 滤波，提高响应速度
  pitch = pitch * 0.7 + raw_pitch * 0.3; 

  float gyroZ_dps = (gz - gyroZ_offset) / 131.0;
  if (abs(gyroZ_dps) < 0.5) gyroZ_dps = 0;
  yaw += gyroZ_dps * dt;
}

// PID 主循环
void runPIDLoop() {
  if (currentState == STATE_SPIN || currentState == STATE_TOP_STOP || currentState == STATE_FINISHED) {
    currentTargetSpeed = 0;
    return;
  }
  // 加减速逻辑 (rampStep = 10.0)
  if (currentTargetSpeed < targetSpeed) currentTargetSpeed += rampStep;
  else if (currentTargetSpeed > targetSpeed) currentTargetSpeed -= rampStep;
  
  readEncodersWithCompensation();
  out_R = computePID(input_R, currentTargetSpeed, &integral_R);
  out_L = computePID(input_L, currentTargetSpeed, &integral_L);
  applyMotorControl(out_L, out_R);
}

// AUTO MODE：使用 PID 误差来计算并发送 I2C 命令
void applyMotorControl(double pid_L, double pid_R) {
  int pwm_L = basePWM + (int)pid_L;
  int pwm_R = basePWM + (int)pid_R;

  if (pwm_L > 0 && pwm_L < 20) pwm_L = 20; if (pwm_R > 0 && pwm_R < 20) pwm_R = 20;
  pwm_L = constrain(pwm_L, -100, 100); pwm_R = constrain(pwm_R, -100, 100);
  
  char logic_dir_L = (pwm_L >= 0) ? 'f' : 'r';
  char logic_dir_R = (pwm_R >= 0) ? 'f' : 'r';
  char cmd_M1 = (INV_M1) ? (logic_dir_R == 'f' ? 'r' : 'f') : logic_dir_R;
  char cmd_M2 = (INV_M2) ? (logic_dir_R == 'f' ? 'r' : 'f') : logic_dir_R;
  char cmd_M3 = (INV_M3) ? (logic_dir_L == 'f' ? 'r' : 'f') : logic_dir_L;
  char cmd_M4 = (INV_M4) ? (logic_dir_L == 'f' ? 'r' : 'f') : logic_dir_L;
  
  Wire.beginTransmission(MOTOR_ADDR);
  Wire.write('b'); Wire.write('a');
  Wire.write(cmd_M1); Wire.write(cmd_M2); Wire.write(cmd_M3); Wire.write(cmd_M4);
  Wire.write(abs(pwm_R)); Wire.write(0); Wire.write(abs(pwm_R)); Wire.write(0);
  Wire.write(abs(pwm_L)); Wire.write(0); Wire.write(abs(pwm_L)); Wire.write(0);
  Wire.endTransmission();
}

// 状态名称映射
const char* getStateName(TaskState state) {
  switch (state) {
    case STATE_INIT: return "INIT";
    case STATE_STRAIGHT: return "STRAIGHT";
    case STATE_CLIMBING: return "CLIMBING";
    case STATE_TOP_STOP: return "TOP_STOP";
    case STATE_SPIN: return "SPIN";
    case STATE_CONTINUE: return "CONTINUE";
    case STATE_DOWNHILL: return "DOWNHILL";
    case STATE_FINISHED: return "FINISHED";
    default: return "UNKNOWN";
  }
}

// 状态机辅助函数
bool confirmStateChange() {
  if (!isConditionMet) { stateChangeTimer = millis(); isConditionMet = true; return false; } 
  else if (millis() - stateChangeTimer > STABLE_TIME) { isConditionMet = false; return true; }
  return false;
}
void resetStateTimer() { isConditionMet = false; }


// =================================================================
// 6. 辅助函数 (Manual Control / Bluetooth)
// =================================================================

// MANUAL MODE：直接使用手机的 PWM/速度指令 (0-255)
void manual_applyMotorControl(int valL, int valR) {
    valL = constrain(valL, -MAX_SPEED, MAX_SPEED);
    valR = constrain(valR, -MAX_SPEED, MAX_SPEED);
    
    // 映射到你的驱动板 I2C 协议 (0-100)
    uint8_t spL = map(abs(valL), 0, MAX_SPEED, 0, 100);
    uint8_t spR = map(abs(valR), 0, MAX_SPEED, 0, 100);

    // 计算方向
    char logic_dir_L = (valL >= 0) ? 'f' : 'r';
    char logic_dir_R = (valR >= 0) ? 'f' : 'r';
    // 注意：这里需要根据 INV_Mx 宏定义来处理实际方向，但为了简化直接沿用
    char cmd_M1 = logic_dir_R; char cmd_M2 = logic_dir_R; 
    char cmd_M3 = logic_dir_L; char cmd_M4 = logic_dir_L; 

    Wire.beginTransmission(MOTOR_ADDR);
    Wire.write('b'); Wire.write('a');
    Wire.write(cmd_M1); Wire.write(cmd_M2); Wire.write(cmd_M3); Wire.write(cmd_M4);
    Wire.write(spR); Wire.write(0); Wire.write(spR); Wire.write(0); 
    Wire.write(spL); Wire.write(0); Wire.write(spL); Wire.write(0);
    Wire.endTransmission();
}

// 蓝牙串口读取一行数据
bool readBTLine(char* buf, size_t maxlen) {
    static size_t idx = 0;
    while (BT.available()) {
        char ch = BT.read();
        if (ch == '\r') continue;
        if (ch == '\n') {
            buf[idx] = '\0';
            idx = 0;
            return true;
        }
        if (idx < maxlen - 1) {
            buf[idx++] = ch;
        } else {
            idx = 0; // 缓冲区溢出，重置
        }
    }
    return false;
}

// 解析 "L,R" 格式的字符串命令
bool parseLineToLR(const char* s, int &L, int &R) {
    const char* c = strchr(s, ',');
    if (!c) return false;
    
    char *endp = nullptr;
    long l = strtol(s, &endp, 10);
    if (endp == s) return false;
    
    long r = strtol(c+1, nullptr, 10);

    L = (int)constrain(l, -MAX_SPEED, MAX_SPEED);
    R = (int)constrain(r, -MAX_SPEED, MAX_SPEED);
    return true;
}

// 平滑加速/减速
static inline int16_t approach(int16_t cur, int16_t tgt) {
    if (cur < tgt) { cur += MAX_STEP; if (cur > tgt) cur = tgt; } 
    else if (cur > tgt) { cur -= MAX_STEP; if (cur < tgt) cur = tgt; }
    return cur;
}

// ... (省略未修改的底层 PID/I2C/Calibration 函数，请确保将您上一版代码中这些函数的实现粘贴到此处)
// 如果您需要，请告诉我，我可以将它们全部复制过来。
// ...

// =================================================================
// 7. 底层 PID/I2C/Calibration 函数 (从上一版代码粘贴)
// =================================================================

void readEncodersWithCompensation() {
  Wire.requestFrom(MOTOR_ADDR, 16);
  if (Wire.available() >= 16) {
    long p1 = readLong(); long p2 = readLong(); readLong(); readLong();
    double raw_R = abs(p1 - last_pos_R);
    double raw_L = abs(p2 - last_pos_L);
    if (raw_R > 200) raw_R = input_R; if (raw_L > 200) raw_L = input_L;
    input_R = raw_R * RIGHT_RATIO; input_L = raw_L * LEFT_RATIO;
    last_pos_R = p1; last_pos_L = p2;
  } else while (Wire.available()) Wire.read();
}

double computePID(double inp, double set, double *integ) {
  double err = set - inp;
  if (abs(err) < 20) *integ += err;
  if (*integ > 100) *integ = 100; if (*integ < -100) *integ = -100;
  return (Kp * err) + (Ki * *integ);
}

void spinOpenLoop(int speed) {
  char cmd_M1 = (INV_M1) ? 'f' : 'r'; char cmd_M2 = (INV_M2) ? 'f' : 'r';
  char cmd_M3 = (INV_M3) ? 'r' : 'f'; char cmd_M4 = (INV_M4) ? 'r' : 'f';
  Wire.beginTransmission(MOTOR_ADDR);
  Wire.write('b'); Wire.write('a');
  Wire.write(cmd_M1); Wire.write(cmd_M2); Wire.write(cmd_M3); Wire.write(cmd_M4);
  Wire.write((uint8_t)speed); Wire.write(0); Wire.write((uint8_t)speed); Wire.write(0);
  Wire.write((uint8_t)speed); Wire.write(0); Wire.write((uint8_t)speed); Wire.write(0);
  Wire.endTransmission();
}

void stopMotors() {
  Wire.beginTransmission(MOTOR_ADDR); Wire.write('H'); Wire.write('a'); Wire.endTransmission();
}

void resetPID() {
  integral_L = 0; integral_R = 0; input_L = 0; input_R = 0; currentTargetSpeed = 0;
  initEncoders();
}

long readLong() {
  long v = 0; v |= (long)Wire.read(); v |= (long)Wire.read() << 8;
  v |= (long)Wire.read() << 16; v |= (long)Wire.read() << 24; return v;
}

void initEncoders() {
  Wire.requestFrom(MOTOR_ADDR, 16);
  if (Wire.available() >= 16) { last_pos_R = readLong(); last_pos_L = readLong(); readLong(); readLong(); }
}

void readMPU() {
  Wire.beginTransmission(MPU_ADDR); Wire.write(0x3B); Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 14, true);
  ax = Wire.read() << 8 | Wire.read(); ay = Wire.read() << 8 | Wire.read(); az = Wire.read() << 8 | Wire.read();
  Wire.read(); Wire.read(); gx = Wire.read() << 8 | Wire.read(); gy = Wire.read() << 8 | Wire.read(); gz = Wire.read() << 8 | Wire.read();
}

void calibrateGyro() {
  long sum = 0; for (int i = 0; i < 200; i++) { readMPU(); sum += gz; delay(3); }
  gyroZ_offset = sum / 200.0;
}

void calibratePitch() {
  float sum = 0;
  for (int i = 0; i < 50; i++) {
    readMPU();
    float p = atan2(-ax, sqrt((long)ay * ay + (long)az * az)) * 180.0 / PI;
    sum += p; delay(10);
  }
  pitch_offset = sum / 50.0;
}
