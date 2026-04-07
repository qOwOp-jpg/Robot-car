#include <EEPROM.h>

// =====================================================
// Analog PID Line Follower + onboard EEPROM log
// DASHBOARD-ONLY version
//
// Motor mapping:
// IN1 = 10  // left back
// IN2 = 9   // right back
// IN3 = 6   // right front
// IN4 = 5   // left front
//
// Sensors:
// A1 = right
// A2 = middle
// A3 = left
//
// Start Button:
// A0 -> GND, INPUT_PULLUP
//
// Flow:
// IDLE
//   -> CALIBRATE
//   -> FIND_LINE
//   -> ALIGN_LINE
//   -> FOLLOW
//      ->Detacted if Obstacle in the way
//
// Notes:
// - no delay()
// - PID remains the core follow algorithm
// - sensors are analog and normalized using min/max from calibration
//
// Commands in IDLE:
//   d / D : dump stored data from EEPROM
//   r / R : reset stored data
// =====================================================

// =====================================================
// Pins
// =====================================================
const int IN1 = 10;   // left back
const int IN2 = 9;    // right back
const int IN3 = 6;    // right front
const int IN4 = 5;    // left front

const int echoPin = 8;
const int trigPin = 7;

const int sensorR = A1;
const int sensorM = A2;
const int sensorL = A3;

const int buttonPin = A0;

// =====================================================
// FSM states
// =====================================================
enum RobotState {
  STATE_IDLE,
  STATE_CALIBRATE,
  STATE_FIND_LINE,
  STATE_ALIGN_LINE,
  STATE_FOLLOW,
  STATE_OBSTACLE,
  STATE_LOST
};

RobotState currentState = STATE_IDLE;
unsigned long stateStartTime = 0;

enum LinePattern {
  PATTERN_NORMAL_LINE,
  PATTERN_INVERTED_LINE,
  PATTERN_CROSSROAD,
  PATTERN_NO_LINE
};

enum LostPhase {
  LOST_TURN_RIGHT_LONG,
  LOST_FORWARD_1,
  LOST_TURN_LEFT_1,
  LOST_FORWARD_2,
  LOST_TURN_LEFT_2
};

LostPhase lostPhase = LOST_TURN_RIGHT_LONG;
unsigned long lostStateStartTime = 0;
unsigned long lostSequenceStartTime = 0;
const unsigned long lostTurnRightLongMs = 3333;
const unsigned long lostForward1Ms      = 420;
const unsigned long lostTurnLeft1Ms     = 520;
const unsigned long lostForward2Ms      = 420;
const unsigned long lostTurnLeft2Ms     = 520;
const unsigned long lostTotalTimeoutMs  = 37333;

enum ObstaclePhase {
  OBS_CHECK_STOP,
  OBS_BACKUP,
  OBS_ARC_TURN
};

ObstaclePhase obstaclePhase = OBS_CHECK_STOP;
unsigned long obstaclePhaseStartTime = 0;

// =====================================================
// Speed settings
// =====================================================
const int MIN_PWM = 100;
const int MAX_PWM = 255;

int baseSpeed        = 135;
int calibrateSpeed   = 180;
int findLineSpeed    = 200;
int alignSpeed       = 170;
int lostTurnSpeed    = 185;
int lostForwardSpeed = 140;
int obstacleBackSpeed = 155;

// =====================================================
// Timing settings
// =====================================================
const unsigned long calibrateSpinMs   = 2700;
const unsigned long findLineTimeoutMs = 2500;

// =====================================================
// Obstacle settings
// =====================================================
const float obstacleTriggerCm   = 12.0;
const float obstacleTargetCm    = 13.0;
const float obstacleToleranceCm = 1.7;
const unsigned long obstacleBackupMs = 333;

bool obstacleLineCandidate = false;
unsigned long obstacleLineCandidateStart = 0;
bool obstacleIgnoreRecheck = false;
unsigned long obstacleIgnoreUntil = 0;

// =====================================================
// PID settings
// =====================================================
float Kp = 68.0;
float Ki = 0.03;
float Kd = 4.7;

float error = 0.0;
float lastError = 0.0;
float integral = 0.0;
float integralLimit = 3.0;
unsigned long lastPIDTime = 0;
bool pidJustEntered = false;

float pTerm = 0.0f;
float iTerm = 0.0f;
float dTerm = 0.0f;

// =====================================================
// Button debounce
// =====================================================
bool buttonState = HIGH;
bool lastReading = HIGH;
unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 80;

// =====================================================
// Sensor calibration / normalized values
// =====================================================
int rawL = 0, rawM = 0, rawR = 0;
int normL = 0, normM = 0, normR = 0;

int minL = 1023, minM = 1023, minR = 1023;
int maxL = 0,    maxM = 0,    maxR = 0;

const bool LINE_IS_HIGH = true;

// =====================================================
// Direction memory
// =====================================================
int lastDirection = 0;

// =====================================================
// Onboard EEPROM log format (dashboard only)
// Total EEPROM on UNO: 1024 bytes
// =====================================================
const uint16_t LOG_MAGIC = 0x4C44; // "LD"
const uint8_t  LOG_VERSION = 2;
const uint8_t  LOG_STATUS_EMPTY     = 0;
const uint8_t  LOG_STATUS_RECORDING = 1;
const uint8_t  LOG_STATUS_COMPLETE  = 2;

const uint8_t FLAG_DASH_FULL = 0x01;

const uint16_t HEADER_ADDR = 0;
const uint16_t DASH_BASE_ADDR = 16;

// The smaller the dash interval is, the more precise the data are.
const uint16_t DASH_INTERVAL_MS = 150;

struct __attribute__((packed)) LogHeader {
  uint16_t magic;
  uint8_t version;
  uint8_t status;
  uint8_t flags;
  uint16_t dashIntervalMs;
  uint8_t dashCount;
  uint8_t dashRecSize;
  uint16_t dashBaseAddr;
  uint16_t reserved0;
  uint16_t reserved1;
  uint16_t reserved2;
  uint8_t reserved3;
};

// 7 bytes/record:
// - preserve original dashboard logic
// - add state for better post-analysis
// - time is reconstructed from index * dashIntervalMs to save EEPROM space
struct __attribute__((packed)) DashRec {
  int8_t err_x50;
  int8_t left_x2;
  int8_t right_x2;
  int8_t p_x2;
  int8_t i_x2;
  int8_t d_x4;
  uint8_t state;
};

const uint8_t DASH_REC_SIZE = sizeof(DashRec); // 7
const uint8_t DASH_MAX_RECORDS = (1024 - DASH_BASE_ADDR) / DASH_REC_SIZE; // 144

LogHeader logHeader;
bool logActive = false;
bool logFinalized = false;
unsigned long lastDashSampleMs = 0;

// =====================================================
// Forward declarations
// =====================================================
void stopMotors();
void startLostSequence();
void setState(RobotState newState);
float readDistanceCm();
bool obstacleDetected();
bool lineSeenForLock();
bool lineCenteredForFollow();
bool lineSeenForObstacleRecover();
bool calibrationValid();
void applyCalibrationFallback();
void finalizeStoredLog();

// =====================================================
// Utility
// =====================================================
int drivePWM(int v) {
  if (v <= 0) return 0;
  if (v < MIN_PWM) return MIN_PWM;
  if (v > MAX_PWM) return MAX_PWM;
  return v;
}

int clampPWM(int v) {
  if (v > MAX_PWM) return MAX_PWM;
  if (v < -MAX_PWM) return -MAX_PWM;
  return v;
}

int8_t clampI8(int v) {
  if (v > 127) return 127;
  if (v < -128) return -128;
  return (int8_t)v;
}

float absf_local(float v) {
  return (v >= 0.0f) ? v : -v;
}

const char* stateName(uint8_t s) {
  switch (s) {
    case STATE_IDLE:      return "IDLE";
    case STATE_CALIBRATE: return "CALIBRATE";
    case STATE_FIND_LINE: return "FIND_LINE";
    case STATE_ALIGN_LINE:return "ALIGN_LINE";
    case STATE_FOLLOW:    return "FOLLOW";
    case STATE_OBSTACLE:  return "OBSTACLE";
    case STATE_LOST:      return "LOST";
    default:              return "UNKNOWN";
  }
}

void clearPidTermsForNonFollow() {
  error = 0.0f;
  pTerm = 0.0f;
  iTerm = 0.0f;
  dTerm = 0.0f;
}

// =====================================================
// EEPROM helpers
// =====================================================
template <typename T>
void eepromWriteStruct(int addr, const T &data) {
  const uint8_t *p = (const uint8_t*)(&data);
  for (unsigned int i = 0; i < sizeof(T); i++) {
    EEPROM.update(addr + (int)i, p[i]);
  }
}

template <typename T>
void eepromReadStruct(int addr, T &data) {
  uint8_t *p = (uint8_t*)(&data);
  for (unsigned int i = 0; i < sizeof(T); i++) {
    p[i] = EEPROM.read(addr + (int)i);
  }
}

void initHeaderDefaults() {
  logHeader.magic = LOG_MAGIC;
  logHeader.version = LOG_VERSION;
  logHeader.status = LOG_STATUS_EMPTY;
  logHeader.flags = 0;
  logHeader.dashIntervalMs = DASH_INTERVAL_MS;
  logHeader.dashCount = 0;
  logHeader.dashRecSize = DASH_REC_SIZE;
  logHeader.dashBaseAddr = DASH_BASE_ADDR;
  logHeader.reserved0 = 0;
  logHeader.reserved1 = 0;
  logHeader.reserved2 = 0;
  logHeader.reserved3 = 0;
}

void writeHeaderToEeprom() {
  eepromWriteStruct(HEADER_ADDR, logHeader);
}

bool readHeaderFromEeprom() {
  eepromReadStruct(HEADER_ADDR, logHeader);
  return (logHeader.magic == LOG_MAGIC &&
          logHeader.version == LOG_VERSION &&
          logHeader.dashBaseAddr == DASH_BASE_ADDR &&
          logHeader.dashRecSize == DASH_REC_SIZE);
}

void clearStoredLogHeader() {
  initHeaderDefaults();
  logHeader.status = LOG_STATUS_EMPTY;
  logHeader.flags = 0;
  logHeader.dashCount = 0;
  writeHeaderToEeprom();

  logActive = false;
  logFinalized = false;
  lastDashSampleMs = 0;
}

// =====================================================
// Telemetry / log (dashboard only)
// =====================================================
void beginNewStoredLog() {
  initHeaderDefaults();
  logHeader.status = LOG_STATUS_RECORDING;
  logHeader.flags = 0;
  logHeader.dashCount = 0;
  writeHeaderToEeprom();

  logActive = true;
  logFinalized = false;
  lastDashSampleMs = 0;
}

void logDashboardSample(int leftCmd, int rightCmd) {
  if (!logActive) return;

  unsigned long now = millis();
  if (now - lastDashSampleMs < DASH_INTERVAL_MS) return;
  lastDashSampleMs = now;

  if (logHeader.dashCount >= DASH_MAX_RECORDS) {
    logHeader.flags |= FLAG_DASH_FULL;
    return;
  }

  DashRec r;
  r.err_x50  = clampI8((int)(error * 50.0f));
  r.left_x2  = clampI8(leftCmd / 2);
  r.right_x2 = clampI8(rightCmd / 2);
  r.p_x2     = clampI8((int)(pTerm / 2.0f));
  r.i_x2     = clampI8((int)(iTerm / 2.0f));
  r.d_x4     = clampI8((int)(dTerm / 4.0f));
  r.state    = (uint8_t)currentState;

  int addr = DASH_BASE_ADDR + (int)logHeader.dashCount * DASH_REC_SIZE;
  eepromWriteStruct(addr, r);
  logHeader.dashCount++;
}

void telemetryCollect(int leftCmd, int rightCmd, bool obstacleFlag) {
  (void)obstacleFlag;
  logDashboardSample(leftCmd, rightCmd);
}

void finalizeStoredLog() {
  if (!logActive || logFinalized) return;
  logHeader.status = LOG_STATUS_COMPLETE;
  writeHeaderToEeprom();
  logActive = false;
  logFinalized = true;
}

void dumpStoredLog() {
  if (!readHeaderFromEeprom()) {
    Serial.println(F("NO_DATA"));
    return;
  }

  if (logHeader.status == LOG_STATUS_EMPTY || logHeader.dashCount == 0) {
    Serial.println(F("NO_DATA"));
    return;
  }

  Serial.println(F("BEGIN_META"));
  Serial.print(F("dash_interval_ms,")); Serial.println(logHeader.dashIntervalMs);
  Serial.print(F("dash_count,")); Serial.println(logHeader.dashCount);
  Serial.print(F("flags,")); Serial.println(logHeader.flags);
  Serial.println(F("END_META"));

  Serial.println(F("BEGIN_DASHBOARD"));
  Serial.println(F("idx,time_s,state,state_name,error,left_pwm,right_pwm,P,I,D"));
  for (uint8_t i = 0; i < logHeader.dashCount; i++) {
    DashRec r;
    int addr = DASH_BASE_ADDR + (int)i * DASH_REC_SIZE;
    eepromReadStruct(addr, r);

    Serial.print(i);
    Serial.print(',');

    float timeS = (float)((unsigned long)i * (unsigned long)logHeader.dashIntervalMs) / 1000.0f;
    Serial.print(timeS, 3);
    Serial.print(',');

    Serial.print(r.state);
    Serial.print(',');
    Serial.print(stateName(r.state));
    Serial.print(',');

    Serial.print((float)r.err_x50 / 50.0f, 3);
    Serial.print(',');

    Serial.print((int)r.left_x2 * 2);
    Serial.print(',');

    Serial.print((int)r.right_x2 * 2);
    Serial.print(',');

    Serial.print((float)r.p_x2 * 2.0f, 3);
    Serial.print(',');

    Serial.print((float)r.i_x2 * 2.0f, 3);
    Serial.print(',');

    Serial.println((float)r.d_x4 * 4.0f, 3);
  }
  Serial.println(F("END_DASHBOARD"));
}

void handleSerialCommand() {
  while (Serial.available() > 0) {
    char c = Serial.read();

    if (currentState != STATE_IDLE) {
      continue;
    }

    if (c == 'd' || c == 'D') {
      dumpStoredLog();
    } else if (c == 'r' || c == 'R') {
      clearStoredLogHeader();
      Serial.println(F("RESET_DONE"));
    }
  }
}

// =====================================================
// Sensor reading
// =====================================================
int readRawL() { return analogRead(sensorL); }
int readRawM() { return analogRead(sensorM); }
int readRawR() { return analogRead(sensorR); }

int normalizeSensor(int raw, int minV, int maxV) {
  if (maxV <= minV + 5) return 0;

  long val = map(raw, minV, maxV, 0, 1000);
  if (val < 0) val = 0;
  if (val > 1000) val = 1000;

  if (!LINE_IS_HIGH) {
    val = 1000 - val;
  }

  return (int)val;
}

void readSensorsNormalized() {
  rawL = readRawL();
  rawM = readRawM();
  rawR = readRawR();

  normL = normalizeSensor(rawL, minL, maxL);
  normM = normalizeSensor(rawM, minM, maxM);
  normR = normalizeSensor(rawR, minR, maxR);
}

LinePattern detectLinePattern() {
  readSensorsNormalized();

  bool leftBlack   = normL > 180;
  bool middleBlack = normM > 180;
  bool rightBlack  = normR > 180;

  long total = (long)normL + (long)normM + (long)normR;

  if (!leftBlack && !middleBlack && !rightBlack) {
    return PATTERN_NO_LINE;
  }

  if (normL > 260 && normR > 260 && normM < 120) {
    return PATTERN_INVERTED_LINE;
  }

  if ((leftBlack && middleBlack && rightBlack) || total > 2200) {
    return PATTERN_CROSSROAD;
  }

  return PATTERN_NORMAL_LINE;
}

// =====================================================
// PID helpers
// =====================================================
void resetPID() {
  error = 0.0f;
  lastError = 0.0f;
  integral = 0.0f;
  pTerm = 0.0f;
  iTerm = 0.0f;
  dTerm = 0.0f;
  lastPIDTime = millis();
}

void setState(RobotState newState) {
  if (currentState == newState) return;

  RobotState oldState = currentState;
  currentState = newState;
  stateStartTime = millis();

  if (newState == STATE_CALIBRATE) {
    minL = minM = minR = 1023;
    maxL = maxM = maxR = 0;

    lastDirection = 0;

    error = 0.0f;
    lastError = 0.0f;
    integral = 0.0f;
    pTerm = 0.0f;
    iTerm = 0.0f;
    dTerm = 0.0f;
    lastPIDTime = millis();
    pidJustEntered = false;

    lostPhase = LOST_TURN_RIGHT_LONG;
    lostStateStartTime = 0;
    lostSequenceStartTime = 0;

    logActive = false;
    logFinalized = false;
    lastDashSampleMs = 0;
  }

  if (newState == STATE_FIND_LINE && oldState == STATE_CALIBRATE) {
    beginNewStoredLog();
  }

  if (newState == STATE_FOLLOW) {
    resetPID();
    pidJustEntered = true;
  }

  if (newState == STATE_LOST) {
    startLostSequence();
  }

  if (newState == STATE_OBSTACLE) {
    obstaclePhase = OBS_CHECK_STOP;
    obstaclePhaseStartTime = millis();
    obstacleLineCandidate = false;
    obstacleLineCandidateStart = 0;
    stopMotors();
  }

  if (newState == STATE_IDLE) {
    finalizeStoredLog();
  }
}

// =====================================================
// Motor control
// =====================================================
void setLeftForward(int speedVal) {
  digitalWrite(IN1, LOW);
  analogWrite(IN4, drivePWM(speedVal));
}

void setLeftBackward(int speedVal) {
  digitalWrite(IN4, LOW);
  analogWrite(IN1, drivePWM(speedVal));
}

void setRightForward(int speedVal) {
  digitalWrite(IN2, LOW);
  analogWrite(IN3, drivePWM(speedVal));
}

void setRightBackward(int speedVal) {
  digitalWrite(IN3, LOW);
  analogWrite(IN2, drivePWM(speedVal));
}

void stopLeftMotor() {
  analogWrite(IN1, 0);
  analogWrite(IN4, 0);
  digitalWrite(IN1, LOW);
  digitalWrite(IN4, LOW);
}

void stopRightMotor() {
  analogWrite(IN2, 0);
  analogWrite(IN3, 0);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
}

void stopMotors() {
  stopLeftMotor();
  stopRightMotor();
}

void setLeftSigned(int speedVal) {
  if (speedVal > 0) {
    setLeftForward(speedVal);
  } else if (speedVal < 0) {
    setLeftBackward(-speedVal);
  } else {
    stopLeftMotor();
  }
}

void setRightSigned(int speedVal) {
  if (speedVal > 0) {
    setRightForward(speedVal);
  } else if (speedVal < 0) {
    setRightBackward(-speedVal);
  } else {
    stopRightMotor();
  }
}

void setBothSigned(int leftSpeed, int rightSpeed) {
  setLeftSigned(leftSpeed);
  setRightSigned(rightSpeed);
}

void goForwardRaw(int speedVal) {
  setLeftForward(speedVal);
  setRightForward(speedVal);
}

void goBackwardRaw(int speedVal) {
  setLeftBackward(speedVal);
  setRightBackward(speedVal);
}

void rotateLeftRaw(int speedVal) {
  setLeftBackward(speedVal);
  setRightForward(speedVal);
}

void rotateRightRaw(int speedVal) {
  setLeftForward(speedVal);
  setRightBackward(speedVal);
}

// =====================================================
// Line logic
// =====================================================
bool computeLineError(float &outError) {
  readSensorsNormalized();

  long weightedSum =
      (long)normL * -1000L +
      (long)normM * 0L +
      (long)normR * 1000L;

  long total = (long)normL + (long)normM + (long)normR;

  bool leftSeen   = normL > 180;
  bool middleSeen = normM > 180;
  bool rightSeen  = normR > 180;

  if (!(leftSeen || middleSeen || rightSeen) || total <= 0) {
    if (lastDirection < 0) outError = -1.8f;
    else if (lastDirection > 0) outError = 1.8f;
    else outError = 0.0f;
    return false;
  }

  outError = (float)weightedSum / (float)total / 1000.0f;

  if (outError < -0.15f) lastDirection = -1;
  else if (outError > 0.15f) lastDirection = 1;
  else lastDirection = 0;

  return true;
}

bool computeInvertedLineError(float &outError) {
  readSensorsNormalized();

  int invL = 1000 - normL;
  int invM = 1000 - normM;
  int invR = 1000 - normR;

  if (invL < 0) invL = 0;
  if (invM < 0) invM = 0;
  if (invR < 0) invR = 0;

  long weightedSum =
      (long)invL * -1000L +
      (long)invM * 0L +
      (long)invR * 1000L;

  long total = (long)invL + (long)invM + (long)invR;

  bool leftSeen   = invL > 180;
  bool middleSeen = invM > 180;
  bool rightSeen  = invR > 180;

  if (!(leftSeen || middleSeen || rightSeen) || total <= 0) {
    if (lastDirection < 0) outError = -1.8f;
    else if (lastDirection > 0) outError = 1.8f;
    else outError = 0.0f;
    return false;
  }

  outError = (float)weightedSum / (float)total / 1000.0f;

  if (outError < -0.15f) lastDirection = -1;
  else if (outError > 0.15f) lastDirection = 1;
  else lastDirection = 0;

  return true;
}

bool lineSeenForLock() {
  readSensorsNormalized();

  long total = (long)normL + (long)normM + (long)normR;
  if (total <= 0) return false;

  int maxVal = max(normL, max(normM, normR));
  int minVal = min(normL, min(normM, normR));

  float maxRatio = (float)maxVal / (float)total;
  float spreadRatio = 0.0f;
  if (maxVal > 0) {
    spreadRatio = (float)(maxVal - minVal) / (float)maxVal;
  }

  return (maxRatio > 0.38f) && (spreadRatio > 0.12f);
}

bool lineSeenForObstacleRecover() {
  readSensorsNormalized();

  long total = (long)normL + (long)normM + (long)normR;
  if (total < 420) return false;

  bool anyTwoSeen =
      ((normL > 170) && (normM > 170)) ||
      ((normM > 170) && (normR > 170)) ||
      ((normL > 170) && (normR > 170));

  bool oneStrong =
      (normL > 260) || (normM > 240) || (normR > 260);

  return anyTwoSeen || oneStrong;
}

bool lineCenteredForFollow() {
  readSensorsNormalized();

  long total = (long)normL + (long)normM + (long)normR;
  if (total <= 0) return false;

  int maxSide = max(normL, normR);
  int minSide = min(normL, normR);

  float midRatio = (float)normM / (float)total;

  float sideImbalance = 0.0f;
  if (maxSide > 0) {
    sideImbalance = (float)(maxSide - minSide) / (float)maxSide;
  }

  bool middleDominant = (normM >= normL && normM >= normR);
  bool middleStrongEnough = (midRatio > 0.38f);
  bool sidesBalancedEnough = (sideImbalance < 0.55f);
  bool bridgeCase = (normM >= (int)(0.75f * maxSide));

  return (middleDominant && middleStrongEnough && sidesBalancedEnough) ||
         (middleStrongEnough && bridgeCase && sidesBalancedEnough);
}

// =====================================================
// Ultrasonic
// =====================================================
float readDistanceCm() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);

  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);

  digitalWrite(trigPin, LOW);

  unsigned long duration = pulseIn(echoPin, HIGH, 30000);

  if (duration == 0) return 999.0f;

  return duration * 0.0343f / 2.0f;
}

bool obstacleDetected() {
  float d = readDistanceCm();
  return (d > 0.0f && d < obstacleTriggerCm);
}

// =====================================================
// Button handling
// =====================================================
void updateButton() {
  bool reading = digitalRead(buttonPin);

  if (reading != lastReading) {
    lastDebounceTime = millis();
  }

  if ((millis() - lastDebounceTime) > debounceDelay) {
    if (reading != buttonState) {
      buttonState = reading;

      if (buttonState == LOW) {
        if (currentState == STATE_IDLE) {
          setState(STATE_CALIBRATE);
        } else {
          stopMotors();
          lastDirection = 0;
          setState(STATE_IDLE);
        }
      }
    }
  }

  lastReading = reading;
}

// =====================================================
// State handlers
// =====================================================
void handleIdle() {
  stopMotors();
}

bool calibrationValid() {
  int spanL = maxL - minL;
  int spanM = maxM - minM;
  int spanR = maxR - minR;

  int goodCount = 0;
  if (spanL >= 80) goodCount++;
  if (spanM >= 80) goodCount++;
  if (spanR >= 80) goodCount++;

  return goodCount >= 2;
}

void applyCalibrationFallback() {
  if (maxL - minL < 50) {
    int c = readRawL();
    minL = max(0, c - 120);
    maxL = min(1023, c + 120);
  }

  if (maxM - minM < 50) {
    int c = readRawM();
    minM = max(0, c - 120);
    maxM = min(1023, c + 120);
  }

  if (maxR - minR < 50) {
    int c = readRawR();
    minR = max(0, c - 120);
    maxR = min(1023, c + 120);
  }
}

void handleCalibrate() {
  rawL = readRawL();
  rawM = readRawM();
  rawR = readRawR();

  if (rawL < minL) minL = rawL;
  if (rawL > maxL) maxL = rawL;

  if (rawM < minM) minM = rawM;
  if (rawM > maxM) maxM = rawM;

  if (rawR < minR) minR = rawR;
  if (rawR > maxR) maxR = rawR;

  rotateLeftRaw(calibrateSpeed);

  if (millis() - stateStartTime >= calibrateSpinMs) {
    if (!calibrationValid()) {
      applyCalibrationFallback();
    }
    setState(STATE_FIND_LINE);
  }
}

void handleFindLine() {
  readSensorsNormalized();
  clearPidTermsForNonFollow();

  if (lineSeenForLock()) {
    setState(STATE_ALIGN_LINE);
    return;
  }

  if (lastDirection < 0) {
    rotateLeftRaw(findLineSpeed);
    telemetryCollect(-findLineSpeed, findLineSpeed, false);
  } else if (lastDirection > 0) {
    rotateRightRaw(findLineSpeed);
    telemetryCollect(findLineSpeed, -findLineSpeed, false);
  } else {
    rotateLeftRaw(findLineSpeed);
    telemetryCollect(-findLineSpeed, findLineSpeed, false);
  }

  if (millis() - stateStartTime >= findLineTimeoutMs) {
    stopMotors();
    setState(STATE_IDLE);
  }
}

void handleAlignLine() {
  readSensorsNormalized();
  clearPidTermsForNonFollow();

  int total = normL + normM + normR;
  if (total < 150) {
    setState(STATE_FIND_LINE);
    return;
  }

  if (lineCenteredForFollow()) {
    setState(STATE_FOLLOW);
    return;
  }

  int lrDiff = normR - normL;

  if (lrDiff > 160) {
    setLeftForward(alignSpeed);
    setRightBackward(alignSpeed);
    telemetryCollect(alignSpeed, -alignSpeed, false);
    lastDirection = 1;
    return;
  }

  if (lrDiff < -160) {
    setLeftBackward(alignSpeed);
    setRightForward(alignSpeed);
    telemetryCollect(-alignSpeed, alignSpeed, false);
    lastDirection = -1;
    return;
  }

  if (normM >= normL && normM >= normR) {
    goForwardRaw(125);
    telemetryCollect(125, 125, false);
    setState(STATE_FOLLOW);
    return;
  }

  if (normR > normL) {
    setLeftForward(alignSpeed);
    setRightBackward(alignSpeed);
    telemetryCollect(alignSpeed, -alignSpeed, false);
    lastDirection = 1;
  } else {
    setLeftBackward(alignSpeed);
    setRightForward(alignSpeed);
    telemetryCollect(-alignSpeed, alignSpeed, false);
    lastDirection = -1;
  }
}

void handleFollow() {
  if (obstacleIgnoreRecheck && millis() >= obstacleIgnoreUntil) {
    obstacleIgnoreRecheck = false;
  }

  if (!obstacleIgnoreRecheck && obstacleDetected()) {
    setState(STATE_OBSTACLE);
    return;
  }

  float lineError = 0.0f;
  bool sawLine = false;

  LinePattern pattern = detectLinePattern();

  if (pattern == PATTERN_CROSSROAD) {
    error = 0.0f;
    pTerm = 0.0f;
    iTerm = 0.0f;
    dTerm = 0.0f;
    goForwardRaw(baseSpeed);
    telemetryCollect(baseSpeed, baseSpeed, false);
    return;
  } else if (pattern == PATTERN_INVERTED_LINE) {
    sawLine = computeInvertedLineError(lineError);
  } else if (pattern == PATTERN_NORMAL_LINE) {
    sawLine = computeLineError(lineError);
  } else {
    sawLine = false;
  }

  if (!sawLine) {
    clearPidTermsForNonFollow();
    telemetryCollect(0, 0, false);
    setState(STATE_LOST);
    return;
  }

  unsigned long now = millis();
  float dt = (float)(now - lastPIDTime) / 1000.0f;
  if (dt <= 0.0f) dt = 0.001f;

  error = lineError;

  integral += error * dt;
  if (integral > integralLimit) integral = integralLimit;
  if (integral < -integralLimit) integral = -integralLimit;

  float derivative = 0.0f;
  if (!pidJustEntered) {
    derivative = (error - lastError) / dt;
  } else {
    derivative = 0.0f;
    pidJustEntered = false;
  }

  pTerm = Kp * error;
  iTerm = Ki * integral;
  dTerm = Kd * derivative;

  float correction = pTerm + iTerm + dTerm;

  if (correction > 90.0f) correction = 90.0f;
  if (correction < -90.0f) correction = -90.0f;

  float absError = absf_local(error);
  if (absError < 0.12f) {
    correction = 0.0f;
  } else {
    const int minCorrection = 28;
    if (error > 0.12f && correction < (float)minCorrection) {
      correction = (float)minCorrection;
    } else if (error < -0.12f && correction > (float)(-minCorrection)) {
      correction = (float)(-minCorrection);
    }
  }

  int dynamicBase = baseSpeed;
  if (absError >= 1.0f) dynamicBase = 105;
  else if (absError >= 0.5f) dynamicBase = 118;
  else dynamicBase = baseSpeed;

  int leftCmd  = clampPWM((int)(dynamicBase - correction));
  int rightCmd = clampPWM((int)(dynamicBase + correction));

  setBothSigned(leftCmd, rightCmd);
  telemetryCollect(leftCmd, rightCmd, false);

  lastError = error;
  lastPIDTime = now;
}

void handleObstacle() {
  unsigned long now = millis();
  clearPidTermsForNonFollow();

  switch (obstaclePhase) {
    case OBS_CHECK_STOP: {
      stopMotors();
      telemetryCollect(0, 0, true);

      if (now - obstaclePhaseStartTime < 140) {
        return;
      }

      float d = readDistanceCm();

      if (d >= (obstacleTargetCm - obstacleToleranceCm) &&
          d <= (obstacleTargetCm + obstacleToleranceCm)) {
        obstaclePhase = OBS_ARC_TURN;
        obstaclePhaseStartTime = now;
      } else if (d < (obstacleTargetCm - obstacleToleranceCm)) {
        obstaclePhase = OBS_BACKUP;
        obstaclePhaseStartTime = now;
      } else {
        obstaclePhase = OBS_ARC_TURN;
        obstaclePhaseStartTime = now;
      }

      break;
    }

    case OBS_BACKUP: {
      unsigned long backupElapsed = now - obstaclePhaseStartTime;

      if (backupElapsed < 110) {
        goBackwardRaw(185);
        telemetryCollect(-185, -185, true);
      } else {
        goBackwardRaw(obstacleBackSpeed);
        telemetryCollect(-obstacleBackSpeed, -obstacleBackSpeed, true);
      }

      if (backupElapsed >= obstacleBackupMs) {
        stopMotors();
        obstaclePhase = OBS_ARC_TURN;
        obstaclePhaseStartTime = now;
      }

      break;
    }

    case OBS_ARC_TURN: {
      unsigned long turnElapsed = now - obstaclePhaseStartTime;

      if (turnElapsed < 500) {
        setLeftForward(185);
        setRightBackward(185);
        telemetryCollect(185, -185, true);
      } else if (turnElapsed < 1100) {
        setLeftForward(175);
        setRightForward(175);
        telemetryCollect(175, 175, true);
      } else if (turnElapsed < 1500) {
        setLeftBackward(185);
        setRightForward(185);
        telemetryCollect(-185, 185, true);
      } else if (turnElapsed < 1800) {
        setLeftForward(130);
        setRightForward(190);
        telemetryCollect(130, 190, true);
      } else if (turnElapsed < 2100) {
        setLeftBackward(185);
        setRightForward(185);
        telemetryCollect(-185, 185, true);
      } else if (turnElapsed < 7500) {
        setLeftForward(105);
        setRightForward(255);
        telemetryCollect(105, 255, true);
      } else {
        stopMotors();
        telemetryCollect(0, 0, true);
        setState(STATE_LOST);
        return;
      }

      if (turnElapsed > 1200) {
        if (lineSeenForObstacleRecover()) {
          if (!obstacleLineCandidate) {
            obstacleLineCandidate = true;
            obstacleLineCandidateStart = now;
          } else if (now - obstacleLineCandidateStart > 30) {
            obstacleIgnoreRecheck = true;
            obstacleIgnoreUntil = millis() + 900;
            obstacleLineCandidate = false;
            setState(STATE_ALIGN_LINE);
            return;
          }
        } else {
          obstacleLineCandidate = false;
        }
      }
      break;
    }
  }
}

void startLostSequence() {
  lostPhase = LOST_TURN_RIGHT_LONG;
  lostStateStartTime = millis();
  lostSequenceStartTime = millis();
}

void nextLostPhase(LostPhase nextPhase) {
  lostPhase = nextPhase;
  lostStateStartTime = millis();
}

void handleLost() {
  clearPidTermsForNonFollow();

  if (lineSeenForLock()) {
    setState(STATE_ALIGN_LINE);
    return;
  }

  if (millis() - lostSequenceStartTime >= lostTotalTimeoutMs) {
    stopMotors();
    telemetryCollect(0, 0, false);
    setState(STATE_IDLE);
    return;
  }

  unsigned long phaseElapsed = millis() - lostStateStartTime;

  switch (lostPhase) {
    case LOST_TURN_RIGHT_LONG:
      rotateRightRaw(lostTurnSpeed);
      telemetryCollect(lostTurnSpeed, -lostTurnSpeed, false);
      if (phaseElapsed >= lostTurnRightLongMs) {
        nextLostPhase(LOST_FORWARD_1);
      }
      break;

    case LOST_FORWARD_1:
      goForwardRaw(lostForwardSpeed);
      telemetryCollect(lostForwardSpeed, lostForwardSpeed, false);
      if (phaseElapsed >= lostForward1Ms) {
        nextLostPhase(LOST_TURN_LEFT_1);
      }
      break;

    case LOST_TURN_LEFT_1:
      rotateLeftRaw(lostTurnSpeed);
      telemetryCollect(-lostTurnSpeed, lostTurnSpeed, false);
      if (phaseElapsed >= lostTurnLeft1Ms) {
        nextLostPhase(LOST_FORWARD_2);
      }
      break;

    case LOST_FORWARD_2:
      goForwardRaw(lostForwardSpeed);
      telemetryCollect(lostForwardSpeed, lostForwardSpeed, false);
      if (phaseElapsed >= lostForward2Ms) {
        nextLostPhase(LOST_TURN_LEFT_2);
      }
      break;

    case LOST_TURN_LEFT_2:
      rotateLeftRaw(lostTurnSpeed);
      telemetryCollect(-lostTurnSpeed, lostTurnSpeed, false);
      if (phaseElapsed >= lostTurnLeft2Ms) {
        nextLostPhase(LOST_TURN_RIGHT_LONG);
      }
      break;
  }
}

// =====================================================
// Setup / loop
// =====================================================
void setup() {
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  digitalWrite(trigPin, LOW);

  pinMode(buttonPin, INPUT_PULLUP);

  Serial.begin(9600);
  stopMotors();

  if (!readHeaderFromEeprom()) {
    clearStoredLogHeader();
  }

  setState(STATE_IDLE);
}

void loop() {
  handleSerialCommand();
  updateButton();

  switch (currentState) {
    case STATE_IDLE:
      handleIdle();
      break;
    case STATE_CALIBRATE:
      handleCalibrate();
      break;
    case STATE_FIND_LINE:
      handleFindLine();
      break;
    case STATE_ALIGN_LINE:
      handleAlignLine();
      break;
    case STATE_FOLLOW:
      handleFollow();
      break;
    case STATE_OBSTACLE:
      handleObstacle();
      break;
    case STATE_LOST:
      handleLost();
      break;
  }
}
