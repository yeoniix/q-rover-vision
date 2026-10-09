#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_VL53L0X.h>
#include <Arduino_RouterBridge.h>

// ===== SLAVE1 (UNO Q) : 눈과 다리 · 회로 V5 =====
// UART (D0 RX / D1 TX = Serial1, 57600bps) <-> MASTER
//   Q -> MASTER : "READY"  부팅 완료 (첫 명령을 받을 때까지 1초마다)
//                 "P1"/"P0" 비상(인형 200mm 이내) / 정상   (100ms마다)
//                 "D<mm>"   현재 거리 (예: D185, D2000=멂, D-1=오류) (100ms마다)
//   MASTER -> Q : "G" 정상 주행 허락 / "L" 감속 (과전류 또는 물체 200mm 이내)
//
// 동작
//   - 인형이 보이고 거리 200mm 이하 -> 비상 상태, P1 전송 (주행은 계속)
//   - 비상 상태에서 거리 100mm 이하 -> 즉시 정지
//   - 비상 상태는 거리가 250mm보다 멀어지면 해제 (가까이서 카메라가 놓쳐도 유지)
//   - 정지 후: 비상이 해제되고 MASTER의 "G"를 받아야 재출발
//   - "L" 받으면 LIMIT_PWM으로 감속, "G" 받으면 정상 속도
//   - 부팅 직후에는 "G"를 받을 때까지 대기
//
// 참고: UNO Q에서 D0/D1 핀은 Serial1 입니다.
//       Serial은 보드 내부(Bridge/Monitor)용이라 D0/D1로 나가지 않습니다.
//       MCU가 부팅할 때 Serial1로 Zephyr 부팅 메시지가 한 번 나가는데,
//       마스터는 모르는 메시지로 보고 무시합니다.

// ---------- DRV8833 (수업 배선) ----------
const int AIN1 = 5;   // 왼쪽 모터
const int AIN2 = 3;
const int BIN1 = 6;   // 오른쪽 모터
const int BIN2 = 9;

// ---------- 주행 설정 ----------
const int NORMAL_PWM = 60;           // 정상 속도 (0~255)
const int LIMIT_PWM = 35;            // 감속(L) 시 속도 (정상의 약 절반)
const int ALERT_MM = 200;            // 인형이 이 거리 이하면 비상(P1)
const int STOP_MM = 100;             // 비상 상태에서 이 거리 이하면 정지
const int ALERT_CLEAR_MM = 250;      // 이 거리보다 멀어지면 비상 해제
const unsigned long KICK_MS = 120;   // 출발할 때만 잠깐 최대 출력 (수업 startDrive)

const int FAR_MM = 2000;
const unsigned long REPORT_MS = 100;
const unsigned long READY_MS = 1000;
const unsigned long PY_WATCHDOG_MS = 1000;

// 상태 코드 (로그 확인용)
const int ST_RUN = 0;       // 정상 주행
const int ST_LIMIT = 1;     // 감속 주행
const int ST_STOP = 2;      // 인형 100mm 이내 -> 정지
const int ST_WAIT_GO = 3;   // G 대기
const int ST_NO_APP = 4;    // 카메라(Python) 꺼짐
const int ST_ALERT = 5;     // 비상(P1) 상태로 주행 중

Adafruit_VL53L0X tof;

// 카메라(Python)
int cmdSeen = 0;
unsigned long lastPyMs = 0;
bool hasPy = false;

// 마스터
bool driveAllowed = false;   // G를 받아야 true, 정지 시 false
bool limitMode = false;      // L이면 true, G면 false
bool goRequested = false;
bool gotCommand = false;
char mbuf[16];
byte mlen = 0;

bool alert = false;          // 비상 상태 (P1)
int prevSpeed = 0;
unsigned long lastReportMs = 0;
unsigned long lastReadyMs = 0;

// ---------- 모터 (수업 코드) ----------
void wheel(int in1, int in2, int speed) {
  speed = constrain(speed, -255, 255);
  if (speed > 0) {
    analogWrite(in1, 255);
    analogWrite(in2, 255 - speed);
  } else if (speed < 0) {
    analogWrite(in2, 255);
    analogWrite(in1, 255 + speed);
  } else {
    analogWrite(in1, 0);
    analogWrite(in2, 0);
  }
}

void drive(int left, int right) {
  wheel(AIN1, AIN2, left);
  wheel(BIN1, BIN2, right);
}

// 네 입력 모두 HIGH = 전기적 제동 (정지 상태 유지)
void brake() {
  analogWrite(AIN1, 255);
  analogWrite(AIN2, 255);
  analogWrite(BIN1, 255);
  analogWrite(BIN2, 255);
}

void setSpeed(int speed) {
  if (speed == 0) {
    if (prevSpeed != 0) brake();       // 달리다 멈출 때 급정지
  } else if (prevSpeed == 0 && !alert) {
    drive(255, 255);                   // 출발할 때만 잠깐 강하게 (비상 중에는 생략)
    delay(KICK_MS);
    drive(speed, speed);
  } else if (prevSpeed == 0) {
    drive(speed, speed);               // 비상 중 출발은 kick 없이
  } else if (speed != prevSpeed) {
    drive(speed, speed);               // 속도만 변경 (G <-> L)
  }
  prevSpeed = speed;
}

// ---------- 카메라(Python) ----------
// main.py가 0.1~0.3초마다 호출: seen = 인형 인식 여부
void set_target(int seen, int err) {
  cmdSeen = seen != 0 ? 1 : 0;
  lastPyMs = millis();
  hasPy = true;
}

// ---------- 마스터 UART (Serial1 = D0/D1) ----------
void handleCommand(char* s) {
  while (*s == ' ') s++;

  if (s[0] == 'G') {
    limitMode = false;
    goRequested = true;
    gotCommand = true;
  } else if (s[0] == 'L') {
    limitMode = true;
    gotCommand = true;
  }
}

void readMaster() {
  while (Serial1.available()) {
    char c = Serial1.read();
    if (c == '\n') {
      mbuf[mlen] = '\0';
      handleCommand(mbuf);
      mlen = 0;
    } else if (c != '\r') {
      if (mlen < sizeof(mbuf) - 1) mbuf[mlen++] = c;
      else mlen = 0;
    }
  }
}

void setup() {
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  drive(0, 0);

  // UNO Q에서 D0/D1 핀은 Serial1 (Serial은 내부 Bridge/Monitor용)
  Serial1.begin(57600);

  Bridge.begin();
  Monitor.begin(115200);
  Bridge.provide_safe("set_target", set_target);

  Wire.begin();
  Wire.setClock(100000);
  delay(200);

  while (!tof.begin(0x29, false, &Wire)) {
    drive(0, 0);
    Monitor.println("VL53L0X not found - check wiring");
    delay(1000);
  }
  tof.configSensor(Adafruit_VL53L0X::VL53L0X_SENSE_LONG_RANGE);

  Serial1.println("READY");
  lastReadyMs = millis();
  Monitor.println("SLAVE1 ready");
}

void loop() {
  readMaster();

  // 1) 거리 측정
  VL53L0X_RangingMeasurementData_t m = {};
  tof.rangingTest(&m, false);

  int distance_mm;
  if (m.RangeStatus == 0) {
    distance_mm = (int)m.RangeMilliMeter;
  } else if (m.RangeStatus == 2 || m.RangeStatus == 4) {
    distance_mm = FAR_MM;                // 범위 밖 = 멀리 있음
  } else {
    distance_mm = -1;                    // 센서 오류
  }

  readMaster();

  // 2) 상황 판단
  unsigned long now = millis();
  bool pythonAlive = hasPy && (now - lastPyMs < PY_WATCHDOG_MS);
  int seen = pythonAlive ? cmdSeen : 0;

  // 비상 시작: 인형이 보이고 200mm 이하 (센서 오류도 가까운 것으로 간주)
  if (seen && (distance_mm < 0 || distance_mm <= ALERT_MM)) {
    alert = true;
  }
  // 비상 해제: 250mm보다 멀어짐
  else if (distance_mm > ALERT_CLEAR_MM) {
    alert = false;
  }

  // 정지: 비상 상태에서 100mm 이하
  if (alert && (distance_mm < 0 || distance_mm <= STOP_MM)) {
    driveAllowed = false;                // G 받기 전까지 정지 유지
  }

  // 재출발: 비상이 풀린 상태에서 G를 받았을 때만
  if (goRequested) {
    goRequested = false;
    if (!alert) {
      driveAllowed = true;
    }
  }

  int state;
  if (!pythonAlive)        state = ST_NO_APP;
  else if (!driveAllowed)  state = alert ? ST_STOP : ST_WAIT_GO;
  else if (alert)          state = ST_ALERT;
  else if (limitMode)      state = ST_LIMIT;
  else                     state = ST_RUN;

  // 3) 구동
  int speed = 0;
  if (state == ST_RUN || state == ST_LIMIT || state == ST_ALERT) {
    speed = limitMode ? LIMIT_PWM : NORMAL_PWM;
  }
  setSpeed(speed);

  // 4) 마스터로 보고 (Serial1 = D1 핀)
  now = millis();

  if (!gotCommand && now - lastReadyMs >= READY_MS) {
    Serial1.println("READY");            // 마스터가 늦게 켜져도 받을 수 있게
    lastReadyMs = now;
  }

  if (now - lastReportMs >= REPORT_MS) {
    lastReportMs = now;

    Serial1.println(alert ? "P1" : "P0"); // 비상 여부
    Serial1.print('D');                   // 현재 거리 -> 마스터가 근접 판단
    Serial1.println(distance_mm);

    // Python 로그용
    Bridge.notify("telemetry", distance_mm, seen, state, speed, limitMode ? 1 : 0);
  }

  delay(20);
}
