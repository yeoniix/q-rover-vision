// ============================================================
//  MASTER (Uno) : 판단 담당
//
//  [UART] UNO Q ↔ MASTER (SoftwareSerial, 57600bps, 줄 끝 '\n')
//     Q → MASTER : "READY"  부팅 완료
//                  "P1"     사람(인형) 감지 → 비상
//                  "P0"     사람 없음
//                  "D<mm>"  TOF 거리 (예: D150, -1 = 센서 오류)
//     MASTER → Q : 'G' 정상 주행 / 'L' 감속(가속 제한)
//                  아무것도 안 보냄 = Q는 대기(WAIT_G) 유지
//
//  [I2C]  SLAVE2 → MASTER : [전류 0~255, 손잡이 0~100, 버튼 횟수]
//  [SPI]  MASTER → SLAVE3 : [0xAA, 상태, 잔량, 충전중, 거리cm]
//
//  [버튼] D3 GO 버튼 : 처음 출발 / 사람 감지 후 재출발
//
//  상태 코드 (SLAVE3 표시용)
//    0 정상 / 1 과전류 감속 / 2 사람 감지 정지 / 3 Q 준비 안 됨
//    4 출발 대기(GO) / 5 물체 가까움 감속 / 6 사람 사라짐(GO 누르면 재출발)
// ============================================================
#include <Wire.h>
#include <SPI.h>
#include <SoftwareSerial.h>

// ★ UNO Q를 연결하지 않고 테스트할 때는 false
const bool USE_Q = true;

// ----- 핀 -----
SoftwareSerial qLink(4, 5);           // RX=D4 (Q TX), TX=D5 (Q RX)
const int BTN_GO = 3;                 // GO 버튼 (반대쪽은 GND)
const int SS_S3  = 10;                // SLAVE3 선택 핀

// ----- 설정값 -----
const byte SLAVE2_ADDR   = 0x08;
const byte OC_ON         = 200;       // 과전류 시작: 가변저항 1 약 78%
const byte OC_OFF        = 188;       // 과전류 해제: 약 73% 아래
const int  CATCH         = 3;         // 잔량 따라잡기 범위 ±3%
const unsigned long CHARGE_STEP = 300;   // 충전: 0.3초마다 +1%
const int  NEAR_MM       = 200;       // 이 거리 이하면 감속
const int  NEAR_CLEAR_MM = 250;       // 이 거리보다 멀어지면 감속 해제

const byte ST_NORMAL = 0, ST_OVERCUR = 1, ST_PERSON = 2, ST_QWAIT = 3,
           ST_WAITGO = 4, ST_NEAR = 5, ST_CLEAR = 6;

// ----- UNO Q 상태 -----
bool qReady    = false;
bool emergency = false;               // P1 받으면 true, GO 버튼으로 해제
bool lastP0    = false;               // 마지막 사람 메시지가 P0인가
bool nearObj   = false;
int  distMm    = -2;                  // -2 = 아직 못 받음, -1 = 센서 오류
char qbuf[16];
byte qlen = 0;
char lastCmd = 0;
unsigned long lastSendMs = 0;

// ----- SLAVE2 / 배터리 -----
byte current  = 0;
byte potLevel = 0;
byte lastCnt  = 0;
bool s2ok     = false;
bool s2synced = false;
byte level      = 100;
bool charging   = false;
bool knobLinked = true;
unsigned long lastChargeTime = 0;

// ----- 그 외 -----
bool overCurrent = false;
bool started     = false;             // GO 버튼으로 출발했는가

// 함수 미리 선언
void handleQ(char *s);
void readQ();
char decideCmd();
void readSlave2();
void updateBattery(unsigned long now);
void checkGoButton(unsigned long now);
byte decideState();
byte distToCm();
void sendSlave3(byte state);
void printDebug(byte state);

// ============================================================
void setup() {
  Serial.begin(9600);                 // 디버그 (시리얼 모니터 9600)
  if (USE_Q) qLink.begin(57600);

  Wire.begin();
  Wire.setWireTimeout(3000, true);    // SLAVE2 응답 없으면 3ms 후 포기

  pinMode(SS_S3, OUTPUT);
  digitalWrite(SS_S3, HIGH);
  SPI.begin();

  pinMode(BTN_GO, INPUT_PULLUP);
  Serial.println("MASTER 시작 - GO 버튼을 누르면 출발");
}

// ============================================================
void loop() {
  unsigned long now = millis();

  // 1) UNO Q 메시지 읽기
  if (USE_Q) readQ();

  // 2) SLAVE2 읽기 + 잔량 계산 (0.1초마다)
  static unsigned long tS2 = 0;
  if (now - tS2 >= 100) {
    tS2 = now;
    readSlave2();
    updateBattery(now);
  }

  // 3) 과전류 판단 (히스테리시스)
  if (!overCurrent && current >= OC_ON)  overCurrent = true;
  if ( overCurrent && current <  OC_OFF) overCurrent = false;

  // 4) GO 버튼
  checkGoButton(now);

  // 5) UNO Q에 명령 (바뀌면 바로, 같으면 0.5초마다 다시)
  if (USE_Q) {
    char cmd = decideCmd();
    if (cmd != 0 && (cmd != lastCmd || now - lastSendMs >= 500)) {
      qLink.write(cmd);
      qLink.write('\n');
      lastCmd = cmd;
      lastSendMs = now;
    }
    if (cmd == 0) lastCmd = 0;
  }

  // 6) SLAVE3에 표시 정보 (0.1초마다)
  byte state = decideState();
  static unsigned long tS3 = 0;
  if (now - tS3 >= 100) {
    tS3 = now;
    sendSlave3(state);
  }

  // 7) 디버그 출력 (0.5초마다)
  static unsigned long tDbg = 0;
  if (now - tDbg >= 500) {
    tDbg = now;
    printDebug(state);
  }
}

// ============================================================
//  UART: UNO Q 메시지 처리 (Q 팀원 참고 코드와 동일한 규칙)
void handleQ(char *s) {
  if (strcmp(s, "READY") == 0) {
    qReady = true;
  } else if (strcmp(s, "P1") == 0) {
    qReady = true;
    emergency = true;
    lastP0 = false;
  } else if (strcmp(s, "P0") == 0) {
    qReady = true;
    lastP0 = true;
  } else if (s[0] == 'D') {
    int d = atoi(s + 1);
    distMm = d;
    if (d < 0 || d <= NEAR_MM)  nearObj = true;    // 200mm 이하 (또는 센서 오류) → 감속
    else if (d > NEAR_CLEAR_MM) nearObj = false;   // 250mm 초과 → 해제
  }
  // 그 외(부팅 메시지 등)는 무시
}

void readQ() {
  while (qLink.available()) {
    char c = qLink.read();
    if (c == '\n') {
      qbuf[qlen] = '\0';
      handleQ(qbuf);
      qlen = 0;
    } else if (c != '\r') {
      if (qlen < sizeof(qbuf) - 1) qbuf[qlen++] = c;
      else qlen = 0;                               // 너무 긴 줄은 버림
    }
  }
}

// 지금 보내야 할 명령: 'L', 'G', 또는 0(아무것도 안 보냄)
char decideCmd() {
  if (!qReady)  return 0;
  if (!started) return 0;                 // GO 누르기 전 → 아무것도 안 보냄 (Q는 대기)
  if (overCurrent || nearObj) return 'L';
  if (emergency) return 0;                // 비상 후에는 GO 버튼 전까지 G 금지
  return 'G';
}

// ============================================================
//  GO 버튼: 처음 누르면 출발, 사람 감지 후에는 P0(사람 없음)일 때만 재출발
void checkGoButton(unsigned long now) {
  static bool last = HIGH;
  static unsigned long t = 0;
  bool b = digitalRead(BTN_GO);
  if (b == LOW && last == HIGH && now - t > 200) {
    t = now;
    if (!started) {
      started = true;
      Serial.println(">> GO: 출발");
    } else if (emergency && lastP0) {
      emergency = false;
      Serial.println(">> GO: 재출발");
    }
  }
  last = b;
}

// ============================================================
//  I2C: SLAVE2에서 3바이트 읽기
void readSlave2() {
  byte n = Wire.requestFrom(SLAVE2_ADDR, (byte)3);
  if (n != 3) {
    s2ok = false;
    while (Wire.available()) Wire.read();
    return;
  }
  byte cur = Wire.read();
  byte pot = Wire.read();
  byte cnt = Wire.read();
  s2ok = true;

  current  = cur;
  potLevel = (pot > 100) ? 100 : pot;

  if (!s2synced) {                        // 처음엔 기준만 맞춤
    s2synced   = true;
    lastCnt    = cnt;
    level      = potLevel;
    knobLinked = true;
    return;
  }

  byte presses = (byte)(cnt - lastCnt);   // 지난번보다 몇 번 더 눌렸나
  lastCnt = cnt;
  if (presses > 10) return;               // SLAVE2만 리셋된 경우 → 무시
  if (presses % 2 == 1) {                 // 홀수 번 → 충전 시작/멈춤
    charging = !charging;
    if (charging) {
      knobLinked     = false;
      lastChargeTime = millis();
    }
  }
}

//  잔량 계산: 충전 + 손잡이 따라잡기
void updateBattery(unsigned long now) {
  if (!s2synced) return;
  if (charging) {
    while (now - lastChargeTime >= CHARGE_STEP) {
      lastChargeTime += CHARGE_STEP;
      if (level < 100) level++;
    }
    if (level >= 100) {
      level = 100;
      charging = false;
    }
  } else {
    int diff = (int)potLevel - (int)level;
    if (diff < 0) diff = -diff;
    if (!knobLinked && diff <= CATCH) knobLinked = true;
    if (knobLinked) level = potLevel;
  }
}

// ============================================================
//  계기판에 보여줄 상태 (중요한 것부터)
byte decideState() {
  if (USE_Q && emergency) return lastP0 ? ST_CLEAR : ST_PERSON;
  if (overCurrent)              return ST_OVERCUR;
  if (USE_Q && nearObj)         return ST_NEAR;
  if (USE_Q && !qReady)         return ST_QWAIT;
  if (!started)                 return ST_WAITGO;
  return ST_NORMAL;
}

//  거리(mm) → SLAVE3용 1바이트 (cm, 0~150 / 254 = 멀리 / 255 = 모름·오류)
byte distToCm() {
  if (!USE_Q || distMm < 0) return 255;
  if (distMm >= 1500) return 254;
  return (byte)(distMm / 10);
}

//  SPI: SLAVE3에 5바이트 [0xAA, 상태, 잔량, 충전중, 거리cm]
//  (0xAA는 시작 표시. 다른 값들은 0xAA(170)가 될 수 없게 범위를 정함)
void sendSlave3(byte state) {
  byte pkt[5] = { 0xAA, state, level, (byte)(charging ? 1 : 0), distToCm() };
  SPI.beginTransaction(SPISettings(500000, MSBFIRST, SPI_MODE0));   // 500kHz
  digitalWrite(SS_S3, LOW);
  for (int i = 0; i < 5; i++) {
    SPI.transfer(pkt[i]);
    delayMicroseconds(40);                // SLAVE3가 1바이트 처리할 시간
  }
  digitalWrite(SS_S3, HIGH);
  SPI.endTransaction();
}

// ============================================================
void printDebug(byte state) {
  const char *names[] = { "정상", "과전류 감속", "사람 감지 정지", "Q 준비 안 됨",
                          "출발 대기(GO)", "물체 가까움 감속", "사람 사라짐(GO로 재출발)" };
  Serial.print("[");        Serial.print(names[state]); Serial.print("]");
  Serial.print("  전류: "); Serial.print(current);
  Serial.print("  손잡이: "); Serial.print(potLevel); Serial.print("%");
  Serial.print("  잔량: "); Serial.print(level);    Serial.print("%");
  if (charging)         Serial.print(" [충전 중]");
  else if (!knobLinked) Serial.print(" [따라잡는 중]");
  if (USE_Q) {
    Serial.print("  거리: ");
    if (distMm < 0) Serial.print("--");
    else { Serial.print(distMm); Serial.print("mm"); }
    Serial.print("  Q명령: ");
    Serial.print(lastCmd ? lastCmd : '-');
  }
  if (!s2ok) Serial.print("  (SLAVE2 응답 없음)");
  Serial.println();
}