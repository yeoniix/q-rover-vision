// ============================================================
//  SLAVE3 (Uno) : 계기판
//  - SPI 슬레이브 (D10~13) ← MASTER : [0xAA, 상태, 잔량, 충전중, 거리cm]
//      상태: 0 정상 / 1 과전류 감속 / 2 사람 감지 정지 / 3 Q 준비 안 됨
//            4 출발 대기(GO) / 5 물체 가까움 감속 / 6 사람 사라짐(GO로 재출발)
//      거리: 0~150 cm / 254 = 멀리 / 255 = 모름
//  - D3      : 빨강 LED (정지 = 켜짐, 감속 = 깜빡임)
//  - D5 / D6 : 3색 LED R / G (공통 애노드: 긴 다리 → 5V, LOW = 켜짐)
//  - A4 / A5 : I2C LCD 16x2 (주소 자동 찾기, 라이브러리 필요 없음)
// ============================================================
#include <Wire.h>

const int RED = 3, RGB_R = 5, RGB_G = 6;
const byte PKT_LEN = 5;

// ----- SPI 수신 (인터럽트에서 사용) -----
volatile byte rxBuf[PKT_LEN];
volatile byte rxIdx = 0;
volatile byte pktState = 0, pktLevel = 0, pktCharging = 0, pktDist = 255;
volatile bool newPkt = false;
unsigned long lastPktTime = 0;
bool everReceived = false;

// 1바이트 받을 때마다 자동 실행
ISR(SPI_STC_vect) {
  byte c = SPDR;
  if (c == 0xAA) { rxIdx = 1; return; }  // 시작 표시 → 처음부터 (데이터에는 0xAA가 나올 수 없음)
  if (rxIdx == 0) return;                // 시작 표시 전 바이트 무시
  rxBuf[rxIdx++] = c;
  if (rxIdx == PKT_LEN) {
    pktState    = rxBuf[1];
    pktLevel    = rxBuf[2];
    pktCharging = rxBuf[3];
    pktDist     = rxBuf[4];
    newPkt = true;
    rxIdx = 0;
  }
}

// ============================================================
//  I2C LCD 직접 제어 (PCF8574 기판, 라이브러리 없이)
//  P0=RS, P1=RW, P2=EN, P3=백라이트, P4~P7=데이터
byte lcdAddr = 0;
const byte LCD_RS = 0x01, LCD_EN = 0x04, LCD_BL = 0x08;

void pcfWrite(byte d) {
  Wire.beginTransmission(lcdAddr);
  Wire.write(d | LCD_BL);
  Wire.endTransmission();
}
void lcdWrite4(byte nibble, byte mode) {
  byte d = (nibble & 0xF0) | mode;
  pcfWrite(d);
  pcfWrite(d | LCD_EN);  delayMicroseconds(1);
  pcfWrite(d & ~LCD_EN); delayMicroseconds(50);
}
void lcdSend(byte v, byte mode) {
  lcdWrite4(v & 0xF0, mode);
  lcdWrite4((byte)(v << 4), mode);
}
void lcdCmd(byte c) {
  lcdSend(c, 0);
  if (c == 0x01 || c == 0x02) delay(2);
}
void lcdLine(byte row, const char *s) {       // 한 줄(16칸)을 통째로 쓰기
  if (!lcdAddr) return;
  lcdCmd(0x80 | (row ? 0x40 : 0x00));
  bool ended = false;
  for (byte i = 0; i < 16; i++) {
    if (!ended && s[i] == '\0') ended = true;
    lcdSend(ended ? ' ' : s[i], LCD_RS);
  }
}
bool i2cFound(byte a) {
  Wire.beginTransmission(a);
  return Wire.endTransmission() == 0;
}
void lcdInit() {
  const byte first[] = { 0x27, 0x3F };
  for (byte i = 0; i < 2 && !lcdAddr; i++) if (i2cFound(first[i])) lcdAddr = first[i];
  for (byte a = 0x20; a <= 0x27 && !lcdAddr; a++) if (i2cFound(a)) lcdAddr = a;
  for (byte a = 0x38; a <= 0x3F && !lcdAddr; a++) if (i2cFound(a)) lcdAddr = a;
  if (!lcdAddr) return;

  delay(50);
  pcfWrite(0); delay(5);
  lcdWrite4(0x30, 0); delay(5);
  lcdWrite4(0x30, 0); delayMicroseconds(150);
  lcdWrite4(0x30, 0); delayMicroseconds(150);
  lcdWrite4(0x20, 0);                        // 4비트 모드
  lcdCmd(0x28);                              // 2줄, 5x8 글꼴
  lcdCmd(0x0C);                              // 화면 켜기, 커서 숨김
  lcdCmd(0x06);
  lcdCmd(0x01);                              // 화면 지우기
}

// ============================================================
//  3색 LED (공통 애노드: LOW = 켜짐)
void rgb(bool r, bool g) {
  digitalWrite(RGB_R, r ? LOW : HIGH);
  digitalWrite(RGB_G, g ? LOW : HIGH);
}

// ============================================================
void setup() {
  pinMode(RGB_R, OUTPUT); digitalWrite(RGB_R, HIGH);   // 먼저 꺼두기
  pinMode(RGB_G, OUTPUT); digitalWrite(RGB_G, HIGH);
  pinMode(RED, OUTPUT);   digitalWrite(RED, LOW);

  Serial.begin(9600);
  Wire.begin();
  lcdInit();
  if (lcdAddr) {
    Serial.print("LCD 찾음: 0x"); Serial.println(lcdAddr, HEX);
    lcdLine(0, "SLAVE3 READY");
    lcdLine(1, "WAITING MASTER");
  } else {
    Serial.println("LCD 못 찾음 → SDA=A4, SCL=A5, 5V, GND 확인");
  }

  // SPI 슬레이브 설정 (SPI.begin()은 마스터용이라 쓰지 않음)
  pinMode(MISO, OUTPUT);
  SPCR = _BV(SPE) | _BV(SPIE);               // SPI 켜기 + 받을 때마다 인터럽트
}

// ============================================================
void loop() {
  static byte state = 0, level = 0, charging = 0, dist = 255;
  static long lastKey = -1;
  unsigned long now = millis();

  // 1) 새 패킷 꺼내기
  if (newPkt) {
    noInterrupts();
    state = pktState; level = pktLevel; charging = pktCharging; dist = pktDist;
    newPkt = false;
    interrupts();
    if (level > 100) level = 100;
    lastPktTime = now;
    everReceived = true;
  }
  bool noSignal = !everReceived || (now - lastPktTime > 1000);

  // 2) 빨강 LED: 정지(사람 감지, 재출발 대기) = 켜짐 / 감속(과전류, 물체 가까움) = 깜빡임
  bool blink = (now / 250) % 2;
  if (noSignal)                         digitalWrite(RED, LOW);
  else if (state == 2 || state == 6)    digitalWrite(RED, HIGH);
  else if (state == 1 || state == 5)    digitalWrite(RED, blink ? HIGH : LOW);
  else                                  digitalWrite(RED, LOW);

  // 3) 3색 LED: 잔량 (60 초과 초록 / 30~60 노랑 / 30 미만 빨강)
  if (noSignal)          rgb(false, false);
  else if (level > 60)   rgb(false, true);
  else if (level >= 30)  rgb(true,  true);
  else                   rgb(true,  false);

  // 4) LCD: 내용이 바뀔 때만 다시 쓰기
  long key = noSignal ? -2 : ((long)state * 1000000L + (long)level * 10000L + (long)dist * 10L + charging);
  if (key != lastKey) {
    lastKey = key;
    char l1[17], l2[17], d[6];
    if (noSignal) {
      snprintf(l1, sizeof(l1), "%s", everReceived ? "SIGNAL LOST" : "WAITING MASTER");
      snprintf(l2, sizeof(l2), "%s", "CHECK SPI WIRE");
    } else {
      // 1줄: "BAT 85% 25cm CHG"
      if (dist == 254)      snprintf(d, sizeof(d), "FAR");
      else if (dist == 255) snprintf(d, sizeof(d), "--");
      else                  snprintf(d, sizeof(d), "%dcm", dist);
      snprintf(l1, sizeof(l1), "BAT%3d%% %-5s%s", level, d, charging ? "CHG" : "");

      // 2줄: 상태
      const char *msg;
      switch (state) {
        case 0:  msg = "NORMAL";           break;
        case 1:  msg = "WARN:OVERCURRENT"; break;
        case 2:  msg = "PERSON! STOP";     break;
        case 3:  msg = "WAITING UNO Q";    break;
        case 4:  msg = "READY: PRESS GO";  break;
        case 5:  msg = "OBJECT NEAR:SLOW"; break;
        case 6:  msg = "CLEAR: PRESS GO";  break;
        default: msg = "UNKNOWN";          break;
      }
      snprintf(l2, sizeof(l2), "%s", msg);
    }
    lcdLine(0, l1);
    lcdLine(1, l2);
  }
}