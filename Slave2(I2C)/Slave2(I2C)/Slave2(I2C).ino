// ============================================================
//  SLAVE2 (Uno) : 배터리 센서
//  - A0 : 가변저항 1 (전류)
//  - A1 : 가변저항 2 (잔량 손잡이)
//  - D2 : 충전 버튼 (반대쪽은 GND)
//  - I2C 주소 0x08 : 마스터가 요청하면 3바이트 전송
//        [0] 전류          0~255
//        [1] 손잡이 위치    0~100 (%)
//        [2] 버튼 누른 횟수 0~255 (255 다음은 0)
//  ※ 충전 / 따라잡기 계산은 마스터가 합니다.
// ============================================================
#include <Wire.h>

const byte I2C_ADDR = 0x08;
const int  BTN      = 2;

volatile byte current    = 0;
volatile byte potLevel   = 0;
volatile byte pressCount = 0;

bool lastBtn = HIGH;
unsigned long lastBtnTime = 0;

void sendData();

// 8번 읽어서 평균 (값 흔들림 줄이기)
int readAvg(int pin) {
  analogRead(pin);                       // 채널 바꾼 직후 첫 값은 버림
  long sum = 0;
  for (int i = 0; i < 8; i++) sum += analogRead(pin);
  return (int)(sum / 8);
}

void setup() {
  pinMode(BTN, INPUT_PULLUP);            // 안 누르면 HIGH, 누르면 LOW
  Wire.begin(I2C_ADDR);                  // I2C 슬레이브, 주소 8번
  Wire.onRequest(sendData);
  Serial.begin(9600);                    // 혼자 확인할 때만 사용
}

void loop() {
  unsigned long now = millis();

  // 1) 전류 (0~1023 → 0~255)
  current = (byte)(readAvg(A0) / 4);

  // 2) 손잡이 위치 (0~1023 → 0~100)
  potLevel = (byte)map(readAvg(A1), 0, 1023, 0, 100);

  // 3) 버튼: 눌린 순간만 1회, 0.2초 안의 떨림은 무시
  bool btn = digitalRead(BTN);
  if (btn == LOW && lastBtn == HIGH && now - lastBtnTime > 200) {
    pressCount++;
    lastBtnTime = now;
  }
  lastBtn = btn;

  // 4) 확인용 출력 (SLAVE2를 노트북에 연결했을 때 시리얼 모니터 9600)
  static unsigned long tDbg = 0;
  if (now - tDbg >= 500) {
    tDbg = now;
    Serial.print("A0 전류: ");   Serial.print(current);
    Serial.print("  A1 손잡이: "); Serial.print(potLevel);
    Serial.print("%  버튼 횟수: "); Serial.println(pressCount);
  }

  delay(5);
}

// 마스터 요청 시 실행 (짧게! analogRead, Serial, delay 금지)
void sendData() {
  byte buf[3] = { current, potLevel, pressCount };
  Wire.write(buf, 3);
}