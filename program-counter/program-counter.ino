//pro micro用
//15,14,16,10: ４ビットカウンタ出力
//21,20,19,18: ４ビットデータバス入力
//          7: クロック出力 - 実行ユニットへ
//          2: スタートボタン
//          3: ジャンプ - データバスの値をPCに入れる
//          9: 高速モードボタン

//pro mini用
//10,11,12,13: ４ビットカウンタ出力
//17,16,15,14: ４ビットデータバス入力
//          7: クロック出力 - 実行ユニットへ
//          2: スタートボタン
//          3: ジャンプ - データバスの値をPCに入れる
//          9: 高速モードボタン

#include <limits.h>

#if defined(__AVR_ATmega32U4__)
#define LED_BIT0 15
#define LED_BIT1 14
#define LED_BIT2 16
#define LED_BIT3 10
#define INP_BIT0 21
#define INP_BIT1 20
#define INP_BIT2 19
#define INP_BIT3 18
#define EXEC_CLK 7
#define START_BUTTON 2
#define JUMP_BUTTON 3
#define FAST_MODE 9
#else
#define LED_BIT0 13
#define LED_BIT1 12
#define LED_BIT2 11
#define LED_BIT3 10
#define INP_BIT0 17
#define INP_BIT1 16
#define INP_BIT2 15
#define INP_BIT3 14
#define EXEC_CLK 7
#define START_BUTTON 2
#define JUMP_BUTTON 3
#define FAST_MODE 9
#endif

volatile unsigned char counter = 0;
volatile unsigned long interruptTime = 0;
volatile int delayTime = 1000;
volatile int clockState = 0; 

void ledUpdate(int leds, int no, int gpio) {
  if (leds & no) {
    digitalWrite(gpio, HIGH);
  } else {
    digitalWrite(gpio, LOW);
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(LED_BIT0, OUTPUT);
  pinMode(LED_BIT1, OUTPUT);
  pinMode(LED_BIT2, OUTPUT);
  pinMode(LED_BIT3, OUTPUT);
  pinMode(INP_BIT0, INPUT);
  pinMode(INP_BIT1, INPUT);
  pinMode(INP_BIT2, INPUT);
  pinMode(INP_BIT3, INPUT);
  pinMode(FAST_MODE, INPUT);
  pinMode(EXEC_CLK, OUTPUT);
  digitalWrite(EXEC_CLK, HIGH);
  pinMode(START_BUTTON, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(START_BUTTON), startButton, CHANGE);
  pinMode(JUMP_BUTTON, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(JUMP_BUTTON), jumpButton, CHANGE);
  Serial.println("initialization done.");
}

void loop() {
  if (clockState == 1) {
    clockState = 2;
  }
  ledUpdate(counter, 0x01, LED_BIT0);
  ledUpdate(counter, 0x02, LED_BIT1);
  ledUpdate(counter, 0x04, LED_BIT2);
  ledUpdate(counter, 0x08, LED_BIT3);
  delay(delayTime/2);
  if (clockState > 1) {
    counter ++;
    digitalWrite(EXEC_CLK, LOW);
    delay(delayTime/10);
    digitalWrite(EXEC_CLK, HIGH);
    if (counter >= 16) {
      counter = 15;
      clockState = 0;
    }
  }
  delay(delayTime/2);
}

unsigned char readInput() {
  return ((digitalRead(INP_BIT3)) << 3) | ((digitalRead(INP_BIT2)) << 2) | ((digitalRead(INP_BIT1)) << 1) | (digitalRead(INP_BIT0));
}

void startButton() {
  if (!digitalRead(START_BUTTON)) {
    noInterrupts();
    if (buttonCommon()) {
    }
    interrupts();
  }
}

void jumpButton() {
  if (!digitalRead(JUMP_BUTTON)) {
    noInterrupts();
    if (buttonCommon()) {
      counter = readInput();
    }
    interrupts();
  }
}

boolean buttonCommon() {
  unsigned long time = millis();
  unsigned char interval = 100;
  if ((time < time-interval && ULONG_MAX-(time-interval) + time > interval) || time-interval > interruptTime) {
    if (!digitalRead(FAST_MODE)) {
      delayTime = 1000;
    } else {
      delayTime = 100;
    }
    interruptTime = time;
    clockState = 1;
    counter = 0;
    Serial.println(digitalRead(FAST_MODE), DEC);
    return true;
  }
  return false;
}
