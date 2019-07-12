//pro mini用
//10,11,12,13: ４ビットカウンタ出力
//          2: スタート/ストップボタン
//          8: 高速モードボタン
//          7: クロック出力

//pro micro用
//15,14,16,10: ４ビットカウンタ出力
//          2: スタート/ストップボタン
//          8: 高速モードボタン
//          7: クロック出力

#include <limits.h>

#if defined(__AVR_ATmega32U4__)
#define LED_BIT1 15
#define LED_BIT2 14
#define LED_BIT3 16
#define LED_BIT4 10
#define LED_CLK 7
#define START_BUTTON 2
#define DASH_BUTTON 8
#else
#define LED_BIT1 13
#define LED_BIT2 12
#define LED_BIT3 11
#define LED_BIT4 10
#define LED_CLK 7
#define START_BUTTON 2
#define DASH_BUTTON 8
#endif

unsigned char n;
int buttonState = LOW; 
unsigned long interruptTime = 0;


void ledUpdate(int leds, int no, int gpio) {
  if (leds & no) {
    digitalWrite(gpio, HIGH);
  } else {
    digitalWrite(gpio, LOW);
  }
}

void setup() {
  Serial.begin(115200);
  n = 0;
  pinMode(LED_BIT1, OUTPUT);
  pinMode(LED_BIT2, OUTPUT);
  pinMode(LED_BIT3, OUTPUT);
  pinMode(LED_BIT4, OUTPUT);
  pinMode(LED_CLK, OUTPUT);
  digitalWrite(LED_CLK, HIGH);
  pinMode(START_BUTTON, INPUT_PULLUP);
  pinMode(DASH_BUTTON, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(START_BUTTON), startStop, LOW);
  Serial.println("initialization done.");
}

void loop() {
  ledUpdate(n, 0x01, LED_BIT1);
  ledUpdate(n, 0x02, LED_BIT2);
  ledUpdate(n, 0x04, LED_BIT3);
  ledUpdate(n, 0x08, LED_BIT4);
  for (int i=0;i<10;i++) {
    if (!buttonState) {
      delay(100);
    } else {
      if (!digitalRead(DASH_BUTTON)) {
        delay(5);
      } else {
        delay(90);
      }
    }
  }
  if (buttonState) {
    n = n + 1;
    digitalWrite(LED_CLK, LOW);
    delay(5);
    digitalWrite(LED_CLK, HIGH);
  }
}

void startStop() {
  unsigned long time = millis();
  unsigned char interval = 100;
  if ((time < time-interval && ULONG_MAX-(time-interval) + time > interval) || time-interval > interruptTime) {
    interruptTime = time;
    buttonState = !buttonState;
    Serial.println(time, DEC);
  }
}
