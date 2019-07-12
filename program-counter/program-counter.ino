//pro mini用
//10-13: ４ビットカウンタ出力
//    2: スタート/ストップボタン
//    8: 高速モードボタン
//    7: クロック出力

#include <limits.h>

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
  pinMode(13, OUTPUT);
  pinMode(12, OUTPUT);
  pinMode(11, OUTPUT);
  pinMode(10, OUTPUT);
  pinMode(7, OUTPUT);
  digitalWrite(7, LOW);
  pinMode(2, INPUT_PULLUP);
  pinMode(8, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(2), startStop, LOW);
  Serial.println("initialization done.");
}

void loop() {
  ledUpdate(n, 0x01, 13);
  ledUpdate(n, 0x02, 12);
  ledUpdate(n, 0x04, 11);
  ledUpdate(n, 0x08, 10);
  for (int i=0;i<10;i++) {
    if (!buttonState) {
      delay(100);
    } else {
      if (!digitalRead(8)) {
        delay(10);
      } else {
        delay(90);
      }
    }
  }
  if (buttonState) {
    n = n + 1;
    digitalWrite(7, HIGH);
    delay(10);
    digitalWrite(7, LOW);
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
