#include <SPI.h>
#include <SD.h>

#if defined(__AVR_ATmega32U4__)
//入力：プログラムカウンタ
#define INP_BIT0 15
#define INP_BIT1 14
#define INP_BIT2 16
#define INP_BIT3 10
//出力：プログラムコード
#define LED_BIT0 5
#define LED_BIT1 6
#define LED_BIT2 7
#define LED_BIT3 8
#define LED_BIT4 9
//再読み込みボタン
#define LOAD_BUTTON 2
#else
//入力：プログラムカウンタ
#define INP_BIT0 13
#define INP_BIT1 12
#define INP_BIT2 11
#define INP_BIT3 10
//出力：プログラムコード
#define LED_BIT0 5
#define LED_BIT1 6
#define LED_BIT2 7
#define LED_BIT3 8
#define LED_BIT4 9
//再読み込みボタン
#define LOAD_BUTTON 2
#endif

unsigned char pc = 0;
unsigned char programdata[16];

void loadProgram() {
  File fp = SD.open("program.txt");
  for (int i=0;i<sizeof(programdata);i++) {
    programdata[i] = 0;
  }
  int p = 0;
  char skip = 0;
  if (fp) {
    unsigned char b = 0;
    while (fp.available()) {
      char ch = fp.read();
      char sk = 0;
      if (ch != '0' && ch != '1') {
        sk = 1;
      }
      if (!sk) {
        skip = 0;
      } else {
        if (skip == 0 && (ch == '\n' || ch == '\r')) {
          if (p < sizeof(programdata)) {
            programdata[p] = b;
          }
          p ++;
          b = 0;
          Serial.println("");
        }
        skip = 1;
      }
      if (sk == 0) {
        b = b << 1;
        if (ch != '0') {
          b |= 1;
        }
        Serial.write(ch);
      }
    }
    fp.close();
  } else {
    Serial.println("error opening program.txt");
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println("Initializing SD card...");

  if (!SD.begin(4)) {
    Serial.println("initialization failed!");
    while (1);
  }
  Serial.println("initialization done.");
  
  // put your setup code here, to run once:
  pinMode(INP_BIT0, INPUT_PULLUP);
  pinMode(INP_BIT1, INPUT_PULLUP);
  pinMode(INP_BIT2, INPUT_PULLUP);
  pinMode(INP_BIT3, INPUT_PULLUP);

  pinMode(LED_BIT0, OUTPUT);
  pinMode(LED_BIT1, OUTPUT);
  pinMode(LED_BIT2, OUTPUT);
  pinMode(LED_BIT3, OUTPUT);
  pinMode(LED_BIT4, OUTPUT);

  pinMode(LOAD_BUTTON, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(LOAD_BUTTON), loadProgram, LOW);

  loadProgram();
}

void ledUpdate(int leds, int no, int gpio) {
  if (leds & no) {
    digitalWrite(gpio, HIGH);
  } else {
    digitalWrite(gpio, LOW);
  }
}

void loop() {
  // put your main code here, to run repeatedly:
  pc = ((!digitalRead(INP_BIT3)) << 3) | ((!digitalRead(INP_BIT2)) << 2) | ((!digitalRead(INP_BIT1)) << 1) | (!digitalRead(INP_BIT0));
  unsigned char val = programdata[pc & 0x0F];
  ledUpdate(val, 0x01, LED_BIT0);
  ledUpdate(val, 0x02, LED_BIT1);
  ledUpdate(val, 0x04, LED_BIT2);
  ledUpdate(val, 0x08, LED_BIT3);
  ledUpdate(val, 0x10, LED_BIT4);
}
