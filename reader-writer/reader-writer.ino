#include <SPI.h>
#include <SD.h>

//#define DEBUG_

//入力：
#define LED_BIT0 9
#define LED_BIT1 8
#define LED_BIT2 7
#define LED_BIT3 6
//出力：
#define INP_BIT0 18
#define INP_BIT1 19
#define INP_BIT2 20
#define INP_BIT3 21
//SD:CS
#define SDCS 10
//モード
#define INP_MODE 3
#define INP_WRIT 2

#define LED_DATA 4
#define LED_PCNT 5

#define FALSE 0
#define TRUE 1

unsigned char pc = 0;
unsigned char val = 0;

#define PROGRAM_SIZE 16
unsigned char programdata0[PROGRAM_SIZE];
unsigned char programdata1[PROGRAM_SIZE];

volatile int writeSignal = 0; 
int buttonTrig = 0;

int loadProgram(const char* filename, unsigned char* programdata) {
  File fp = SD.open(filename);
  int p = 0;
  int t = 0;
  char skip = 0;
  if (fp) {
#ifdef DEBUG_
    Serial.println(filename);
#endif
    for (int i=0;i<PROGRAM_SIZE;i++) {
      programdata[i] = 0;
    }
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
          if (p < PROGRAM_SIZE) {
            programdata[p] = b;
          }
          p ++;
          b = 0;
          t = 0;
#ifdef DEBUG_
          Serial.println("");
#endif
        }
        skip = 1;
      }
      if (sk == 0) {
        t ++;
        b = b << 1;
        if (ch != '0') {
          b |= 1;
        }
#ifdef DEBUG_
        Serial.write(ch);
#endif
      }
    }
    if (t >= 4) {
      if (p < PROGRAM_SIZE) {
        programdata[p] = b;
      }
      p++;
    }
    fp.close();
  } else {
#ifdef DEBUG_
    Serial.print("error opening ");
    Serial.println(filename);
#endif
    return FALSE;
  }
  return TRUE;
}

int saveProgram(const char* filename, unsigned char* programdata) {
  if (SD.exists(filename)) {
    SD.remove(filename);
  }
  File fp = SD.open(filename, FILE_WRITE);
  if (fp) {
    for (int i=0;i<PROGRAM_SIZE;i++) {
      char t[5];
      int v = programdata[i];
      if (v & 0x08) { t[0] = '1'; } else { t[0] = '0'; }
      if (v & 0x04) { t[1] = '1'; } else { t[1] = '0'; }
      if (v & 0x02) { t[2] = '1'; } else { t[2] = '0'; }
      if (v & 0x01) { t[3] = '1'; } else { t[3] = '0'; }
      t[4] = 0;
      fp.println(t);
    }
    fp.close();
  } else {
#ifdef DEBUG_
    Serial.print("error opening ");
    Serial.println(filename);
#endif
    return FALSE;
  }
  return TRUE;
}

void setup() {
#ifdef DEBUG_
  Serial.begin(115200);
  while (!Serial) ;
  Serial.println("Initializing SD card...");
#endif

  pinMode(INP_BIT0, INPUT);
  pinMode(INP_BIT1, INPUT);
  pinMode(INP_BIT2, INPUT);
  pinMode(INP_BIT3, INPUT);

  pinMode(INP_MODE, INPUT);

  pinMode(LED_BIT0, OUTPUT);
  pinMode(LED_BIT1, OUTPUT);
  pinMode(LED_BIT2, OUTPUT);
  pinMode(LED_BIT3, OUTPUT);

  pinMode(LED_DATA, OUTPUT);
  pinMode(LED_PCNT, OUTPUT);

  pinMode(SDCS, OUTPUT);
  digitalWrite(SDCS, HIGH);

  pinMode(INP_WRIT, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(INP_WRIT), writeButton, CHANGE);

  delay(1000);
  while (true) {
    if (SD.begin(SDCS)) {
      break;
    }
#ifdef DEBUG_
    Serial.println("Retry SD begin...");
#endif
    digitalWrite(LED_BIT0, HIGH);
    delay(3000);
    digitalWrite(LED_BIT0, LOW);
  }

  loadProgram("program0.txt", programdata0);
  loadProgram("program1.txt", programdata1);

#ifdef DEBUG_
  Serial.println("initialization done.");
#endif
}

void ledUpdate(int leds, int no, int gpio) {
  if (leds & no) {
    digitalWrite(gpio, HIGH);
  } else {
    digitalWrite(gpio, LOW);
  }
}

void writeButton() {
  noInterrupts();
  if (digitalRead(INP_WRIT)) {
    writeSignal = 1;
  } else {
    writeSignal = 0;
  }
  interrupts();
}

void loop() {
  if (!digitalRead(INP_MODE)) {
    val = programdata0[pc];
  } else {
    if (writeSignal) {
      val = ((digitalRead(INP_BIT3)) << 3) | ((digitalRead(INP_BIT2)) << 2) | ((digitalRead(INP_BIT1)) << 1) | (digitalRead(INP_BIT0));
    } else {
      val = 0;
    }
  }

  if (writeSignal) {
    if (buttonTrig == 0) {
      buttonTrig = 1;
    }
    if (digitalRead(INP_MODE)) {
      programdata0[pc] = val;
    }
//    if (buttonTrig) {
//      val = ((digitalRead(INP_BIT3)) << 3) | ((digitalRead(INP_BIT2)) << 2) | ((digitalRead(INP_BIT1)) << 1) | (digitalRead(INP_BIT0));
//    }
  } else {
    if (buttonTrig) {
      if (digitalRead(INP_MODE)) {
        saveProgram("program0.txt", programdata0);
      }
      pc ++;
      if (pc >= 16) pc = 0;
    }
    buttonTrig = 0;
  }

  ledUpdate(0, 0x01, LED_BIT0);
  ledUpdate(0, 0x02, LED_BIT1);
  ledUpdate(0, 0x04, LED_BIT2);
  ledUpdate(0, 0x08, LED_BIT3);
  digitalWrite(LED_DATA, LOW);
  digitalWrite(LED_PCNT, HIGH);
  ledUpdate(val, 0x01, LED_BIT0);
  ledUpdate(val, 0x02, LED_BIT1);
  ledUpdate(val, 0x04, LED_BIT2);
  ledUpdate(val, 0x08, LED_BIT3);
  delay(10);

  ledUpdate(0, 0x01, LED_BIT0);
  ledUpdate(0, 0x02, LED_BIT1);
  ledUpdate(0, 0x04, LED_BIT2);
  ledUpdate(0, 0x08, LED_BIT3);
  digitalWrite(LED_DATA, HIGH);
  digitalWrite(LED_PCNT, LOW);
  ledUpdate(pc, 0x01, LED_BIT0);
  ledUpdate(pc, 0x02, LED_BIT1);
  ledUpdate(pc, 0x04, LED_BIT2);
  ledUpdate(pc, 0x08, LED_BIT3);
  delay(10);
}
