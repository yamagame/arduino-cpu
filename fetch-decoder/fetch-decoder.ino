#include <SPI.h>
#include <SD.h>

//入力：プログラムカウンタ
#define INP_BIT0 21
#define INP_BIT1 20
#define INP_BIT2 19
#define INP_BIT3 18
//出力：プログラムコード
#define LED_BIT0 6
#define LED_BIT1 7
#define LED_BIT2 8
#define LED_BIT3 9
//SD:CS
#define SDCS 10

unsigned char pc = 0;
unsigned char programdata[16];

void loadProgram() {
  File fp = SD.open("program.txt");
  for (int i=0;i<sizeof(programdata);i++) {
    programdata[i] = 0;
  }
  int p = 0;
  int t = 0;
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
          t = 0;
          //Serial.println("");
        }
        skip = 1;
      }
      if (sk == 0) {
        t ++;
        b = b << 1;
        if (ch != '0') {
          b |= 1;
        }
        //Serial.write(ch);
      }
    }
    if (t >= 4) {
      if (p < sizeof(programdata)) {
        programdata[p] = b;
      }
      p++;
    }
    {
      int i;
      int skip=0;
      for (i = 0;i < p;i++) {
        switch (programdata[i]) {
          case B0001: //JMP
          case B0010: //JNC
          case B1110: //LD
          case B1111: //ST
            skip = 1;
            break;
          case B1100: //HALT
            if (skip == 0) {
              i++;
              programdata[i] = B1111;
            }
            skip = 0;
            break;
          default:
            skip = 0;
            break;
        }
      }
    }
    fp.close();
  } else {
    //Serial.println("error opening program.txt");
  }
}

void setup() {
//  Serial.begin(115200);
//  while (!Serial) ;
//  Serial.println("Initializing SD card...");

  // put your setup code here, to run once:
  pinMode(INP_BIT0, INPUT);
  pinMode(INP_BIT1, INPUT);
  pinMode(INP_BIT2, INPUT);
  pinMode(INP_BIT3, INPUT);

  pinMode(LED_BIT0, OUTPUT);
  pinMode(LED_BIT1, OUTPUT);
  pinMode(LED_BIT2, OUTPUT);
  pinMode(LED_BIT3, OUTPUT);

  pinMode(SDCS, OUTPUT);
  digitalWrite(SDCS, HIGH);

  delay(1000);
  while (true) {
    if (SD.begin(SDCS)) {
      break;
    }
    digitalWrite(LED_BIT0, HIGH);
    delay(3000);
    digitalWrite(LED_BIT0, LOW);
  }

  loadProgram();

//  Serial.println("initialization done.");
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
  pc = ((digitalRead(INP_BIT3)) << 3) | ((digitalRead(INP_BIT2)) << 2) | ((digitalRead(INP_BIT1)) << 1) | (digitalRead(INP_BIT0));
  unsigned char val = programdata[pc & 0x0F];
  ledUpdate(val, 0x01, LED_BIT0);
  ledUpdate(val, 0x02, LED_BIT1);
  ledUpdate(val, 0x04, LED_BIT2);
  ledUpdate(val, 0x08, LED_BIT3);
}
