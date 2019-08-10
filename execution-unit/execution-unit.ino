#if defined(__AVR_ATmega32U4__)
//入力：プログラムコード
#define INP_BIT0 15
#define INP_BIT1 14
#define INP_BIT2 16
#define INP_BIT3 10
//出力：レジスタ
#define LED_BIT0 6
#define LED_BIT1 7
#define LED_BIT2 8
#define LED_BIT3 9
//出力：Aレジスタ
#define LED_AREG 3
//出力：Bレジスタ
#define LED_BREG 4
//出力：フラッグ
#define LED_CFLAG 19
#define LED_JFLAG 18
//入力：クロック
#define INP_CLK 2
//BEEP音
#define BEEP_PIN 20
#define BEEP_TIM 50
#else
#endif

#define C_FLAG 0x01
#define J_FLAG 0x02
#define S_FLAG 0x04

unsigned char AReg = 0;
unsigned char BReg = 0;
unsigned char FReg = 0;
unsigned char RFlg = 0;

unsigned char ram[16] = { 0 };

int freq[16] = {
  196,
  220,
  247,
  262,

  294,
  330,
  349,
  392,

  440,
  494,
  523,
  587,

  659,
  698,
  784,
  880,
};
volatile int toneFreq = -1;

void execProg() {
  toneFreq = -1;
  if (digitalRead(INP_CLK)) return;
  toneFreq = BReg;
  int b0 = digitalRead(INP_BIT0);
  int b1 = digitalRead(INP_BIT1);
  int b2 = digitalRead(INP_BIT2);
  int b3 = digitalRead(INP_BIT3);
  unsigned char code = (b3 << 1) | b2;
  unsigned char func = (b1 << 1) | b0;
//  Serial.print(code, DEC);
//  Serial.print(",");
//  Serial.println(func, DEC);
  if (RFlg > 0) {
    unsigned char adr = (b3 << 3) | (b2 << 2) | (b1 << 1) | b0;
    switch (RFlg) {
      case 1:
        AReg = ram[adr];
        break;
      default:
        ram[adr] = AReg;
        break;
    }
    RFlg = 0;
    return;
  } else
  if (FReg & S_FLAG) {
    if (FReg & J_FLAG) {
      digitalWrite(LED_JFLAG, LOW);
    } else {
      digitalWrite(LED_JFLAG, HIGH);
    }
    FReg &= ~(S_FLAG | J_FLAG);
    return;
  }
  digitalWrite(LED_JFLAG, HIGH);
  switch (code) {
    case 0:
      switch (func) {
        case 0:  //NOP
          break;
        case 1:  //JUMP
          FReg |= (S_FLAG | J_FLAG);
          break;
        case 2:  //JNC
          FReg |= S_FLAG;
          if (!(FReg & C_FLAG)) {
            FReg |= J_FLAG;
          } else {
            FReg &= ~J_FLAG;
          }
          break;
        case 3:  //RND B
          BReg = random(0, 16);
          break;
      }
      break;
    case 1:   //MOV
      switch (func) {
        case 0:  //MOV A,0
          AReg = 0;
          break;
        case 1:  //MOV B,0
          BReg = 0;
          break;
        case 2:  //MOV A,B
          AReg = BReg;
          break;
        case 3:  //MOV B,A
          BReg = AReg;
          break;
      }
      break;
    case 2:   //ADD
      switch (func) {
        case 0:  //ADD A,1
          AReg ++;
          if (AReg > 0x0f) {
            FReg |= C_FLAG;
          } else {
            FReg &= ~(C_FLAG);
          }
          AReg &= 0x0f;
          break;
        case 1:  //ADD A,2
          AReg += 2;
          if (AReg > 0x0f) {
            FReg |= C_FLAG;
          } else {
            FReg &= ~(C_FLAG);
          }
          AReg &= 0x0f;
          break;
        case 2:  //ADD A,A
          AReg += AReg;
          if (AReg > 0x0f) {
            FReg |= C_FLAG;
          } else {
            FReg &= ~(C_FLAG);
          }
          AReg &= 0x0f;
          break;
        case 3:  //ADD A,B
          AReg += BReg;
          if (AReg > 0x0f) {
            FReg |= C_FLAG;
          } else {
            FReg &= ~(C_FLAG);
          }
          AReg &= 0x0f;
          break;
      }
      break;
    case 3:   //OTHER
      switch (func) {
        case 0:  //HALT
          FReg |= (S_FLAG | J_FLAG);
          break;
        case 1:  //
          break;
        case 2:  //LD
          RFlg = 1;
          break;
        case 3:  //ST
          RFlg = 2;
          break;
      }
      break;
  }
}

void setup() {
//  Serial.begin(115200);
//  while (!Serial) ;
//  Serial.println("Initializing...");

  int i;
  for (i = 0;i < sizeof(ram) ; i++) {
    ram[i] = 0;
  }

  pinMode(INP_BIT0, INPUT);
  pinMode(INP_BIT1, INPUT);
  pinMode(INP_BIT2, INPUT);
  pinMode(INP_BIT3, INPUT);

  pinMode(LED_BIT0, OUTPUT);
  pinMode(LED_BIT1, OUTPUT);
  pinMode(LED_BIT2, OUTPUT);
  pinMode(LED_BIT3, OUTPUT);

  pinMode(LED_AREG, OUTPUT);
  pinMode(LED_BREG, OUTPUT);

  pinMode(BEEP_PIN, OUTPUT);

  pinMode(LED_CFLAG, OUTPUT);
  pinMode(LED_JFLAG, OUTPUT);
  digitalWrite(LED_JFLAG, HIGH);

  delay(1000);

  pinMode(INP_CLK, INPUT);
  attachInterrupt(digitalPinToInterrupt(INP_CLK), execProg, CHANGE);

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
  digitalWrite(LED_AREG, LOW);
  digitalWrite(LED_BREG, HIGH);
  ledUpdate(AReg, 0x01, LED_BIT0);
  ledUpdate(AReg, 0x02, LED_BIT1);
  ledUpdate(AReg, 0x04, LED_BIT2);
  ledUpdate(AReg, 0x08, LED_BIT3);
  delay(5);

  digitalWrite(LED_AREG, HIGH);
  digitalWrite(LED_BREG, LOW);
  ledUpdate(BReg, 0x01, LED_BIT0);
  ledUpdate(BReg, 0x02, LED_BIT1);
  ledUpdate(BReg, 0x04, LED_BIT2);
  ledUpdate(BReg, 0x08, LED_BIT3);
  delay(5);

  ledUpdate(FReg, C_FLAG, LED_CFLAG);

  if (toneFreq >= 0) {
    tone(BEEP_PIN, freq[toneFreq], BEEP_TIM);
    toneFreq = -1;
  }
}
