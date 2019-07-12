#if defined(__AVR_ATmega32U4__)
//入力：プログラムコード
#define INP_BIT0 18
#define INP_BIT1 15
#define INP_BIT2 14
#define INP_BIT3 16
#define INP_BIT4 10
//出力：レジスタ
#define LED_BIT0 6
#define LED_BIT1 7
#define LED_BIT2 8
#define LED_BIT3 9
//出力：Aレジスタ
#define LED_AREG 4
//出力：Bレジスタ
#define LED_BREG 5
//出力：フラッグ
#define LED_FLAG 3
//入力：クロック
#define INP_CLK 2
#else
//入力：プログラムコード
#define INP_BIT0 A0
#define INP_BIT1 13
#define INP_BIT2 12
#define INP_BIT3 11
#define INP_BIT4 10
//出力：レジスタ
#define LED_BIT0 6
#define LED_BIT1 7
#define LED_BIT2 8
#define LED_BIT3 9
//出力：Aレジスタ
#define LED_AREG 4
//出力：Bレジスタ
#define LED_BREG 5
//出力：フラッグ
#define LED_FLAG 3
//入力：クロック
#define INP_CLK 2
#endif

unsigned char AReg = 0;
unsigned char BReg = 0;
unsigned char FReg = 0;

void execProg() {
  int b0 = !digitalRead(INP_BIT0);
  int b1 = !digitalRead(INP_BIT1);
  int b2 = !digitalRead(INP_BIT2);
  int b3 = !digitalRead(INP_BIT3);
  int b4 = !digitalRead(INP_BIT4);
  unsigned char code = (b4 << 2) | (b3 << 1) | b2;
  unsigned char func = (b1 << 1) | b0;
  switch (code) {
    case 0:   //NOP
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
          break;
        case 1:  //ADD A,2
          AReg += 2;
          break;
        case 2:  //ADD A,A
          AReg += AReg;
          break;
        case 3:  //ADD A,B
          AReg += BReg;
          break;
      }
      break;
    case 4:   //OUT
      switch (func) {
        case 0:  //ACT BEEP
          break;
        case 1:
          break;
        case 2:
          break;
        case 3:
          break;
      }
      break;
  }
}

void setup() {
  Serial.begin(115200);

  pinMode(INP_BIT0, INPUT);
  pinMode(INP_BIT1, INPUT);
  pinMode(INP_BIT2, INPUT);
  pinMode(INP_BIT3, INPUT);
  pinMode(INP_BIT4, INPUT);

  pinMode(LED_BIT0, OUTPUT);
  pinMode(LED_BIT1, OUTPUT);
  pinMode(LED_BIT2, OUTPUT);
  pinMode(LED_BIT3, OUTPUT);

  pinMode(LED_AREG, OUTPUT);
  pinMode(LED_BREG, OUTPUT);

  pinMode(LED_FLAG, OUTPUT);

  pinMode(INP_CLK, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(INP_CLK), execProg, LOW);
}

void ledUpdate(int leds, int no, int gpio) {
  if (leds & no) {
    digitalWrite(gpio, HIGH);
  } else {
    digitalWrite(gpio, LOW);
  }
}

void loop() {
  ledUpdate(AReg, 0x01, LED_BIT0);
  ledUpdate(AReg, 0x02, LED_BIT1);
  ledUpdate(AReg, 0x04, LED_BIT2);
  ledUpdate(AReg, 0x08, LED_BIT3);
  digitalWrite(LED_AREG, LOW);
  digitalWrite(LED_BREG, HIGH);

  ledUpdate(BReg, 0x01, LED_BIT0);
  ledUpdate(BReg, 0x02, LED_BIT1);
  ledUpdate(BReg, 0x04, LED_BIT2);
  ledUpdate(BReg, 0x08, LED_BIT3);
  digitalWrite(LED_AREG, HIGH);
  digitalWrite(LED_BREG, LOW);

  ledUpdate(FReg, 0x01, LED_FLAG);
}
