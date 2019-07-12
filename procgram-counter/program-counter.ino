unsigned char n;
int buttonState = 0; 

int leddata[] = {
  0x01,
  0x02,
  0x04,
  0x08,
};

void ledUpdate(int leds, int no, int gpio) {
  if (leds & no) {
    digitalWrite(gpio, HIGH);
  } else {
    digitalWrite(gpio, LOW);
  }
}

// the setup function runs once when you press reset or power the board
void setup() {
  // initialize digital pin LED_BUILTIN as an output.
  n = 0;
  pinMode(15, OUTPUT);
  pinMode(14, OUTPUT);
  pinMode(16, OUTPUT);
  pinMode(10, OUTPUT);
  pinMode(9, INPUT_PULLUP);
  pinMode(8, INPUT_PULLUP);
}

// the loop function runs over and over again forever
void loop() {
//  ledUpdate(leddata[n], 0x01, 15);
//  ledUpdate(leddata[n], 0x02, 14);
//  ledUpdate(leddata[n], 0x04, 16);
//  ledUpdate(leddata[n], 0x08, 10);
  ledUpdate(n, 0x01, 15);
  ledUpdate(n, 0x02, 14);
  ledUpdate(n, 0x04, 16);
  ledUpdate(n, 0x08, 10);
  for (int i=0;i<10;i++) {
    if (buttonState) {
      delay(100);
    } else {
      if (digitalRead(8)) {
        delay(10);
      } else {
        delay(100);
      }
    }
    buttonState = digitalRead(9);
  }
  if (!buttonState) {
    n = n + 1;
  }
//  if (n >= sizeof(leddata)/sizeof(int)) {
//    n = 0;
//  }
}
