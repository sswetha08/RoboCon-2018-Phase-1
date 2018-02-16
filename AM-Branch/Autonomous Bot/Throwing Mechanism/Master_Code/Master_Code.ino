#define BTSerial Serial1
#define BT_BAUD_RATE 38400
#define ENCODER_A 2
#define ENCODER_B 3
#define ENC_LIMIT 4000


volatile int count = ENC_LIMIT;
long temp = 0;

void setup() {
  // put your setup code here, to run once:
  pinMode(ENCODER_A, INPUT_PULLUP);
  pinMode(ENCODER_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A), adjustCount, RISING);
  BTSerial.begin(BT_BAUD_RATE);
  Serial.begin(9600);
  count = 0;
}

void adjustCount() {
  if (digitalRead(ENCODER_B) == LOW) {
    count --;
  } else {
    count ++;
  }
}

void loop() {
  // put your main code here, to run repeatedly:
  if (count != temp) {
    temp = count;
    BTSerial.write(count);
    Serial.println(count);
  }
}
