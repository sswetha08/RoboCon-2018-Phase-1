volatile int count = 0;
int i_state = 0;
int A = 2,B = 3;
void setup() {
  pinMode(A, INPUT_PULLUP);
  pinMode(B, INPUT_PULLUP);

  attachInterrupt(0, increment, CHANGE);
  Serial.begin(9600);
}

void increment() {
  if (digitalRead(A) == HIGH) {
    if (digitalRead(B) == LOW) {
      count++;
    } else if (digitalRead(B) == HIGH) {
      count--;
    }
  }
  /*
  // Uncomment this for 800 Pulse mode
  if (digitalRead(A) == LOW) {
    if (digitalRead(B) == HIGH) {
      count++;
    } else if (digitalRead(B) == LOW) {
      count--;
    }
  }
  */
}

void loop() {
  Serial.println(count);

}
