void setup() {
  // Begin serial monitor connection
  Serial1.begin(9600);
}

void loop() {
  // Toggle flap every second
  Serial1.write('f');
  delay(1000);
  Serial1.write('g');
  delay(1000);
}
