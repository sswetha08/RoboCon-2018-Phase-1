
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial3.begin(9600);


}
// put your setup code here, to run once:

void loop() {
  // put your main code here, to run repeatedly:

  if (Serial3.available()) {
    Serial.print("Value1 obtained from motion : ");
  //  int value1 =  Serial3.parseInt();
    Serial.println((char)Serial3.read());
  }
  if (Serial.available()) {
    byte valsend = Serial.read();
    Serial3.write(valsend);
  }
}


