//LSA
int enableLSA1 = 15;
int enableLSA2 = 14;
int enableLSA3 = 13;
float valLSA1 = 0;
float valLSA2 = 0;
float valLSA3 = 0;
HardwareSerial *lsaSerial = NULL;

void setLsaSerial (HardwareSerial *serial); //so that serial has to be changed only once

void setup() {

  setLsaSerial(&Serial2);
  lsaSerial->begin(38400);
  Serial.begin(38400);
  pinMode(enableLSA1, OUTPUT);
  pinMode(enableLSA2, OUTPUT);
  pinMode(enableLSA3, OUTPUT);

  digitalWrite(enableLSA1, LOW);
  digitalWrite(enableLSA2, LOW);
  digitalWrite(enableLSA3, LOW);
}

void loop()
{
  readLSA();
}

void clearLSA()
{
  while (lsaSerial->available())
  {
    lsaSerial->read();
  }
}

void readLSA()
{
  Serial.print("Values : ");
  clearLSA();
  valLSA1 = analogRead(enableLSA1);
  //valLSA1=(valLSA1/1023.0)*70;
  clearLSA();
  delayMicroseconds(1000);

  valLSA2 = analogRead(enableLSA2);
  clearLSA();
  delayMicroseconds(1000);

  valLSA3 = analogRead(enableLSA3);
  clearLSA();
  delayMicroseconds(1000);



  Serial.print(valLSA1);
  Serial.print(',');
  Serial.print(valLSA2);
  Serial.print(',');
  Serial.println(valLSA3);

}
void setLsaSerial (HardwareSerial *serial)
{
  lsaSerial = serial;
}
