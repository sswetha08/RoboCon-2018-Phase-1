//LSA
int enableLSA1 = 22;
int enableLSA2 = 24;
int enableLSA3 = 26;
int valLSA1 = 0;
int valLSA2 = 0;
int valLSA3 = 0;
HardwareSerial *lsaSerial = NULL;

void setLsaSerial (HardwareSerial *serial); //so that serial has to be changed only once

void setup() {

  setLsaSerial(&Serial3);
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
  digitalWrite(enableLSA1, HIGH);
  delayMicroseconds(100);
  while (!(lsaSerial->available()));
  while (lsaSerial->available())
  {
  valLSA1 = lsaSerial->read();
  }
  digitalWrite(enableLSA1, LOW);

  //delayMicroseconds(100);

  /* digitalWrite(enableLSA2, HIGH);
    delayMicroseconds(100);
    while (!(lsaSerial->available()));
    while (lsaSerial->available())
    {
      valLSA2 = lsaSerial->read();
    }
    digitalWrite(enableLSA2, LOW);
    clearLSA();
    delayMicroseconds(100);

     digitalWrite(enableLSA3, HIGH);
    delayMicroseconds(100);
    while (!(lsaSerial->available()));
    while (lsaSerial->available())
    {
     valLSA3 = lsaSerial->read();
    }
    digitalWrite(enableLSA3, LOW);
    clearLSA();
    delayMicroseconds(100);
  */


  Serial.print(valLSA1);
  Serial.println();
  /* Serial.print(',');
    Serial.print(valLSA2);
    Serial.print(',');
    Serial.println(valLSA3);
  */
}
void setLsaSerial (HardwareSerial *serial)
{
  lsaSerial = serial;
}
