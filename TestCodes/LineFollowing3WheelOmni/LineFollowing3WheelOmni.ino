//LSA
int enableLSA1 = 22;
int enableLSA2 = 24;
int enableLSA3 = 28;
int valLSA1 = 0;
int valLSA2 = 0;
int valLSA3 = 0;
HardwareSerial *lsaSerial = NULL;

//Motors
int pwmPins[3] = {6, 3, 4 };
int dirPins[3] = {7, 2, 5};
int mag[3] = {0};
double magController = 0;
double angController = 0;
double overshootControlScale  = 0.7;

//PD
double kp = 1.2;
double kd = -0.2;
int a = 2;
int scale = 1.5;
int prevError;
int pd = 0;
int val = 0;
int error=0;

void setLsaSerial (HardwareSerial *serial); //so that serial has to be changed only once

void setup() {
  for (int i = 0; i < 3; i++)
  {
    pinMode(pwmPins[i], OUTPUT);
    pinMode(dirPins[i], OUTPUT);
  }
  setLsaSerial(&Serial3);
  lsaSerial->begin(38400);
  Serial.begin(57600);
  pinMode(enableLSA1, OUTPUT);
  pinMode(enableLSA2, OUTPUT);
  pinMode(enableLSA3, OUTPUT);

  digitalWrite(enableLSA1, LOW);
  digitalWrite(enableLSA2, LOW);
  digitalWrite(enableLSA3, LOW);
}

void loop()
{
 // readLSA();
  magController = 100;
  angController = 180 * (PI / 180.0);
  clearMag();
  addTranslate();
 // add_PD();
 // mag[0] = 100;
// mag[1] = 100;
// mag[2] = 100;
  moveMotors();
}

void clearMag()
{
  int i;
  for (i = 0; i < 3; i++)
  {
    mag[i] = 0;
  }
}



void add_PD()
{
 
  if (angController == 0)
    val = valLSA1;
  if (angController == 180 * (PI / 180.0))
    val = valLSA2;
  if (angController == 90 * (PI / 180.0))
    val = valLSA3;
  error = (35 - val);

  pd = (error * kp) + (error - prevError) * kd ;
  Serial.print("error : ");
  Serial.print(error);
  Serial.print(" , pd : ");
  Serial.println(pd);
  //Serial.println(kd);

  //pd = constrain(pd, -40, 40);



  for ( int i = 0; i < 3; i++)
  {
    mag[i] = mag[i] + (int)pd;
  }

  /* if (abs(mag[0]) + abs(mag[1]) + abs(mag[2]) > 100)
    {
     Serial.println("Magnitude exceeded abs sum = 100");
     mag[0] = overshootControlScale * mag[0];
     mag[1] = overshootControlScale * mag[1];
     mag[2] = overshootControlScale * mag[2];
    }*/
  prevError = error;
}


void addTranslate()
{
  mag[0] += magController * cos(2.6180 - angController);
  mag[1] += magController * cos(0.5236 - angController);
  mag[2] += magController * cos(4.7124 - angController);
}

void clearLSA()
{
  char useless;
  while (lsaSerial->available())
  {
    useless = lsaSerial->read();
  }
}

//@AM
bool killAllMotors = false;

void moveMotors()
{
  int i;
  for (i = 0; i < 3 ; i++)
  {

    /* if ((angController == 0 && valLSA1 > 70) || (angController == 180 * (PI / 180.0) && valLSA2 > 70) || (angController == 90 * (PI / 180.0) && valLSA3 > 70));
      {
       analogWrite(pwmPins[0], 0);
       analogWrite(pwmPins[1], 0);
       analogWrite(pwmPins[2], 0);
       Serial.println("LOL");
       break;

      }*/
    Serial.println(mag[i]);
     analogWrite(killAllMotors ? 0 :  pwmPins[i], abs(mag[i]));
   // analogWrite(  pwmPins[i], abs(mag[i]));
    digitalWrite(dirPins[i], (mag[i] >0) ? HIGH : LOW); //change this for axis change
  }
}


void readLSA()
{
  //Serial.println("values");
  clearLSA();
  digitalWrite(enableLSA1, HIGH);
  delayMicroseconds(100);
  while (!(lsaSerial->available()));
  while (lsaSerial->available())
  {
    valLSA1 = lsaSerial->read();
  }
  digitalWrite(enableLSA1, LOW);
  clearLSA();
  delayMicroseconds(100);

  digitalWrite(enableLSA2, HIGH);
  delayMicroseconds(100);
  while (!(lsaSerial->available()));
  while (lsaSerial->available())
  {
    valLSA2 = lsaSerial->read();
  }
  digitalWrite(enableLSA2, LOW);
  clearLSA();
  delayMicroseconds(100);

  if (valLSA2 > 70 && (angController == 180 * (PI / 180.0))) {
    killAllMotors = true;
  } else {
    killAllMotors = false;
  }

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
  if (valLSA3 > 70 && (angController == 90 * (PI / 180.0))) {
    killAllMotors = true;
  } else {
    killAllMotors = false;
  }

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
