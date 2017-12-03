
//Motors
int pwmPins[3] = {6, 3, 4 };
int dirPins[3] = {7, 2, 5};
int mag[3] = {0};
double magController = 0;
double angController = 0;
double overshootControlScale  = 0.7;

void setup() {
  for (int i = 0; i < 3; i++)
  {
    pinMode(pwmPins[i], OUTPUT);
    pinMode(dirPins[i], OUTPUT);
  }
}

void loop()
{

  magController = 100;
  angController = 180 * (PI / 180.0);
  clearMag();
  
  /*addTranslate();
  mag[0] = 100;
  mag[1] = 100;
  mag[2] = 100;*/
  
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

void addTranslate()
{
  mag[0] += magController * cos(2.6180 - angController);
  mag[1] += magController * cos(0.5236 - angController);
  mag[2] += magController * cos(4.7124 - angController);
}

void moveMotors()
{
  int i;
  for (i = 0; i < 3 ; i++)
  {
    Serial.println(mag[i]);
    analogWrite(  pwmPins[i], abs(mag[i]));
    digitalWrite(dirPins[i], (mag[i] > 0) ? HIGH : LOW); //change this for axis change
  }
}

