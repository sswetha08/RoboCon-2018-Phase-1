
//Motors
int pwmPins[3] = {7, 5, 6 };
int dirPins[3] = {49, 43, 45};
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

  magController = 70;
  angController = 0 * (PI / 180.0);
  clearMag();
  addTranslate();
    
 // mag[0] = 50;
  //mag[1] = 50;
  //mag[2] = 50;
  
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

