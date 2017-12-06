#include<Cytron_PS2Shield.h>
Cytron_PS2Shield PS(50,52); //(12,11)

int pwmPins[3] = {4, 3, 6};
int dirPins[3] = {5, 2, 7};
int mag[3] = {0, 0, 0};


//PS2
float RightX = 0;
float RightY = 0;
float LeftX = 0;
float LeftY = 0;
int speeds = 100;
float angle = 0;
//float angle =((a*3.14)/180);

//PID
int Error = 0;
int prevError = 0;
double kd = 1;
double kp = 1.8;
int pid_mag;

//IMU
int readByte = 0;
unsigned char data[2] = {0};
int currentOrient = 0;
int prevAngle = 0;
int idealOrient = 0;
int initialOrient = 0;
HardwareSerial *imuSerial = NULL;

void clearIMU()
{
  imuSerial->write("#f");
  char junk;
  while (imuSerial->available())
  {
    junk = imuSerial->read();
  }
}
void addTranslate()
{
  // At zero degrees, moves in direction of motor 1 (motor 1 does not move)
  // 120 , for motor 2
  // 240 , for motor 3

  mag[0] = (speeds/2) * cos(0.5236 - angle); //(150) 
  mag[1] = (speeds/2) * cos(4.7124 - angle); //(30) 
  mag[2] = (speeds/2) * cos(2.6180 - angle); // (270) cos 270 = 0
}

int getYaw()
{
  Serial2.write("#f");
  byte input[2]; int x;
  while (Serial2.available())
  {
    Serial2.readBytes(input, 2);
    x = (input[0] << 8 | input[1]);
    /*Serial.print("Real : ");
      Serial.println(val1); */
  }
  //if(x<0)
  //x= 180+(180+x); // 0 to 360
  Serial.println(x);
  delay(40);
  return x;
}
void setup()
{
  int i;
  for (i = 0; i < 3; i++)
  {
    pinMode(pwmPins[i], OUTPUT);
    pinMode(dirPins[i], OUTPUT);
  }
  PS.begin(57600);
  Serial2.begin(57600);
  Serial.begin(9600);
  Serial2.flush();
  setImuSerial(&Serial2); // Check

  clearIMU();
  clearIMU();
  int junk = getYaw();
  delay(200);
  initialOrient = getCurrentOrient();
  currentOrient = initialOrient;
  idealOrient = initialOrient;


  //Serial.println("Initial: ");
  // Serial.println(initialOrient);


}
void setImuSerial (HardwareSerial *serial)
{
  imuSerial = serial;
}
void moveMotors()
{
  int i;
  for (i = 0; i < 3; i++)
  {
    analogWrite(pwmPins[i], abs(mag[i]));
    digitalWrite(dirPins[i], (mag[i] > 0) ? LOW: HIGH); // Anticlockwise
  }
}

void clearMag()
{
  for (int i = 0; i < 3; i++)
    mag[i] = 0;
}

void loop()
{
  /*if(millis() >= 50000)
    {clearMag();} */
  readPS2();
  addTranslate();
  pid();
  moveMotors();
  delay(40);

}


void readPS2()
{
  // add rotate part
  LeftX = 255 - PS.readButton(PS2_JOYSTICK_RIGHT_X_AXIS);
  LeftY = PS.readButton(PS2_JOYSTICK_RIGHT_Y_AXIS);

  adjust();

  /*Serial.println("Coordinates: ");
    Serial.print(LeftX);
    Serial.print(',');
    Serial.println(LeftY);*/
  setValues();
  //delay(100);

}
void setValues()
{
  float temp = (float)LeftY / LeftX;
  //Serial.print("Ratio: ");
  // Serial.print(temp);
  // Serial.print(',');

  angle = atan2(LeftX, LeftY); // **later change idealOrient to angle
  if (angle < 0)
    angle = 3.14 + (3.14 + angle);
  //angle = atan(temp);
  //angle+=3.14;
  //angle =  angle * 180 / 3.14;
  //Serial.print("Angle: ");
  //Serial.println(angle);  // Value is in range -180 - 180 */
  calcSpeed();

}

void adjust() // Makes the coordinates from -128 to 128
{
  LeftX = LeftX - 128;
  /*Serial.print("Adjusted X: ");
    Serial.println(LeftX);*/
  LeftY = -(LeftY - 127); //  value is zero at upper end
  //Serial.print("Adjusted Y: ");
  //Serial.println(LeftY);
}

void calcSpeed()
{
  //squareCirc();
 // speeds = (abs(LeftX) > abs(LeftY)) ? abs(LeftX) : abs(LeftY);

  speeds = pow(pow(LeftX, 2) + pow(LeftY, 2), 0.5);
  /*Serial.print("Speed: ");
    Serial.println(speeds);*/
}

void squareCirc()
{
  int ul = (abs(LeftX) > abs(LeftY)) ? abs(LeftX) : abs(LeftY); //setting upperlimit (max radius)
  if (ul != 0)
  { // Bring the coordinates in range of -1 to 1
    LeftX = map(LeftX, -ul, ul, -1, 1);
    LeftY = map(LeftY, -ul, ul, -1, 1);
  }

  /*Serial.println("Coord: ");
    Serial.print(LeftX);
    Serial.print(',');
    Serial.println(LeftY);  */

  map2(); // Conv
  Serial.print("Circular X: ");
  Serial.println(LeftX);
  Serial.print("Circular Y: ");
  Serial.println(LeftY);

  LeftX = map(LeftX, -1, 1, -ul, ul);
  LeftY = map(LeftY, -1, 1, -ul, ul);

}

void map2()
{
  if (LeftX * LeftY < 0)
  {
    LeftX = LeftX * sqrt((1 - (-0.5 * LeftY * LeftY)));
    LeftY = LeftY * sqrt((1 - (-0.5 * LeftX * LeftX)));

  }
  else
  {
    LeftX = LeftX * sqrt((1 - (0.5 * LeftY * LeftY)));
    LeftY = LeftY * sqrt((1 - (0.5 * LeftX * LeftX)));
  }

}

void pid()
{
  currentOrient = getCurrentOrient();
  Error = idealOrient - currentOrient;
  //Serial.println(Error);
  pid_mag = Error * kp + (Error - prevError) * kd;
  if(abs(pid_mag)>60)
  {
    if(pid_mag>0)
    pid_mag=60;
    else
    pid_mag=-60;
  }
  //Serial.println(pid_mag);
  for (int i = 0; i < 3; i++)
  {
    mag[i] -= pid_mag;
    
  }
  /*Serial.print(mag[0]);
  Serial.print(',');
   Serial.print(mag[1]);
  Serial.print(',');
   Serial.println(mag[2]); */
 
  
  prevError = Error;
}


int getCurrentOrient()
{
  int angle = getYaw();
  int delta = 0;
  angle = 180 - angle;
  delta = angle - prevAngle;
  if (angle < 5 && prevAngle > 355)
  {
    delta = angle + abs(360 - prevAngle);
  }
  else if (angle > 355 && prevAngle < 5)
  {
    delta = -1 * (abs(360 - angle) + abs(prevAngle));
  }
  currentOrient = currentOrient + delta;
  prevAngle = angle;
  //Serial.println(currentOrient);
  return currentOrient;
}
