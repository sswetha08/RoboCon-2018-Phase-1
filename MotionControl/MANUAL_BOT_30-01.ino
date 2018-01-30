#include <Cytron_PS2Shield.h>
Cytron_PS2Shield PS; //(12,11)

int pwmPins[3] = {2, 4, 5};
int dirPins[3] = {32, 28, 30};
int mag[3] = {0, 0, 0};
int END =0;
int rightButtonState=1;
int leftButtonState=1;
int currentFR=1;
int currentFL=1;
int prevRight=1;
int prevLeft=1;
int prevFR=1;
int prevFL=1;
int countLt=0;
int countRt=0;
//PS2
float RightX = 0;
float RightY = 0;
float LeftX = 0;
float LeftY = 0;
int speeds = 0;
float angle = 0;
//float angle =((a*3.14)/180);

//PID
int Error = 0;
int prevError = 0;
double kd = 1.5;
double kp = 2.1;
int pid_mag = 0;

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
  // Serial.println("Speeds");
  // Serial.println(speeds);
  mag[0] = (speeds) * cos(4.7124 - angle); //(150) 0.5236
  mag[1] = (speeds) * cos(0.5236 - angle); //(30)
  mag[2] = (speeds) * cos(2.6180 - angle); // (270) cos 270 = 0

}

int getYaw()
{
  Serial3.write("#f");
  byte input[2]; int x;
  while (Serial3.available())
  {
    Serial3.readBytes(input, 2);
    x = (input[0] << 8 | input[1]);
    /*Serial.print("Real : ");
      Serial.println(val1); */
  }
  //if(x<0)
  //x= 180+(180+x); // 0 to 360
  //Serial.println(x);
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
  Serial3.begin(57600);
  Serial.begin(9600);
  Serial3.flush();
  setImuSerial(&Serial3); // Check

  //clearIMU();
  //clearIMU();
  int junk = getYaw();
  //junk = getYaw();

  Serial.println("junk: ");
  Serial.println(junk);
  junk = getYaw();

  Serial.println("junk: ");
  Serial.println(junk);
  delay(2000);
  initialOrient = getCurrentOrient();
  currentOrient = initialOrient;
  idealOrient = initialOrient;


  Serial.println("Initial: ");
  Serial.println(initialOrient);
  Serial.println("Current: ");
  Serial.println(currentOrient);
  Serial.println("Ideal: ");
  Serial.println(idealOrient);


}
void setImuSerial (HardwareSerial *serial)
{
  imuSerial = serial;
}
void moveMotors()
{
  int i;
  /*Serial.println("Motor speeds");
    Serial.print(mag[0]);
    Serial.print(',');
      Serial.print(mag[1]);
    Serial.println(',');
    Serial.println(mag[2]);*/
  for (i = 0; i < 3; i++)
  {
    analogWrite(pwmPins[i], abs(mag[i]));
    digitalWrite(dirPins[i], (mag[i] > 0) ? LOW : HIGH); // Anticlockwise
  }
}

void clearMag()
{
  for (int i = 0; i < 3; i++)
    mag[i] = 0;
}

void loop()
{
  rightButtonState = PS.readButton(PS2_RIGHT_1);
  leftButtonState = PS.readButton(PS2_LEFT_1);
  currentFR = PS.readButton(PS2_RIGHT_2);
  currentFL = PS.readButton(PS2_LEFT_2);

  if(PS.readButton(PS2_CROSS)==0)
  {
    END=1;
    clearMag();
  }
  
  else if (((rightButtonState == 0) && (prevRight == 0)) || ((rightButtonState == 0) && (prevRight == 1)))
  {
    
    idealOrient -= 5;
    
  }

   else if (((currentFR == 0) && (prevFR == 0)) || ((currentFR == 0) && (prevFR == 1)))
  {
      countRt++;
    if(countRt==20)
    {
      idealOrient-=5;
       countRt=0;
    }

  }

   else if (((currentFL == 0) && (prevFL == 0)) || ((currentFL == 0) && (prevFL == 1)))
  {
    countLt++;
    if(countLt==20)
    {
      idealOrient+=5;
      countLt=0; 
    }
  }
  
 else if (((leftButtonState == 0) && (prevLeft == 0)) || ((leftButtonState == 0) && (prevLeft == 1)))
  {
    
    idealOrient += 5;
    
  }
  if(END!=1)
  {
  readPS2();
  addTranslate();
  pid();
  moveMotors();
  prevRight = rightButtonState;
  prevLeft = leftButtonState;
  prevFL = currentFL;
  prevFR = currentFR;
  }

}


void readPS2()
{
  // add rotate part
  LeftX = 255 - PS.readButton(PS2_JOYSTICK_LEFT_X_AXIS);
  LeftY = PS.readButton(PS2_JOYSTICK_LEFT_Y_AXIS);

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
  LeftY = -LeftY;
  LeftX = -LeftX;
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
  LeftX = LeftX - 127;
  // Serial.print("Adjusted X: ");
  //   Serial.println(LeftX);
  LeftY = -(LeftY - 127); //  value is zero at upper end
  // Serial.print("Adjusted Y: ");
  //Serial.println(LeftY);
}

void calcSpeed()
{
  //squareCirc();
  speeds = (abs(LeftX) > abs(LeftY)) ? abs(LeftX) : abs(LeftY);

  //speeds = pow(pow(LeftX, 2) + pow(LeftY, 2), 0.5);
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
  // Serial.print("CO in pid");
  // Serial.println(currentOrient);
  Error = idealOrient - currentOrient;
  //Serial.print("Error in pid");
  //Serial.println(Error);
  pid_mag = Error * kp + (Error - prevError) * kd;
  if (abs(pid_mag) > 40)
  {
    if (pid_mag > 0)
      pid_mag = 40;
    else
      pid_mag = -40;
  }
  // Serial.print("Magnitude of pid");
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
  Serial.print("Current Orient: ");
  Serial.println(currentOrient);
  Serial.print("ideal Orient: ");
  Serial.println(idealOrient);
  return currentOrient;
}
