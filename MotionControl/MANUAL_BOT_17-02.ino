
// Manual bot - 2018
// 17-02-18

#include <Cytron_PS2Shield.h>
#include <Servo.h>

Cytron_PS2Shield PS;
Servo ser[5];


//Motor pins
int pwmPins[3] = {4, 5, 3};
int dirPins[3] = {28, 30, 34};
float mag[3] = {0, 0, 0};

//Servo

//int servoPins[] = {53,49,45,41,37};
//int ledPins[] = {51,47,43,39,35};
int closePos[] = {85, 80, 90, 87, 80};
int openPos[] = {145, 145, 145, 135, 130};
int servoPins[] = {41, 37, 33, 29, 25}; // LEFT TO RIGHT
int ledPins[] = {39, 35, 31, 27, 23};   // LEFT TO RIGHT

int select = 0;
int selectLight = 0;

//PS2 Button states
int END = 1;
int endStart = 0;
int rightButtonState = 1;
int leftButtonState = 1;
int currentFR = 1;
int currentFL = 1;
int prevRight = 1;
int prevLeft = 1;
int prevFR = 1;
int prevFL = 1;
int prevOpen = 1;
int openGripper = 1;
int closeGripper = 1;
int prevClose = 1;
int gripRight = 1;
int gripLeft = 1;
int prevGR = 1;
int prevGL = 1;
int countLt = 0;
int countRt = 0;
int gripCount = 0; // Set by Left and Right control
int controlR = 0;

//PS2 Joystick
float RightX = 0; // RIGHT JOYSTICK FOR LOWER SPEED
float RightY = 0;
float LeftX = 0; // LEFT JOYSTICK FOR HIGH SPEED
float LeftY = 0;
float speeds = 0;
float cap = 0.8;  // SPEED CAP (127*cap) (LEFT JOYSTICK)
float angle = 0;
//float angle =((a*3.14)/180);

//PID
int Error = 0;
int prevError = 0;
double kd = 1.5;
double kp = 2.1;
int pid_mag = 0;
int pid_cap = 40;

//IMU

int currentOrient = 0;
int prevAngle = 0;
int idealOrient = 0;
int initialOrient = 0;
byte input[2];
int x;
HardwareSerial *imuSerial = NULL;

//Loop time check
int time1 = 0, oldtime = 0; 

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
  // Serial.println("Speeds");
  // Serial.println(speeds);
  mag[0] =  (int)(speeds * cap) * cos(0.5236 - angle); 
  mag[1] =  (int)(speeds * cap) * cos(2.6180 - angle); 
  mag[2] =  (int)(speeds * cap) * cos(4.7124 - angle); 

}

int getYawVal()
{
  imuSerial->write("#f");
  while (imuSerial->available())
  {
    imuSerial->readBytes(input, 2);
    x = (input[0] << 8 | input[1]);
    /*Serial.print("IMU  : ");
      Serial.println(val1); */
  }

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

  Serial3.begin(57600);
  Serial.begin(9600);
  
  setImuSerial(&Serial3);
  delay(4000); 
  
  int junk = getYawVal();
  Serial.println("junk: ");
  Serial.println(junk);
  
  junk = getYawVal();
  Serial.println("junk: ");
  Serial.println(junk);
  
  for (int i = 0; i < 5; i++)
  {
    ser[i].attach(servoPins[i]);
    ser[i].write(openPos[i]);
  }

  for (int i = 0; i < 5; i++)
  {
    pinMode(ledPins[i], OUTPUT);
    digitalWrite(ledPins[i], LOW);
  }

  digitalWrite(ledPins[0], HIGH); // First LED is ON as first gripper is selected by default
  
  PS.begin(115200);
 
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
  /* Serial.println("Motor speeds");
    Serial.print(mag[0]);
    Serial.print(',');
    Serial.print(mag[1]);
    Serial.println(',');
    Serial.println(mag[2]);
  */
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
  time1 = millis();
  //Serial.println(time1 - oldtime);
  //Serial.println(END);

  readButtonStates();
  if (END != 1)
  {
    readPS2Stick();
    addTranslate();
    pid();
    moveMotors();
  }

  oldtime = time1;

}


void readPS2Stick()
{
 
  LeftX = 255 - PS.readButton(PS2_JOYSTICK_LEFT_X_AXIS);
  LeftY = PS.readButton(PS2_JOYSTICK_LEFT_Y_AXIS);
  RightX = 255 - PS.readButton(PS2_JOYSTICK_RIGHT_X_AXIS);
  RightY = PS.readButton(PS2_JOYSTICK_RIGHT_Y_AXIS);
  
  //Serial.println("Coordinates: ");
  //Serial.print(LeftX);
  //Serial.print(',');
  //Serial.println(LeftY);
  
  adjust();
  calcSpeed();
 

}
void readButtonStates()
{
  rightButtonState = PS.readButton(PS2_RIGHT_1);
  leftButtonState = PS.readButton(PS2_LEFT_1);
  currentFR = PS.readButton(PS2_RIGHT_2);
  currentFL = PS.readButton(PS2_LEFT_2);
  openGripper = PS.readButton(PS2_CIRCLE);
  closeGripper = PS.readButton(PS2_SQUARE);
  gripRight = PS.readButton(PS2_RIGHT);
  gripLeft =  PS.readButton(PS2_LEFT);

  if (PS.readButton(PS2_CROSS) == 0)
  {
    END = 1;
    clearMag();
    moveMotors();
  }
  else if (PS.readButton(PS2_START) == 0)
  {
    END = 0;
  }
  
  else if ((gripRight == 0) && (prevGR == 1))
  {
    gripCount++;
    for (int i = 0; i < 5; i++)
      digitalWrite(ledPins[i], LOW);
    selectLight = gripCount % 5;
    digitalWrite(ledPins[selectLight], HIGH);

  }

  else if ((gripLeft == 0) && (prevGL == 1))
  {
    gripCount--;
    for (int i = 0; i < 5; i++)
      digitalWrite(ledPins[i], LOW);
    selectLight = gripCount % 5;
    digitalWrite(ledPins[selectLight], HIGH);

  }

  else if (((openGripper == 0) && (prevOpen == 1)))

  {
    select = gripCount % 5;
    ser[select].write(openPos[select]);
    //Serial.println(select);
  }

  else if ((closeGripper == 0) && (prevClose == 1))
  {
    for (int i = 0; i < 5; i++)
      ser[i].write(closePos[i]);

  }


  else if (((rightButtonState == 0) && (prevRight == 0)) || ((rightButtonState == 0) && (prevRight == 1)))
  {
    idealOrient -= 3;
  }

  else if (((currentFR == 0) && (prevFR == 0)) || ((currentFR == 0) && (prevFR == 1)))
  {
    countRt++;
    if (countRt == 20)
    {
      idealOrient -= 5;
      countRt = 0;
    }

  }

  else if (((currentFL == 0) && (prevFL == 0)) || ((currentFL == 0) && (prevFL == 1)))
  {
    countLt++;
    if (countLt == 20)
    {
      idealOrient += 5;
      countLt = 0;
    }
  }

  else if (((leftButtonState == 0) && (prevLeft == 0)) || ((leftButtonState == 0) && (prevLeft == 1)))
  {
    idealOrient += 3;
  }



  prevRight = rightButtonState;
  prevLeft = leftButtonState;
  prevGR = gripRight;
  prevGL = gripLeft;
  prevFL = currentFL;
  prevFR = currentFR;
  prevOpen = openGripper;
  prevClose = closeGripper;
 

}
void setAngle()
{
  float temp = (float)LeftY / LeftX;
  LeftY = -LeftY;
  LeftX = -LeftX;
  
  RightY = -RightY;
  RightX = -RightX;
  
  if (controlR == 1)
    angle = atan2(RightX, RightY);
  else
    angle = atan2(LeftX, LeftY); 
    
  if (angle < 0)
    angle = 3.14 + (3.14 + angle);
    
  //angle = atan(temp);
  //angle+=3.14;
  //angle =  angle * 180 / 3.14;
  //Serial.print("Angle: ");
  //Serial.println(angle);  // Value is in range -180 - 180 */


}

void adjust() // Makes the coordinates from -128 to 128
{
  LeftX = LeftX - 127;
  RightX = RightX - 127;
   // Serial.print("Adjusted X: ");
  // Serial.println(LeftX);
  
  RightY = -(RightY - 127);
  LeftY = -(LeftY - 127); //  value is zero at upper end
  //Serial.print("Adjusted Y: ");
  //Serial.println(LeftY);
}

void calcSpeed()
{
  //squareCirc();
  speeds = (abs(LeftX) > abs(LeftY)) ? abs(LeftX) : abs(LeftY);
  if (speeds == 0)
  {
    controlR = 1;
    speeds =  0.3 * ((abs(RightX) > abs(RightY)) ? abs(RightX) : abs(RightY));
  }
  else
  {
    controlR = 0;
  }
  setAngle();
  //speeds = pow(pow(LeftX, 2) + pow(LeftY, 2), 0.5);
  //Serial.print("Speed: ");
  // Serial.println(speeds);
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
  //Serial.print("Error in pid");
  //Serial.println(Error);
  
  pid_mag = Error * kp + (Error - prevError) * kd;
  
  if (abs(pid_mag) > pid_cap)
  {
    if (pid_mag > 0)
      pid_mag = pid_cap;
    else
      pid_mag = -pid_cap;
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
  int angle = getYawVal();
  int delta = 0;
  angle = 180 - angle;
  delta = angle - prevAngle;
  //   Serial.print("Delta: ");
  // Serial.println(delta);
  if (angle < 10 && prevAngle > 350)
  {
    delta = angle + abs(360 - prevAngle);
  }
  else if (angle > 350 && prevAngle < 10)
  {
    delta = -1 * (abs(360 - angle) + abs(prevAngle));
  }

  currentOrient = currentOrient + delta;
  prevAngle = angle;

  /*Serial.print("Current Orient: ");
    Serial.println(currentOrient);
    Serial.print("ideal Orient: ");
    Serial.println(idealOrient);
    return currentOrient;
  */
}
