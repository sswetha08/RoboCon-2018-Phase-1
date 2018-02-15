int LSA_Pin1 = 13;
int valLSA1 = 0;
int LSA_Pin2 =15;
int valLSA2 = 0;
//Motors
int pwmPins[3] = {7, 5, 6};
int dirPins[3] = {49, 43, 45};
int mag[3] = {};
double angularDirection = 0 * (PI / 180.0);

//PID
double Kp = 0.6, Kd = 0.12; //0.5 0.12
double KpF = 1.8, KdF = 0.6;  //1 0.8
double KpB = 1.5, KdB = 0.4; // 0.7 0.5
double previousError = 0;
double previousErrorFront = 0;
double previousErrorBack = 0;
float PIDCorrectionBack = 0;


int state = 3;

void setup() {
  for (int i = 0; i < 3; i++)
  {
    pinMode(pwmPins[i], OUTPUT);
    pinMode(dirPins[i], OUTPUT);
  }

  Serial.begin(9600);


}

void loop() {
  readLSA();
  /* if (valLSA1 != 255)
    {
     mag[0] = 0;
     mag[1] = 0;
     mag[2] = 0;
     moveMotors();
     readLSA();
    }
    if(valLSA1 == 255)
    {
     mag[0] = 50;
     mag[1] = 50;
     mag[2] = 50;
     moveMotors();
     readLSA();
     //delay(100);
    }*/
  //readLSA();

  if (state == 3)
  {
    while (state == 3)
    {
      state3();
    }
  }

   

}

void readLSA()
{
  valLSA1 = analogRead(LSA_Pin1);
  if (valLSA1 <= 900)
  {
    valLSA1 = map(valLSA1, 0, 900, 0, 70);
  }
  else
    valLSA1 = 255;
   
  valLSA2=analogRead(LSA_Pin2);
  if(valLSA2<=900)
  {
    valLSA2 = map(valLSA2,0,900,0,70);
  }
  else
  valLSA2=255;

}

void motorEquations(double angle, double magController) {
  mag[0] = magController * cos(150 * PI / 180.0 - angle);
  mag[1] = magController * cos(30 * PI / 180.0 - angle);
  mag[2] = magController * cos(270 * PI / 180.0 - angle);
}

void moveMotors() {
  for (int i = 0; i < 3; i++) {
    analogWrite(pwmPins[i], abs(mag[i]));
    digitalWrite(dirPins[i], (mag[i] > 0) ? HIGH : LOW);
    // Serial.print(mag[i] + " ");
  }
  //Serial.println();
}

void state3() //rotation at J1
{
  readLSA();
  while (valLSA1 >= 0 && valLSA1 <= 70)
  {
    mag[0] = -30;
    mag[1] = -30;
    mag[2] = -30;
    moveMotors();
    readLSA();
  }
  while (valLSA1 == 255)
  {
    mag[0] = -30;
    mag[1] = -30;
    mag[2] = -30;
    moveMotors();
    readLSA();
    delay(100);
  }
  mag[0] = 0;
  mag[1] = 0;
  mag[2] = 0;
  moveMotors();
  motorEquations(0 * PI / 180.0, 70);
  addPID_2();
  moveMotors();
  delay(1055);
  mag[0] = 0;
  mag[1] = 0;
  mag[2] = 0;
  moveMotors();
  state++;
  
}

void state4() //rotation at TZJ1
{
  readLSA();
  while (valLSA2 >= 0 && valLSA2 <= 70)
  {
    mag[0] = 30;
    mag[1] = 30;
    mag[2] = 30;
    moveMotors();
    readLSA();
  }
  while (valLSA2 == 255)
  {
    mag[0] = 30;
    mag[1] = 30;
    mag[2] = 30;
    moveMotors();
    readLSA();
    delay(100);
  }
  mag[0] = 0;
  mag[1] = 0;
  mag[2] = 0;
  moveMotors();
   state++;
  
}

void addPID_2() {
  int valueFront, valueBack;
  if (angularDirection == 0 * PI / 180.0)
  {
    if(valLSA1!=255)
    valueFront = valLSA1;
    if (valLSA2!=255)
      valueBack = valLSA2;
  }
  else if (angularDirection == 180 * PI / 180.0) {
   if(valLSA1!=255)
    valueFront = valLSA2;
    if (valLSA2!=255)
      valueBack = valLSA1;
  }
  int errorFront = (35 - valueFront);
  int errorBack = (35 - valueBack);


  float PIDCorrectionFront = KpF * errorFront - KdF * (errorFront - previousErrorFront);
 
    PIDCorrectionBack = KpB * errorBack - KdB * (errorBack - previousErrorBack);
    mag[1] += PIDCorrectionBack;
    mag[0] += PIDCorrectionBack;
    mag[2] += PIDCorrectionFront;


  if (valLSA1 == 255)
  {
   // mag[2] = -0.5 * mag[2];
  //   mag[0] = 0;
   //  mag[1] = 0;
  //   mag[2] = 0;
     mag[0] = -0.5 * mag[0];
    mag[1] = -0.5 * mag[1];
    mag[2] = -0.5 * mag[2];
  }
  if (valueBack == 255)
  {
    //mag[1] = -0.5 * mag[1];
    // mag[0] = -0.5 * mag[2];
    // mag[2] = -0.5 * mag[2];
    //mag[1]=0;
    //  mag[2]=0;
    //mag[0]=0;
  }


  previousErrorFront = errorFront;
  previousErrorBack = errorBack;
}
