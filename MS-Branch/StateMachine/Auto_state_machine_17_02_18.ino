//LSA
int enableLSA1 = 22;
int enableLSA2 = 26;
int enableLSA3 = 24;
int LSA_Pin1 = 13;
int LSA_Pin2 = 15;
int LSA_Pin3 = 14;
int valLSA1 = 0;
int valLSA2 = 0;
int valLSA3 = 0;
HardwareSerial  *LSASerial = NULL;

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

void initialiseLSASerial(HardwareSerial *serialPort);

//State machine
int state =1;
int over = 0;

//Colour sensor
#define r 10
#define g 11
#define b 12
#define S0 48
#define S1 46
#define s2 44
#define s3 42
#define LED 52
#define sensorOut 50
double rfrequency = 0;
double gfrequency = 0;
double bfrequency = 0;
char stat = 'A';
double var_lg, var_dg, var_red;


void setup() {
  for (int i = 0; i < 3; i++)
  {
    pinMode(pwmPins[i], OUTPUT);
    pinMode(dirPins[i], OUTPUT);
  }
  initialiseLSASerial(&Serial2);
  LSASerial->begin(38400);
  Serial.begin(9600);
  pinMode(enableLSA1, OUTPUT);
  pinMode(enableLSA2, OUTPUT);
  pinMode(enableLSA3, OUTPUT);

  digitalWrite(enableLSA1, LOW);
  digitalWrite(enableLSA2, LOW);
  digitalWrite(enableLSA3, LOW);

  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(s2, OUTPUT);
  pinMode(s3, OUTPUT);
  pinMode(r, OUTPUT);
  pinMode(g, OUTPUT);
  pinMode(b, OUTPUT);
  pinMode(sensorOut, INPUT);
  pinMode(LED, OUTPUT);
  digitalWrite(LED, HIGH); // Setting frequency-scaling to 20%
  digitalWrite(S0, HIGH);
  digitalWrite(S1, LOW);
  Serial.begin(9600);
}

void loop() {
  readLSA();
  // motorEquations(0 * PI / 180.0, 170);
  //addPID(); // 0.5 0.12

  //  moveMotors();

  if (state == 1)
  {
    while (state == 1)
    {
      state1();
    }
  }
  over = 0;
  if (state == 2)
  {
    while (state == 2)
    {
      state2();
    }
  }

  if (state == 3)
  {
    while (state == 3)
    {
      state3();
    }
  }
  // state++;   // To skip state4
  if (state == 4)
  {
    while (state == 4)
    {
      state4();
    }
  }
  over = 0;   //variable for overshoot
  if (state == 5)
  {
    while (state == 5)
    {
      state5();
    }
  }
  if (state == 6)
  {
    while (state == 6)
    {
      state6();
    }
  }
  
  if (state == 7)
  {
    while (state == 7)
    {
      state7();
    }
  }
  if (state == 8)
  {
    while (state == 8)
    {
      state8();
    }
  }

  state=0; //stops at the above state (functions below dont get executed)
   if (state == 9)
  {
    while (state == 9)
    {
      state9();
    }
  }
   if (state == 10)
  {
    while (state == 10)
    {
      state10();
    }
  }
  
}

void addPID() {
  int valueFront, valueBack;
  if (angularDirection == 0 * PI / 180.0)
  {
    if (valLSA1 != 255)
      valueFront = valLSA1;
    if (valLSA2 != 255)
      valueBack = valLSA2;
  }
  else if (angularDirection == 180 * PI / 180.0) {
    if (valLSA1 != 255)
      valueFront = valLSA2;
    if (valLSA2 != 255)
      valueBack = valLSA1;
  }
  int errorFront = (35 - valueFront);
  int errorBack = (35 - valueBack);


  float PIDCorrectionFront = KpF * errorFront - KdF * (errorFront - previousErrorFront);

  PIDCorrectionBack = KpB * errorBack - KdB * (errorBack - previousErrorBack);
  mag[1] += PIDCorrectionBack;
  mag[0] += PIDCorrectionBack;
  mag[2] += PIDCorrectionFront;


  if (valLSA1 == 255)   // in case of deviation from the line
  {
    mag[0] = -0.5 * mag[0];
    mag[1] = -0.5 * mag[1];
    mag[2] = -0.5 * mag[2];
  }
  if (valueBack == 255)
  {
  }


  previousErrorFront = errorFront;
  previousErrorBack = errorBack;
}


//LSA
void clearLSA()   //For serial read LSA mode only
{
  while (LSASerial->available())
  {
    LSASerial->read();
  }
}

void readLSA()
{
  Serial.print("Values : ");
  /*                      //Serial Read
   * clearLSA();
    digitalWrite(enableLSA1, HIGH);
    delayMicroseconds(100);
    while (!(LSASerial->available()));
    while (LSASerial->available())
    {
    valLSA1 = LSASerial->read();
    if (valLSA1 == 255) continue;
    else break;
    }
    digitalWrite(enableLSA1, LOW);
    clearLSA();
    delayMicroseconds(100);

    digitalWrite(enableLSA2, HIGH);
    delayMicroseconds(100);
    while (!(LSASerial->available()));
    while (LSASerial->available())
    {
    valLSA2 = LSASerial->read();
    if (valLSA2 == 255) continue;
    else break;
    }
    digitalWrite(enableLSA2, LOW);
    clearLSA();
    delayMicroseconds(100);

    digitalWrite(enableLSA3, HIGH);
    delayMicroseconds(100);
    while (!(LSASerial->available()));
    while (LSASerial->available())
    {
    valLSA3 = LSASerial->read();
    if (valLSA3 == 255) continue;
    else break;
    }
    digitalWrite(enableLSA3, LOW);
    clearLSA();
    delayMicroseconds(100);*/
   //Analog Read
  valLSA1 = analogRead(LSA_Pin1);
  if (valLSA1 <= 900)
  {
    valLSA1 = map(valLSA1, 0, 900, 0, 70); //Mapping Analog values to 0 to 70
  }
  else
    valLSA1 = 255;

  valLSA2 = analogRead(LSA_Pin2);
  if (valLSA2 <= 900)
  {
    valLSA2 = map(valLSA2, 0, 900, 0, 70);
  }
  else
    valLSA2 = 255;

  valLSA3 = analogRead(LSA_Pin3);
  if (valLSA3 <= 900)
  {
    valLSA3 = map(valLSA3, 0, 900, 0, 70);
  }
  else
    valLSA3 = 255;

  Serial.print(valLSA1);
  Serial.print(',');
  Serial.print(valLSA2);
  Serial.print(',');
  Serial.println(valLSA3);
}
void initialiseLSASerial (HardwareSerial *serialPort)
{
  LSASerial = serialPort;
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
  // Serial.println();
}

//RGB sensor
void RGB_Sensor_Out(int s_2, int s_3)
{
  digitalWrite(s2, s_2);
  digitalWrite(s3, s_3);
}

void RGB_Sensor_Response()
{
  double dr, dg, db;
  RGB_Sensor_Out(0, 0);
  rfrequency = pulseIn(sensorOut, LOW);
  RGB_Sensor_Out(1, 1);
  gfrequency = pulseIn(sensorOut, LOW);
  RGB_Sensor_Out(0, 1);
  bfrequency = pulseIn(sensorOut, LOW);
  dr = rfrequency - 101;
  dg = gfrequency - 385;
  db = bfrequency - 326;
  var_red = sqrt(dr * dr + db * db + dg * dg);
}


//State Machine

void state1() //Go to the Manual zone white line
{
  readLSA();
  while (valLSA2 >= 0 && valLSA2 <= 70) // move right to get off the line
  {
    readLSA();
    motorEquations(90 * PI / 180.0, 70);
    // addPID();
    moveMotors();
    //delay(100);
  }
  while (valLSA2 == 255) //moves right till finds another line
  {
    readLSA();
    motorEquations(90 * PI / 180.0, 70);
    // addPID();
    moveMotors();
    //delay(100);
  }
  readLSA();
  motorEquations(180 * PI / 180.0, 70);  //Move forward without PID to get both LSAs on the line
  //addPIDOneLSA()          //If moving forward deviates LSAs from the line ---- alternative, needs to be tested
  moveMotors();
  delay(1000);       // to move forward till boh LSAs on white line
  state++;

 }

void state2() //backwards, stopping at manual zone junction1 for TZ1
{
  readLSA();
  while (valLSA3 == 255 && over == 0) //before the junction
  {
    readLSA();
    motorEquations(180 * PI / 180.0, 170);
    addPID();
    moveMotors();
   }
  if (valLSA3 >= 0 && valLSA3 <= 70)  // at the junction
  {
    //Serial.print("junction");
    motorEquations(0 * PI / 180.0, 0);
    moveMotors();
    delay(100);
    over = 1;
  }
  readLSA();
  if ( valLSA3 == 255 && over == 1)  // correction for the overshoot
  {
    Serial.println("overshoot");
    motorEquations(0 * PI / 180.0, 70);
    addPID();
    moveMotors();
    delay(1055);
    mag[0] = 0;
    mag[1] = 0;
    mag[2] = 0;
    motorEquations(0 * PI / 180.0, 0);
    moveMotors();
    over = 0;
    state++;
  }
}

void state3() //rotation 90 degrees at J1
{
  readLSA();
  while (valLSA1 >= 0 && valLSA1 <= 70)
  {
    mag[0] = -50;
    mag[1] = -50;
    mag[2] = -50;
    moveMotors();
    readLSA();
  }
  while (valLSA1 == 255)  // to get the front LSA on the line
  {
    mag[0] = -35;
    mag[1] = -35;
    mag[2] = -35;
    moveMotors();
    readLSA();
    delay(150);   //to give a slight offset to the front LSA
  }

  while (valLSA2 == 255) //to get the back LSA on the line 
  {
    mag[0] = 35;
    mag[1] = 35;
    mag[2] = -25;
    moveMotors();
    readLSA();
    delay(200);
  }

  mag[0] = 0;
  mag[1] = 0;
  mag[2] = 0;
  moveMotors();
  delay(2000);

  motorEquations(0 * PI / 180.0, 100);
  moveMotors();
  delay(1000);
  state++;
}

void state4()
{
  stat = 'A';
  RGB_Sensor_Response();
  if (var_red < 50)
    stat = 'R';
  if (stat == 'R')
  {
    motorEquations(0 * PI / 180.0, -100);
    addPID();
    moveMotors();
    delay(500);
    motorEquations(0 * PI / 180.0, 0);
    moveMotors();
    Serial.println("REDDDD");
    delay(2000);               // Add transfer sync
    motorEquations(0 * PI / 180.0, 100);
    moveMotors();
    delay(1000);
    state++;
  }
  else
  {
    readLSA();
    motorEquations(0 * PI / 180.0, 100);
    addPID();
    moveMotors();

  }
}

void state5() //Red line to TZJ1
{

   readLSA();
  while (valLSA3 == 255 && over == 0)
  {
    readLSA();
    motorEquations(0 * PI / 180.0, 100);
    addPID();
    moveMotors();
  }
  if (valLSA3 >= 0 && valLSA3 <= 70)
  {
    //Serial.print("junction");
    motorEquations(0 * PI / 180.0, 0);
    moveMotors();
    delay(100);
    over = 1;
  }
  readLSA();
  if ( valLSA3 == 255 && over == 1)
  {
    Serial.println("overshoot");
    motorEquations(180 * PI / 180.0, 70);
    addPID();
    moveMotors();
    delay(1040);
    mag[0] = 0;
    mag[1] = 0;
    mag[2] = 0;
    motorEquations(0 * PI / 180.0, 0);
    moveMotors();
    over = 0;
    state++;

  }
}

void state6() //rotate at TZJ1 for throwing
{

  readLSA();
  while (valLSA1 >= 0 && valLSA1 <= 70)
  {
    mag[0] = 50;
    mag[1] = 50;
    mag[2] = 50;
    moveMotors();
    readLSA();
  }
  while (valLSA1 == 255)
  {
    mag[0] = 35;
    mag[1] = 35;
    mag[2] = 35;
    moveMotors();
    readLSA();
    delay(100);
  }

  while (valLSA2 == 255)
  {
    mag[0] = 35;
    mag[1] = 35;
    mag[2] = -45;
    moveMotors();
    readLSA();
    delay(100);
  }

  mag[0] = 0;
  mag[1] = 0;
  mag[2] = 0;
  moveMotors();
  delay(2000);    //Add Throwing feedback 

  state++;

}

void state7() //rotate at TZJ1 for gripping
{
  readLSA();
  while (valLSA1 >= 0 && valLSA1 <= 70)
  {
    mag[0] = -50;
    mag[1] = -50;
    mag[2] = -50;
    moveMotors();
    readLSA();
  }
  while (valLSA1 == 255)
  {
    mag[0] = -35;
    mag[1] = -35;
    mag[2] = -40;
    moveMotors();
    readLSA();
    delay(70);
  }

  while (valLSA2 == 255)
  {
    mag[0] = -35;
    mag[1] = -35;
    mag[2] = 25;
    moveMotors();
    readLSA();
    delay(200);
  }

  mag[0] = 0;
  mag[1] = 0;
  mag[2] = 0;
  moveMotors();
  delay(2000);

  state++;

}

void state8() // backward till red line for griping  ------call state 4 to go to TZJ1
{
   stat = 'A';
  RGB_Sensor_Response();
  if (var_red < 50)
    stat = 'R';
  if (stat == 'R')
  {
    motorEquations(18 * PI / 180.0, -100);
    addPID();
    moveMotors();
    delay(500);
    motorEquations(180 * PI / 180.0, 0);
    moveMotors();
    Serial.println("REDDDD");
    delay(2000);
    motorEquations(180 * PI / 180.0, 100);
    moveMotors();
    delay(1000);
    state++;
  }
  else
  {
    readLSA();
    motorEquations(180 * PI / 180.0, 100);
    addPID();
    moveMotors();

  }
}

void state9() // backward from red line to Manual zone Junction 1
{
  //colour sensor check red line
  readLSA();
  while (valLSA3 == 255 && over == 0)
  {
    readLSA();
    motorEquations(180 * PI / 180.0, 100);
    addPID();
    moveMotors();
  }
  if (valLSA3 >= 0 && valLSA3 <= 70)
  {
    //Serial.print("junc");
    motorEquations(0 * PI / 180.0, 0);
    moveMotors();
    delay(100);
    over = 1;
  }
  readLSA();
  if ( valLSA3 == 255 && over == 1)
  {
    Serial.println("overshoot");
    motorEquations(0 * PI / 180.0, 70);
    addPID();
    moveMotors();
    delay(1055);
    mag[0] = 0;
    mag[1] = 0;
    mag[2] = 0;
    motorEquations(0 * PI / 180.0, 0);
    moveMotors();
    over = 0;
    state++;
  }

}

void state10() // rotation at manual zone junction 1 to go to junction 2
{
  readLSA();
  while (valLSA1 >= 0 && valLSA1 <= 70)
  {
    mag[0] = 50;
    mag[1] = 50;
    mag[2] = 50;
    moveMotors();
    readLSA();
  }
  while (valLSA1 == 255)
  {
    mag[0] = 35;
    mag[1] = 35;
    mag[2] = 35;
    moveMotors();
    readLSA();
    delay(150);
  }

  while (valLSA2 == 255)
  {
    mag[0] = -35;
    mag[1] = -35;
    mag[2] = 25;
    moveMotors();
    readLSA();
    delay(200);
  }

  mag[0] = 0;
  mag[1] = 0;
  mag[2] = 0;
  moveMotors();
  delay(2000);

  motorEquations(0 * PI / 180.0, 100);
  moveMotors();
  delay(2000);
  state++;
}

void state11() //Manual Zone Junction 1 to Junction2
{
  if (valLSA3 >= 0 && valLSA3 <= 70)
  {
    motorEquations(0 * PI / 180.0, 100);
    moveMotors();
  }
}






