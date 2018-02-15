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

int state=0;



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
  if(state==0)
  {
    while(state==0)
    {
      state1();
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


  if (valLSA1 == 255)
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
void readLSA()
{
  Serial.print("Values : ");
  /*clearLSA();
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

  valLSA1 = analogRead(LSA_Pin1);
  if (valLSA1 <= 900)
  {
    valLSA1 = map(valLSA1, 0, 900, 0, 70);
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
  dr = rfrequency - 85;
  dg = gfrequency - 322;
  db = bfrequency - 271;
  var_red = sqrt(dr * dr + db * db + dg * dg);
}

void state1()
{
  stat = 'A';
  RGB_Sensor_Response();
  if (var_red < 50)
    stat = 'R';
  if (stat == 'R')
  {
    motorEquations(0 * PI / 180.0, -100);
    moveMotors();
    delay(500);
    motorEquations(0 * PI / 180.0, 0);
    moveMotors();
    Serial.println("REDDDD");
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
