//Developer Information
#define ON 1
#define OFF 0

#define DEVELOPER_MODE ON
//LSA
int enableLSA1 = 22;
int enableLSA2 = 24;
int enableLSA3 = 28;
int valLSA1 = 0;
int valLSA2 = 0;
int valLSA3 = 0;
HardwareSerial  *LSASerial = NULL;

//Motors
int pwmPins[3] = {6, 3, 4};
int dirPins[3] = {7, 2, 5};
int mag[3] = {};

//Developer
double angularDirection = 180 * (PI / 180.0);
double defaultMotorMagnitude = 40;

//PID
double Kp = 1.2, Kd = 0.8;
double previousError = 0;

void initialiseLSASerial(HardwareSerial *serialPort);


void setup() {
  for (int i = 0; i < 3; i++)
  {
    pinMode(pwmPins[i], OUTPUT);
    pinMode(dirPins[i], OUTPUT);
  }
  initialiseLSASerial(&Serial1);
  LSASerial->begin(38400);
  Serial.begin(9600);
  pinMode(enableLSA1, OUTPUT);
  pinMode(enableLSA2, OUTPUT);
  pinMode(enableLSA3, OUTPUT);

  digitalWrite(enableLSA1, LOW);
  digitalWrite(enableLSA2, LOW);
  digitalWrite(enableLSA3, LOW);
  readLSA();  //Store buffer
}



void loop() {
  readLSA();
  motorEquations(180 * PI / 180, 40);
  addPID();
  moveMotors();
  if (DEVELOPER_MODE == ON) {
    printMotorStatus();
    controlUsingSerial();
  }

}

//Developer opions  #dev
void controlUsingSerial() {
  if (Serial.available()) {
    while (Serial.available()) {
      char c = Serial.read();
      switch (c) {
        case 'D': //Direction of motion of bot in degrees
          angularDirection = (Serial.parseInt()) * PI / 180.0;
          break;
        case 'M': case 'S': //Motor speed
          defaultMotorMagnitude = Serial.parseInt();
          break;
        case 'K':
          c = Serial.read();
          if (c == 'p') { //Set Kp
            Kp = Serial.parseInt();
          } else if (c == 'd') {  //Set Kd
            Kd = Serial.parseInt();
          }
        default:
          break;
      }
    }
  }
}

//#PID
void addPID() {
  int value;
  if (angularDirection == 0 * PI / 180.0) {
    value = valLSA1;
  }
  else if (angularDirection == 180 * PI / 180.0) {
    value = valLSA2;
  } else if (angularDirection == 90 * PI / 180) {
    value = valLSA3;
  } else {
    return; //We haven't though about this
  }
  int error = 35 - value;


  float PIDCorrection = Kp * error - Kd * (error - previousError);

  if (DEVELOPER_MODE == ON) {
    Serial.print("PID Status: ");
    Serial.print("Error = ");
    Serial.print(error);
    Serial.print("\tPID Correction = ");
    Serial.print(PIDCorrection);
  }

  for (int i = 0 ; i < 3 ; i++) {
    mag[i] += PIDCorrection;

    if (value == 255)
    {
       mag[0] = -0.5 * mag[0];
       mag[1] = -0.5 * mag[1];
       mag[2] = -0.5 * mag[2];
      while (valLSA2 == 255) {
        readLSA();
        }
       mag[0]=0; 
       mag[1]=0;
       mag[2]=0;
    }

  }
}

//LSA
void clearLSA()
{
  while (LSASerial->available())
  {
    LSASerial->read();
  }
}

void readLSA()
{
  Serial.print("Values : ");
  clearLSA();
  digitalWrite(enableLSA1, HIGH);
  delayMicroseconds(100);
  while (!(LSASerial->available()));
  while (LSASerial->available())
  {
    valLSA1 = LSASerial->read();
    if (valLSA2 == 255) continue;
    else break;

  }
  digitalWrite(enableLSA1, LOW);

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
    if (valLSA2 == 255) continue;
    else break;
  }
  digitalWrite(enableLSA3, LOW);
  clearLSA();
  delayMicroseconds(100);



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

//#Motor
void printMotorStatus() {
  Serial.print("\nMotor status ");
  for (int i = 0 ; i < 3; i++) {
    Serial.print(i);
    Serial.print(") ");
    Serial.print(mag[i]);
    Serial.print("   ");
  }
  Serial.print("A,M = ");
  Serial.print(angularDirection * 180.0 / PI);
  Serial.print(", ");
  Serial.print(defaultMotorMagnitude);
  Serial.println();
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
  }
}
