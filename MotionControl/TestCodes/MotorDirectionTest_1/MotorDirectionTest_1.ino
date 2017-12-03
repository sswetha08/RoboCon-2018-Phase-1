//Developer Information
#define ON 1
#define OFF 0

#define DEVELOPER_MODE ON

//LSA details
int enableLSA1 = 22;
int enableLSA2 = 24;
int enableLSA3 = 28;
int valLSA1 = 0;
int valLSA2 = 0;
int valLSA3 = 0;
HardwareSerial  *LSASerial = NULL;

//Motor details
int pwmPins[3] = {6, 3, 4 };
int dirPins[3] = {7, 2, 5};
int mag[3] = {};
//Developer - @AM
double angularDirection = 180 * (PI / 180.0);
double defaultMotorMagnitude = 60;

//PID
double Kp = 1, Kd = 0;
double previousError = 0;

void initialiseLSASerialTo(HardwareSerial *serialPort, int baudRate = 38400);

void setup() {
  initialiseLSASerialTo(&Serial1);
  setupLSAPinModes();
  Serial.begin(9600);

  setupMotorPinModes();
}

void turnMotorInDirection(int motorNumber, boolean _direction, int magnitude = 20);

void loop() {
  readLSAs();
  initialiseMotorPWMsForDirection(angularDirection, defaultMotorMagnitude);
  runMotors();
  if (DEVELOPER_MODE == ON) {
    printMotorStatus();
    controlUsingSerial();
  }
  //  turnMotorInDirection(0,HIGH);
  //  turnMotorInDirection(1,HIGH);
  //  turnMotorInDirection(2,HIGH);
}

//PID

void applyPID() {
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

  float PIDCorrection = Kp * error + Kd * (error - previousError);

  if (DEVELOPER_MODE == ON) {
    Serial.println("PID Status");
    Serial.print("Error = ");
    Serial.print(error);
    Serial.print("\tPID Correction = ");
    Serial.print(PIDCorrection);
  }

  for (int i = 0 ; i < 3 ; i++) {
    mag[i] += PIDCorrection;
  }
}


//Developer opions
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




//#LSA
void getLSASerialDataInto(int &variable, int LSANumber = -1, int timeout = 10);
void getLSASerialDataInto(int &variable, int LSANumber, int timeout) {
  //Wait for the serial to respond with a certain timeout
  long entranceTime = millis();
  while (!LSASerial->available()) {
    if (millis() - entranceTime >= timeout) { //Request timeout error
      variable = NULL;
      Serial.print("LSA ");
      Serial.print((LSANumber == -1) ? -1 : LSANumber);
      Serial.print(" is not responding - Request timeout error\n");
    }
  }
  while(LSASerial->read() == 255) {
    Serial.println("Got 255 in the serial, ignoring it...");
  }
  variable = LSASerial->read(); //Actual data read and fucntion returns;
  return;
}

void readLSAs() {
  clearLSADataBuffer();

  digitalWrite(enableLSA1, HIGH);
  getLSASerialDataInto(valLSA1, 1);
  digitalWrite(enableLSA1, LOW);

  digitalWrite(enableLSA2, HIGH);
  getLSASerialDataInto(valLSA2, 2);
  digitalWrite(enableLSA2, LOW);

  digitalWrite(enableLSA3, HIGH);
  getLSASerialDataInto(valLSA3, 3);
  digitalWrite(enableLSA3, LOW);

  Serial.print("The LSA values are : ");
  Serial.print(valLSA1);
  Serial.print(", ");
  Serial.print(valLSA2);
  Serial.print(", ");
  Serial.print(valLSA3);
  Serial.print("\n");
}

void clearLSADataBuffer() {
  while (LSASerial->available()) {
    LSASerial->read();
  }
}

void setupLSAPinModes() {
  pinMode(enableLSA1, OUTPUT);
  pinMode(enableLSA2, OUTPUT);
  pinMode(enableLSA3, OUTPUT);

  digitalWrite(enableLSA1, LOW);
  digitalWrite(enableLSA2, LOW);
  digitalWrite(enableLSA3, LOW);
}

void initialiseLSASerialTo(HardwareSerial *serialPort, int baudRate) {
  LSASerial = serialPort;
  LSASerial->begin(baudRate);
}


//#Motor
void printMotorStatus() {
  Serial.print("\nMotor status ");
  for (int i = 0 ; i < 3; i++) {
    Serial.print(i);
    Serial.print(") ");
    Serial.print(mag[i]);
    Serial.print("\t");
  }

}

void setupMotorPinModes() {
  for (int i = 0; i < 3 ; i++) {
    pinMode(pwmPins[i], OUTPUT);
    pinMode(dirPins[i], OUTPUT);
  }
}

void runMotors() {
  for (int i = 0; i < 3; i++) {
    analogWrite(pwmPins[i], abs(mag[i]));
    digitalWrite(dirPins[i], (mag[i] > 0) ? HIGH : LOW);
  }
}

void initialiseMotorPWMsForDirection(double angle, double magController) {
  mag[0] = magController * cos(150 * PI / 180.0 - angle);
  mag[1] = magController * cos(30 * PI / 180.0 - angle);
  mag[2] = magController * cos(270 * PI / 180.0 - angle);
}

void turnMotorInDirection(int motorNumber, boolean _direction, int magnitude) {
  analogWrite(pwmPins[motorNumber], magnitude);
  digitalWrite(dirPins[motorNumber], _direction);
}
