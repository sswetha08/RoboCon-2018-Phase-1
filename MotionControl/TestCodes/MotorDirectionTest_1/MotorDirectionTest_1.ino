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

void initialiseLSASerialTo(HardwareSerial *serialPort, int baudRate = 38400);

void setup() {
  initialiseLSASerialTo(&Serial1);
  setupLSAPinModes();
  Serial.begin(9600);

  setupMotorPinModes();
}

void turnMotorInDirection(int motorNumber, boolean _direction, int magnitude = 20);

void loop() {
  ///readLSAs();
  initialiseMotorPWMsForDirection(angularDirection, defaultMotorMagnitude);
  runMotors();
  if (DEVELOPER_MODE == ON) {
    printMotorStatus();
    motionControlUsingSerial();
  }
  //  turnMotorInDirection(0,HIGH);
  //  turnMotorInDirection(1,HIGH);
  //  turnMotorInDirection(2,HIGH);
}

void motionControlUsingSerial() {
  if (Serial.available()) {
    while (Serial.available()) {
      char c = Serial.read();
      switch (c) {
        case 'D':
          angularDirection = (Serial.parseInt()) * PI / 180.0;
          break;
        case 'M': case 'S':
          defaultMotorMagnitude = Serial.parseInt();
          break;
        default:
          break;
      }
    }
  }
}

void readLSAs() {
  clearLSADataBuffer();

  digitalWrite(enableLSA1, HIGH);
  while (!LSASerial->available()); //Wait to receive anything;
  LSASerial->read();  //Reading junk here
  valLSA1 = LSASerial->read();
  digitalWrite(enableLSA1, LOW);

  digitalWrite(enableLSA2, HIGH);
  while (!LSASerial->available()); //Wait to receive anything;
  LSASerial->read();
  valLSA2 = LSASerial->read();
  digitalWrite(enableLSA2, LOW);

  digitalWrite(enableLSA3, HIGH);
  while (!LSASerial->available()); //Wait to receive anything;
  LSASerial->read();
  valLSA3 = LSASerial->read();
  digitalWrite(enableLSA3, LOW);

  Serial.print("The LSA values are : ");
  Serial.print(valLSA1);
  Serial.print(", ");
  Serial.print(valLSA2);
  Serial.print(", ");
  Serial.print(valLSA3);
  Serial.print(", ");
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
