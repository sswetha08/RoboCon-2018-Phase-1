class ThreeWheelBotMotors {
    /*
       Important functions :
     * * attachPWM_DIRPins(PWMs[], DIRs[])
     * * moveAtWithAngle(int PWM, int ANGLE_DEGREES)
     * * increasePWMs(value (s))
     * * attachDebuggerSerial(&Serial)
     * * enableDebugger()
     * * disableDebugger()
    */
    int MOTOR_MAG[3];
    int MAX_MAG;
    int MOTOR_MAG_PINS[3];
    int MOTOR_DIR_PINS[3];
    bool DEBUGGER_MODE;
    HardwareSerial *debuggerSerial;
    void setMotorPWM(int number, float PWM) {
      if (PWM > 0) {
        digitalWrite(MOTOR_DIR_PINS[number], LOW);  // Rotate inwards
      }
      else if (PWM < 0) {
        digitalWrite(MOTOR_DIR_PINS[number], HIGH); // Rotate outwards
      }
      if (abs(PWM) < MAX_MAG) {
        MOTOR_MAG[number] = PWM;
        analogWrite(MOTOR_MAG_PINS[number], abs((int)PWM));
      } else if (DEBUGGER_MODE == true) {
        debuggerSerial->print("\nMotor index ");
        debuggerSerial->print(number);
        debuggerSerial->print(" exceeded MAX_PWM (");
        debuggerSerial->print(MAX_MAG);
        debuggerSerial->print(")- ");
        debuggerSerial->println(PWM);
      }
    }
  public:
    // Attach Pins
    void attachPWM_DIRPins(int PWM_PINS[], int DIR_PINS[]) {
      attachPWMPins(PWM_PINS[0], PWM_PINS[1], PWM_PINS[2]);
      attachDIRPins(DIR_PINS[0], DIR_PINS[1], DIR_PINS[2]);
    }
    void attachDIRPins(int dir_M1, int dir_M2, int dir_M3) {
      MOTOR_DIR_PINS[0] = dir_M1;
      MOTOR_DIR_PINS[1] = dir_M2;
      MOTOR_DIR_PINS[2] = dir_M3;
      pinMode(dir_M1, OUTPUT);
      pinMode(dir_M2, OUTPUT);
      pinMode(dir_M3, OUTPUT);
      if (DEBUGGER_MODE) {
        debuggerSerial->print("\nMotor DIR Pins attached to ");
        debuggerSerial->print(dir_M1);
        debuggerSerial->print(", ");
        debuggerSerial->print(dir_M2);
        debuggerSerial->print(", ");
        debuggerSerial->println(dir_M3);
      }
    }
    void attachPWMPins(int PWM_M1, int PWM_M2, int PWM_M3) {
      MOTOR_MAG_PINS[0] = PWM_M1;
      MOTOR_MAG_PINS[1] = PWM_M2;
      MOTOR_MAG_PINS[2] = PWM_M3;
      pinMode(PWM_M1, OUTPUT);
      pinMode(PWM_M2, OUTPUT);
      pinMode(PWM_M3, OUTPUT);
      if (DEBUGGER_MODE) {
        debuggerSerial->print("\nMotor PWM Pins attached to ");
        debuggerSerial->print(PWM_M1);
        debuggerSerial->print(", ");
        debuggerSerial->print(PWM_M2);
        debuggerSerial->print(", ");
        debuggerSerial->println(PWM_M3);
      }
    }
    ThreeWheelBotMotors() {
      // CAUTION : Set PWMs to 0
      MAX_MAG = 0;
      MOTOR_MAG[0] = MOTOR_MAG[1] = MOTOR_MAG[2] = 0;
      DEBUGGER_MODE = false;
      debuggerSerial = NULL;
    }
    void setMaxPWM(int MAX_PWM) {
      MAX_MAG = MAX_PWM;
    }
    //Debugger Mode
    void attachDebuggerSerial(HardwareSerial *s) {
      debuggerSerial = s;
      DEBUGGER_MODE = true;
    }
    void disableDebugger() {
      DEBUGGER_MODE = false;
    }
    void enableDebugger() {
      DEBUGGER_MODE = true;
    }
    // Alter motor PWMs
    void increasePWMs(int mag_M1, int mag_M2, int mag_M3) {
      MOTOR_MAG[0] += mag_M1;
      MOTOR_MAG[1] += mag_M2;
      MOTOR_MAG[2] += mag_M3;
    }
    void decreasePWMs(int mag_M1, int mag_M2, int mag_M3) {
      increasePWMs(-mag_M1, -mag_M2, -mag_M3);
    }
    void decreasePWMs(int M) {
      decreasePWMs(M, M, M);
    }
    void increasePWMs(int M) {
      increasePWMs(M, M, M);
    }
    // Motion of motor
    void moveAtWithAngle(int PWM, int ANGLE) {
      moveAtWithAngle_DEG(PWM, ANGLE);
    }
    void moveAtWithAngle_RAD(float PWM, float ANGLE) {
      /*
         Derived DCM Equations
         Equations :->
        [
        1, 1, 1;
        1, -cos(60), -cos(60);
        0, -sin(60), sin(60)
        ] * [v1;v2;v3] = [0; V Sin(THETA) ; V Cos(THETA)]
        ----- 0 degrees is the wheel 1 (index 0), angle taken counter clockwise
         WARNING : DO NOT CHANGE IF NOT UNDERSTOOD
         V1 = 0.6666666 * (PWM) * sin(ANGLE)
         V2 = -0.3333333 * (PWM) * sin(ANGLE) - 0.57735027 * (PWM) * cos(ANGLE)
         V3 = -0.3333333 * (PWM) * sin(ANGLE) + 0.57735027 * (PWM) * cos(ANGLE)
      */
      setMotorPWM(0, 0.6666666 * PWM * sin(ANGLE));
      setMotorPWM(1, -0.33333333 * PWM * sin(ANGLE) - 0.57735027 * PWM * cos(ANGLE));
      setMotorPWM(2, -0.33333333 * PWM * sin(ANGLE) + 0.57735027 * PWM * cos(ANGLE));
    }
    void moveAtWithAngle_DEG(int PWM, int ANGLE_DEG) {
      moveAtWithAngle_RAD(PWM, (float(ANGLE_DEG)/180.0)*PI);
    }

    void printMotorStatus() {
      Serial.print("\nMotor 1 : ");
      Serial.print(MOTOR_MAG[0]);
      Serial.print("\nMotor 2 : ");
      Serial.print(MOTOR_MAG[1]);
      Serial.print("\nMotor 3 : ");
      Serial.println(MOTOR_MAG[2]);
    }

    void setMotorStatus(int M_NO, int PWM, int DIR) {
      digitalWrite(MOTOR_DIR_PINS[M_NO], DIR);
      analogWrite(MOTOR_MAG_PINS[M_NO], PWM);
      Serial.print("Pin ");
      Serial.print(MOTOR_MAG_PINS[M_NO]);
      Serial.print(" at ");
      Serial.print(PWM);
      Serial.print(" - dir pin ");
      Serial.print(MOTOR_DIR_PINS[M_NO]);
      Serial.print(" at ");
      Serial.println(DIR);
    }
};


ThreeWheelBotMotors motors;

int MotorPWMs[3] = {8, 9, 10};
int MotorDIRs[3] = {5, 6, 3};
void setup() {
  Serial.begin(9600);
  motors.attachDebuggerSerial(&Serial);
  motors.attachPWM_DIRPins(MotorPWMs, MotorDIRs);
  motors.setMaxPWM(100);
}
int CURRENT_ANGLE = -60, CURRENT_PWM = 50;
void loop() {
  while (Serial.available()) {
    char c = Serial.read();
    switch (c) {
      case 'p': case 'P':
        CURRENT_PWM = Serial.parseInt();
        break;
      case 'a': case 'A':
        CURRENT_ANGLE = Serial.parseInt();
        break;
      default: break;
    }
  }
  /*
    Serial.print("PWM - ");
    Serial.print(CURRENT_PWM);
    Serial.print(" ANGLE = ");
    Serial.println(CURRENT_ANGLE);
  */
  motors.moveAtWithAngle(CURRENT_PWM, CURRENT_ANGLE);
 // motors.setMotorStatus(2, 40, 1);
  motors.printMotorStatus();
  /*
    digitalWrite(7, 0);
    analogWrite(10, 100);
  */
}

