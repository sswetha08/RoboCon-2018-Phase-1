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
    int MOTOR_MAG[3]; // Current Magnitude of Motors
    int MAX_MAG;      // Maximum PWM cap on motors
    int MOTOR_MAG_PINS[3];  // Motor Magnitude Pins
    int MOTOR_DIR_PINS[3];  // Motor Direction Pins
    boolean MOTOR_CW[3];    // Voltage levels to rotate the motors CW
    bool DEBUGGER_MODE;     // Debugger mode (ON/OFF)
    HardwareSerial *debuggerSerial;
    void setMotorPWM(int number, float PWM) { // Motor 'number' and 'PWM' value (set motor index 'number' to PWM) (it does a MAX_MAG test)
      if (PWM > 0) {
        digitalWrite(MOTOR_DIR_PINS[number], MOTOR_CW[number]);  // Rotate inwards : Support CW Direction
      }
      else if (PWM < 0) {
        digitalWrite(MOTOR_DIR_PINS[number], !MOTOR_CW[number]); // Rotate outwards : CCW Direction
      }
      if (abs(PWM) < MAX_MAG) {
        MOTOR_MAG[number] = PWM;  // Set MOROT_MAG to PWM
        analogWrite(MOTOR_MAG_PINS[number], abs((int)PWM));
      } else if (DEBUGGER_MODE == true) {
        debuggerSerial->print("\nMotor index ");
        debuggerSerial->print(number);
        debuggerSerial->print(" exceeded MAX_PWM (");
        debuggerSerial->print(MAX_MAG);
        debuggerSerial->print(")- Value ");
        debuggerSerial->println(PWM);
      }
    }
  public:
    // Attach Pins
    void attachPWM_DIRPins(int PWM_PINS[], int DIR_PINS[], boolean MOTOR_CW_PINS[]) { // PWM_PINS, DIR_PINS, CW_DIR_PINS
      attachPWMPins(PWM_PINS[0], PWM_PINS[1], PWM_PINS[2]);
      attachDIRPins(DIR_PINS[0], DIR_PINS[1], DIR_PINS[2]);
      attachCWDirs(MOTOR_CW_PINS[0],MOTOR_CW_PINS[1],MOTOR_CW_PINS[2]);
    }
    void attachCWDirs(boolean CW0, boolean CW1, boolean CW2) { // Attach CW0, CW1, CW2 (for Motor 0,1,2)
      MOTOR_CW[0] = CW0;
      MOTOR_CW[1] = CW1;
      MOTOR_CW[2] = CW2;
      if (DEBUGGER_MODE) {
        debuggerSerial->print("\nMotor directions ");
        for (int i = 0; i < 3; i++) {
          if (MOTOR_CW[i] == HIGH) {
            debuggerSerial->print("CW\t");
          } else {
            debuggerSerial->print("CCW\t");
          }
        }
        debuggerSerial->println();
      }
    }
    void attachDIRPins(int dir_M1, int dir_M2, int dir_M3) {  // Attach dir0, dir1, dir2 (for Motor 0,1,2)
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
    void attachPWMPins(int PWM_M1, int PWM_M2, int PWM_M3) { // Attach PWM0, PWM1, PWM2 (for Motor 0,1,2)
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
    ThreeWheelBotMotors() { // Initialise everything to 0
      // CAUTION : Set PWMs to 0
      MAX_MAG = 0;
      MOTOR_MAG[0] = MOTOR_MAG[1] = MOTOR_MAG[2] = 0;
      DEBUGGER_MODE = false;
      debuggerSerial = NULL;
    }
    void setMaxPWM(int MAX_PWM) { // Assign maximum PWM value
      MAX_MAG = MAX_PWM;
    }
    //Debugger Mode
    void attachDebuggerSerial(HardwareSerial *s) { // Pass it a HardwareSerial, it'll attach it to debuggerSerial
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
    void increasePWMs(int mag_M1, int mag_M2, int mag_M3) { // Increase M0 by mag_M1, M1 by mag_M2 and M2 by mag_M3
      setMotorPWM(0,MOTOR_MAG[0] + mag_M1);
      setMotorPWM(1,MOTOR_MAG[1] + mag_M1);
      setMotorPWM(2,MOTOR_MAG[2] + mag_M1);
      if (DEBUGGER_MODE) {
        debuggerSerial->print("\nMotor PWM Delta = ");
        debuggerSerial->print(mag_M1);
        debuggerSerial->print(", ");
        debuggerSerial->print(mag_M2);
        debuggerSerial->print(", ");
        debuggerSerial->print(mag_M3);
        debuggerSerial->print("\tCurrent Status = ");
        debuggerSerial->print(MOTOR_MAG[0]);
        debuggerSerial->print(",");
        debuggerSerial->print(MOTOR_MAG[1]);
        debuggerSerial->print(",");
        debuggerSerial->print(MOTOR_MAG[2]);
        debuggerSerial->println();
      }
    }
    void decreasePWMs(int mag_M1, int mag_M2, int mag_M3) { // Decrease M0 by mag_M1, M1 by mag_M2 and M2 by mag_M3
      increasePWMs(-mag_M1, -mag_M2, -mag_M3);
    }
    void decreasePWMs(int M) {  // Decrease all motors by M
      decreasePWMs(M, M, M);
    }
    void increasePWMs(int M) {  // Increase all motors by M
      increasePWMs(M, M, M);
    }
    // Motion of motor
    void moveAtWithAngle(int PWM, int ANGLE) {  // Move at 'PWM' keeping angle 'ANGLE' with vertical (DEFAULT ANGLE IN DEGREES)
      moveAtWithAngle_DEG(PWM, ANGLE);
    }
    void moveAtWithAngle_RAD(float PWM, float ANGLE) {  // Move at 'PWM' keeping angle 'ANGLE' with vertical (ANGLE IN RADIANS)
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
    void moveAtWithAngle_DEG(int PWM, int ANGLE_DEG) { // Move at 'PWM' keeping angle 'ANGLE' with vertical (ANGLE IN DEGREES)
      moveAtWithAngle_RAD(PWM, (float(ANGLE_DEG)/180.0)*PI);
    }

    void printMotorStatus() { // Print Motor PWMs to the serial (debuggerSerial)
      Serial.print("\nMotor 1 : ");
      Serial.print(MOTOR_MAG[0]);
      Serial.print("\nMotor 2 : ");
      Serial.print(MOTOR_MAG[1]);
      Serial.print("\nMotor 3 : ");
      Serial.println(MOTOR_MAG[2]);
    }

    void setMotorStatus(int M_NO, int PWM, int DIR) { // Set motor M_NO to PWM and direction (voltage) as DIR
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

// Motor properties
int MotorPWMs[3] = {2, 5, 4};
int MotorDIRs[3] = {32, 30, 28};
boolean CW_DIRS[3] = {HIGH, HIGH, HIGH};  // The voltages levels we need to give to pins to rotate in a clockwise manner [M0,M1,M2]

void setup() {
  Serial.begin(9600);
  motors.attachDebuggerSerial(&Serial);
  motors.attachPWM_DIRPins(MotorPWMs, MotorDIRs, CW_DIRS);
  motors.setMaxPWM(100);
}
int CURRENT_ANGLE = -90, CURRENT_PWM = 80;
void loop() {
  while (Serial.available()) {  // For Debugger mode
    char c = Serial.read();
    switch (c) {
      case 'p': case 'P': // Edit PWM
        CURRENT_PWM = Serial.parseInt();
        break;
      case 'a': case 'A': // Edit Angle
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
  // Move Bot 
  // motors.moveAtWithAngle(CURRENT_PWM, CURRENT_ANGLE);
  /*
  // Rotate bot clockwise
  motors.setMotorStatus(2, 40, 1);
  motors.setMotorStatus(1, 40, 1);
  motors.setMotorStatus(0, 40, 1);
  */
  motors.printMotorStatus();
  /*
    digitalWrite(7, 0);
    analogWrite(10, 100);
  */
}
