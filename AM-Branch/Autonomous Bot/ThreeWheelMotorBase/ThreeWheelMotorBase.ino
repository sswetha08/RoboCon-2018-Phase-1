#define DEBUGGER true
#define Serial_BAUD_RATE 9600

class ThreeWheelBotMotors {
    /*
       Important functions :
     * * attachPWM_DIRPins(PWMs[], DIRs[], CCWs_voltage_levels[])
     * * moveAtWithAngle(int PWM, int ANGLE_DEGREES)
     * * increasePWMs(value_M1,value_M2,value_M3) , or give only one value for all three wheels
     * * decreasePWMs(value_M1,value_M2,value_M3) , or give only one value for all three wheels
     * * attachDebuggerSerial(&Serial) , attach debugger serial to the &Serial
     * * enableDebugger()
     * * disableDebugger()
     * * setMotorStatus(MOTOR_NUMBER, PWM, DIRECTION) ------ DEBUGGER MODE ONLY
     * * setFWD_PDparameters(KP,KD)
     * * setBWD_PDparameters(KP,KD)
     * * applyXAxisMotionCorrection(int LSA_Values[], int lastLSA_Values[])
    */
    int MOTOR_MAG[3]; // Current Magnitude of Motors
    int MAX_MAG;      // Maximum PWM cap on motors
    int MOTOR_MAG_PINS[3];  // Motor Magnitude Pins
    int MOTOR_DIR_PINS[3];  // Motor Direction Pins
    boolean MOTOR_CCW[3];    // Voltage levels to rotate the motors CCW : If viewed from top, the bot will rotate counter clockwise with these voltage levels given to the motor
    bool DEBUGGER_MODE;     // Debugger mode (ON/OFF)
    HardwareSerial *debuggerSerial;
    // Don't move this function to public:, it has to happen in private
    void setMotorPWM(int number, float PWM) { // Motor 'number' and 'PWM' value (set motor index 'number' to PWM) (it does a MAX_MAG test)
      if (PWM > 0) {
        digitalWrite(MOTOR_DIR_PINS[number], MOTOR_CCW[number]);  // Rotate inwards : Support CCW Direction
      }
      else if (PWM < 0) {
        digitalWrite(MOTOR_DIR_PINS[number], !MOTOR_CCW[number]); // Rotate outwards : CW Direction
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
    void attachPWM_DIRPins(int PWM_PINS[], int DIR_PINS[], boolean MOTOR_CCW_PINS[]) { // PWM_PINS, DIR_PINS, CW_DIR_PINS
      attachPWMPins(PWM_PINS[0], PWM_PINS[1], PWM_PINS[2]);
      attachDIRPins(DIR_PINS[0], DIR_PINS[1], DIR_PINS[2]);
      attachCCWDirs(MOTOR_CCW_PINS[0],MOTOR_CCW_PINS[1],MOTOR_CCW_PINS[2]);
    }
    void attachCCWDirs(boolean CCW0, boolean CCW1, boolean CCW2) { // Attach CCW0, CCW1, CCW2 (for Motor 0,1,2)
      MOTOR_CCW[0] = CCW0;
      MOTOR_CCW[1] = CCW1;
      MOTOR_CCW[2] = CCW2;
      if (DEBUGGER_MODE) {
        debuggerSerial->print("\nMotor directions ");
        for (int i = 0; i < 3; i++) {
          if (MOTOR_CCW[i] == HIGH) {
            debuggerSerial->print("CCW\t");
          } else {
            debuggerSerial->print("CW\t");
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
      /*    **IMPORTANT DCM Equations
         Derived DCM Equations
         Equations :->
        [
        1, 1, 1;
        ] * [v1 ;v2;v3] = [0; V Sin(THETA) ; V Cos(THETA)]
        1, -cos(60), -cos(60);
        0, -sin(60), sin(60)
        The inverse of the matrix is given by
        [
        0.3333333333, 0.6666666666, 0;
        0.3333333333, -0.3333333333, -0.5773502691;
        0.3333333333, -0.3333333333, 0.5773502691
        ]
        ----- 0 degrees is the wheel 1 (index 0), angle taken counter clockwise, then wheel 2 and then wheel 3
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
    void addOmega(float omega) {  // Add rotational velocity
      setMotorPWM(0,MOTOR_MAG[0] + omega/3.0);
      setMotorPWM(1,MOTOR_MAG[1] + omega/3.0);
      setMotorPWM(2,MOTOR_MAG[2] + omega/3.0);
    }
    void setOmega(float omega) {  // Set rotational velocity
      setMotorPWM(0,omega/3.0);
      setMotorPWM(1,omega/3.0);
      setMotorPWM(2,omega/3.0);
    }
    void printMotorStatus() { // Print Motor PWMs to the serial (debuggerSerial)
        if (DEBUGGER_MODE) {
        Serial.print("\nMotor 1 : ");
        Serial.print(MOTOR_MAG[0]);
        Serial.print("\nMotor 2 : ");
        Serial.print(MOTOR_MAG[1]);
        Serial.print("\nMotor 3 : ");
        Serial.println(MOTOR_MAG[2]);
      }
    }
    void setMotorStatus(int M_NO, int PWM, int DIR) { // Set motor M_NO to PWM and direction (voltage) as DIR : Only debugging mode, DON'T USE JUST FOR LOLS
      digitalWrite(MOTOR_DIR_PINS[M_NO], DIR);
      analogWrite(MOTOR_MAG_PINS[M_NO], PWM);
      if (DEBUGGER_MODE) {
        Serial.print("Pin ");
        Serial.print(MOTOR_MAG_PINS[M_NO]);
        Serial.print(" at ");
        Serial.print(PWM);
        Serial.print(" - dir pin ");
        Serial.print(MOTOR_DIR_PINS[M_NO]);
        Serial.print(" at ");
        Serial.println(DIR);
      }
    }

    // Line Following correction
    // Values fine tuned for 150 Magnitude
    float KpFWD = 2, KdFWD = 0.6, KpBWD = 1.7, KdBWD = 0.5;
    void setFWD_PDparameters(float newKpFWD, float newKdFWD) {  // Set PD parameters for PD control (in forward 0 degree motion)
      KpFWD = newKpFWD;
      KdFWD = newKdFWD;
    }
    void setBWD_PDparameters(float newKpBWD, float newKdBWD) {  // Set PD parameters for PD control (in backward, 180 degree motion)
      KpBWD = newKpBWD;
      KdBWD = newKdBWD;
    }
    int errorFWD = 0, prevErrorFWD = 0, errorBWD = 0, prevErrorBWD = 0;
    void applyXAxisMotionCorrection_OnlyFWDLSA(int LSA_Values[],int lastLSA_Values[]) {
      int PWMCorrection = 0;
      // Front wheels
      int LSAValue = 0;
      if (LSA_Values[0] != 255) { // LSA0 on line
        LSAValue = LSA_Values[0];
      } else {  // 255 : out of the line
        if (lastLSA_Values[0] < 35) {
          LSAValue = -20;
        } else {
          LSAValue = 90;
        }
      }
      PWMCorrection = 0;
      // 0 degree wheel PID
      errorFWD = 35 - LSAValue;
      PWMCorrection = KpFWD * errorFWD - KdFWD * (errorFWD - prevErrorFWD); // FWD PWM Correction
      setMotorPWM(0, MOTOR_MAG[0] + PWMCorrection);
      prevErrorFWD = errorFWD;

    }
    void applyXAxisMotionCorrection(int LSA_Values[], int lastLSA_Values[]) { // Apply 0 degree or 180 degree PD control using LSA0 and LSA1
      int PWMCorrection = 0;
      // Front wheels : From front LSA
      int LSAValue = 0;
      if (LSA_Values[0] != 255) { // LSA0 on line
        LSAValue = LSA_Values[0];
      } else {  // 255 : out of the line
        if (lastLSA_Values[0] < 35) {
          LSAValue = -20;
        } else {
          LSAValue = 90;
        }
      }
      PWMCorrection = 0;
      // 0 degree wheel PID
      errorFWD = 35 - LSAValue;
      PWMCorrection = KpFWD * errorFWD - KdFWD * (errorFWD - prevErrorFWD); // FWD PWM Correction
      setMotorPWM(0, MOTOR_MAG[0] + PWMCorrection);
      prevErrorFWD = errorFWD;

      // Back wheels
      LSAValue = 0;
      if (LSA_Values[1] != 255) { // LSA1 on line
        LSAValue = LSA_Values[1];
      } else {  // 255 : out of the line
        if (lastLSA_Values[1] < 35) {
          LSAValue = -20;
        } else {
          LSAValue = 90;
        }
      }
      PWMCorrection = 0;
      // 180 degree wheels PID
      errorBWD = 35 - LSAValue;
      PWMCorrection = KpBWD * errorBWD - KdBWD * (errorBWD - prevErrorBWD);
      setMotorPWM(1, MOTOR_MAG[1] + PWMCorrection);
      setMotorPWM(2, MOTOR_MAG[2] + PWMCorrection);
      prevErrorBWD = errorBWD;

    }
};

ThreeWheelBotMotors motors;

// Motor properties
int MotorPWMs[3] = {6,7,5};   //6,7,5,8
int MotorDIRs[3] = {45,49,43};//45,49,43,47
boolean CCW_DIRS[3] = {HIGH, HIGH, HIGH};  // The voltages levels we need to give to pins to rotate in a counter clockwise manner [M0,M1,M2]

int LSA_PINs[3] = {A13,A15,A14};  // LSA 1,2 and 3 of the bot
int LSA_SerialSelectPins[3] = {22,24,26};
/*
  LSA Tristate buffer pins : 22, 24, 26
*/
HardwareSerial LSASerial = Serial2;
#define LSASerial_BAUD_RATE 38400
int LSA_Values[3] = {0,0,0};      // The Actual - Scaled down LSA values (0 to 70 and then 255)
int lastLSA_Values[3] = {0,0,0};  // The last (NON 255 - Legit) LSA Value
int LSA_Values_Raw[3] = {0,0,0};  // The raw value LSAs get on the analog pins (10 bit)

void setup() {
  Serial.begin(Serial_BAUD_RATE);
  // LSAs
  LSASerial.begin(LSASerial_BAUD_RATE);
  for (int i = 0 ; i < 3 ; i++) { // Configure LSA analog input pins
    pinMode(LSA_PINs[i], INPUT);
    pinMode(LSA_SerialSelectPins[i], OUTPUT);
    digitalWrite(LSA_SerialSelectPins[i], LOW);
    if (DEBUGGER) {
      Serial.print("LSA ");
      Serial.print(i);
      Serial.print(" recognised at ");
      Serial.println(LSA_PINs[i]);
    }
  }
  // Motors
  motors.attachDebuggerSerial(&Serial);
  if (DEBUGGER) {
    motors.enableDebugger();
  } else {
    motors.disableDebugger();
  }
  motors.attachPWM_DIRPins(MotorPWMs, MotorDIRs, CCW_DIRS);
  motors.setMaxPWM(255);
    // Motor PD parameters for forward and backward motion
    /*
      Magnitude : KpFWD,KdFWD; KpBWD, KdBWD (150 - 400)
      150 : 1.4,0.1;0.7,0.2
      250 : 2.5,0.7;1.5,0.2
      400 : 3.8,1.4;2.0,0.6

    */
  motors.setFWD_PDparameters(3.1,1.2);
  motors.setBWD_PDparameters(1.6,0.4);
}

int CURRENT_ANGLE = 0, CURRENT_PWM = 330;
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
  // Grab all 3 LSA values
  GrabLSAValues();
  DisplayLSAValues();
  // Move Bot
  // motors.moveAtWithAngle(CURRENT_PWM,CURRENT_ANGLE);
  motors.setOmega(120);
  /*
  motors.moveAtWithAngle(CURRENT_PWM, CURRENT_ANGLE);
  motors.applyXAxisMotionCorrection(LSA_Values,lastLSA_Values);
  // motors.setMotorStatus(0,0,0);
  motors.printMotorStatus();
  */
  // motors.applyXAxisMotionCorrection(LSA_Values,lastLSA_Values);
}

// LSA Codes
void GrabLSAValues_Analog() {  // Get values from the LSAs (Analog mode)
  // Currently, we're reading values from the analog ports
  for (int i = 0; i < 3 ; i++) {
    LSA_Values_Raw[i] = analogRead(LSA_PINs[i]);
    if (LSA_Values_Raw[i] <= 900) {
      LSA_Values[i] = map(LSA_Values_Raw[i],0,900,0,70);  // Mapping of the LSA values
      lastLSA_Values[i] = LSA_Values[i];  // Here is where we get the last LSA values
    } else {  // 255 case (out of line or no line found)
        LSA_Values[i] = 255;
    }
  }
}
void DisplayLSAValues() { // Print LSA Values
  if (DEBUGGER) {
    for (int i = 0 ; i < 3; i++) {
      Serial.print("\nLSA ");
      Serial.print(i);
      Serial.print(" : ");
      Serial.print(LSA_Values_Raw[i]);
      Serial.print("(");
      Serial.print(LSA_Values[i]);
      Serial.print(")");
    }
  }
}
void GrabLSAValues_Serial() {
  digitalWrite(LSA_SerialSelectPins[0], HIGH);
  if (DEBUGGER) {
    Serial.println(LSASerial.read());
  }
}
void GrabLSAValues() {
  GrabLSAValues_Analog();
}
