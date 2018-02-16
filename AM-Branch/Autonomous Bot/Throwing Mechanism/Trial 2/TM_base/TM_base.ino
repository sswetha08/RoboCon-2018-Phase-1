#include<Servo.h>

class ThrowingMechanism_Base {
  // Driving Motor
  int MOTOR_DIR_PIN;
  int MOTOR_PWM_PIN;
  boolean ThrowingVoltageLevel; // The voltage level that does an underarm throw
  // IR array for the arm
  int IR_array[8];
  // The supporting servo (flap)
  Servo supportFlap;
  int OPENING_ANGLE;
  int CLOSING_ANGLE;
  // Bluetooth module
  HardwareSerial BTSerial;
  SoftwareSerial BTSerial_software;


};

// Encoders
  // Digital pins to which the encoders are connected
#define ENCODER_TM_A 2
#define ENCODER_TM_B 3
  // Interrupt function
#define ENCODER_INTERRUPT_FUNCTION

// Servo
  // Arrach pins
#define SERVO_TM_SUPPORT_PIN 8
#define

int ENCODER_THRESHOLD[3] = {3900,0,0};
