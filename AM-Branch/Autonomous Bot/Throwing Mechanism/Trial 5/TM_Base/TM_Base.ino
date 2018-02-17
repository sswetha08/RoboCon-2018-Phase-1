#define ENCODER_A 2
#define ENCODER_B 3
#define PHOTOELECTRIC_SENSOR_PIN 4
#define BTSerial Serial3
#define BTSerial_BAUD_RATE 38400
#define FLAP_SERVO_PIN 10
#define FLAP_DEPLOY_ANGLE 10
#define FLAP_RETRACT_ANGLE 90
#define MOTOR_PWM_PIN 9
#define MOTOR_DIR_PIN 8
#define MOTOR_THROW_VOLTAGE_LEVEL HIGH
#define ARM_THRESHOLD_TIME_LIMIT 60

#define DEBUGGER_MODE true

int RF_Module[4] = {23};

#include<Servo.h>

volatile long encoderCount = 0, tempEncoderCount = 0;
Servo flapServo;

void setup() {
  // Serial
  Serial.begin(9600);
  BTSerial.begin(BTSerial_BAUD_RATE);
  // Encoder
  pinMode(ENCODER_A, INPUT_PULLUP);
  pinMode(ENCODER_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A), encoderTicked, RISING);
  // Flap servo
  flapServo.attach(FLAP_SERVO_PIN);
  // Motor pins
  pinMode(MOTOR_DIR_PIN, OUTPUT);
  pinMode(MOTOR_PWM_PIN, OUTPUT);
  // RF Module
  for (int i = 0;i < 4; i++) {
    pinMode(RF_Module[i], OUTPUT);
  }
  retractFlap();
  openArmGripper();
  digitalWrite(MOTOR_DIR_PIN, HIGH);
  analogWrite(MOTOR_PWM_PIN, 30);
}

void printEncoderTicks() {
  if (DEBUGGER_MODE) {
    Serial.print("\nEncoder Ticks : ");
    Serial.println(encoderCount);
  }
}
void encoderTicked() {
  if (digitalRead(ENCODER_B) == LOW) {
    encoderCount--;
  } else {
    encoderCount++;
  }
}
void PerformFlapServoDeployAction() { // Perform the receiving action
  digitalWrite(MOTOR_DIR_PIN, !MOTOR_THROW_VOLTAGE_LEVEL);  // Reverse the direction. Rotate in the CCW when seen from LSA 3 side
  analogWrite(MOTOR_PWM_PIN, 25); // Rotate
  while(digitalRead(PHOTOELECTRIC_SENSOR_PIN) == LOW) {
    if (DEBUGGER_MODE) {
      Serial.println("Waiting for the sensor to detect arm's long end");
    }
  }
  // Sensor read HIGH here
  long prevEncoderCount = encoderCount;
  encoderCount = 400; // Set a buffer, this number is going to decrease
  if (DEBUGGER_MODE) {
    Serial.println("Arm's long end detected and crossed, reading encoders (count buffer = 400)");
  }
  while (encoderCount >= 300) {
    printEncoderTicks();  // Waiting for the encoder to show a quarter circle has been completed
    if (DEBUGGER_MODE) {
      Serial.println("Waiting for quarter circle to complete");
    }
  }
  // Deploy flap servo
  deployFlap();
  // Reduce the PWM of the arm
  analogWrite(MOTOR_PWM_PIN, 15);
}
void deployFlap() {
  if (DEBUGGER_MODE) {
    Serial.println("Deployed flap");
  }
  flapServo.write(FLAP_DEPLOY_ANGLE);
}
void retractFlap() {
  if (DEBUGGER_MODE) {
    Serial.println("Flap retracted");
  }
  flapServo.write(FLAP_RETRACT_ANGLE);
}
void openArmGripper() { // Open the arm gripper
  BTSerial.write('x');
  digitalWrite(RF_Module[0], HIGH);
}
void closeArmGripper() {  // Close the arm gripper
  BTSerial.write('y');
  digitalWrite(RF_Module[0], LOW);
}

void loop() {
  if (tempEncoderCount != encoderCount) {
    if (DEBUGGER_MODE) {
      Serial.print("Encoder : ");
      Serial.println(encoderCount);
    }
    tempEncoderCount = encoderCount;
  }
  if(Serial.available()) {
    char c = Serial.read();
    int PWM;
    boolean dir;
    boolean shuttlecockTransferComplete;
    switch(c) {
      case 'r':case 'R':
        PerformFlapServoDeployAction();
        openArmGripper(); // Wait for receiving shuttlecock
      break;
      case 'o':case 'O':
      if (DEBUGGER_MODE) {
        Serial.println("Opening the arm gripper");
      }
        openArmGripper();
      break;
      case 'c':case 'C':
      if (DEBUGGER_MODE) {
        Serial.println("Closing the arm gripper");
      }
        closeArmGripper();
      break;
      case 'm':case 'M':  // Motor actuate : DEFAULT (throwing direction)
        if (DEBUGGER_MODE) {
          Serial.println("Enter the PWM : ");
        }
        PWM = Serial.parseInt();
        digitalWrite(MOTOR_DIR_PIN, MOTOR_THROW_VOLTAGE_LEVEL);
        analogWrite(MOTOR_PWM_PIN, PWM);
        if (DEBUGGER_MODE) {
          Serial.print("Motor PWM is now : ");
          Serial.println(PWM);
        }
      break;
      case 'd':case 'D':  // Change motor direction
        if (DEBUGGER_MODE) {
          Serial.println("Enter the direction : ");
        }
        c = Serial.read();
        if (c == '1') {
          digitalWrite(MOTOR_DIR_PIN, HIGH);
        } else if (c == '0') {
          digitalWrite(MOTOR_DIR_PIN, LOW);
        }
        digitalWrite(MOTOR_DIR_PIN, dir);
      break;
      case 'f':case 'F':
        retractFlap();
      break;
      case 'g':case 'G':
        deployFlap();
      break;
      case 'e':case 'E': case 'a': case 'A': // Perform the entire process
        PerformFlapServoDeployAction();
        openArmGripper();
        shuttlecockTransferComplete = false;
        while(!shuttlecockTransferComplete) { // Wait for the transfer to be successfull
          for (int i = 1 ; i <= ARM_THRESHOLD_TIME_LIMIT; i++) {
            delayMicroseconds(10); // 1ms low
            if (DEBUGGER_MODE) {
              Serial.print("Duration : ");
              Serial.print(i);
              Serial.print("\t Left duration : ");
              Serial.println(ARM_THRESHOLD_TIME_LIMIT-i);
            }
            if (digitalRead(PHOTOELECTRIC_SENSOR_PIN) == HIGH && i == ARM_THRESHOLD_TIME_LIMIT) {
              if (DEBUGGER_MODE) {
                Serial.println("Shuttlecock loaded for deploy");
              }
              closeArmGripper();
              retractFlap();
              encoderCount = 0;
              shuttlecockTransferComplete = true;
              break;
            }
            if (digitalRead(PHOTOELECTRIC_SENSOR_PIN) == LOW) {
              if (DEBUGGER_MODE) {
                Serial.println("Lost shuttlecock, LOL retrying...");
              }
              break;
            }
          }
        }
        if (DEBUGGER_MODE) {
          Serial.println("Transfer completed");
        }
      break;
      default:
      if (DEBUGGER_MODE) {
        Serial.println("Invalid input");
      }
      break;
    }
  }
}
