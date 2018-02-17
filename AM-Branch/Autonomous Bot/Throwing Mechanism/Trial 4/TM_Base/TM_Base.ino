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
  retractFlap();
  openArmGripper();
  digitalWrite(MOTOR_DIR_PIN, HIGH);
}

void resetEncoderCount() {
  encoderCount = 0;
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
  resetEncoderCount();
  analogWrite(MOTOR_PWM_PIN, 10);
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
}
void closeArmGripper() {  // Close the arm gripper
  BTSerial.write('y');
}

void loop() {
  if (tempEncoderCount != encoderCount) {
    if (DEBUGGER_MODE) {
      printEncoderTicks();
    }
    tempEncoderCount = encoderCount;
  }
  if(Serial.available()) {
    char c = Serial.read();
    int PWM;
    boolean dir;
    int del;
    boolean shuttlecockTransferComplete;
    switch(c) {
      case 'r':case 'R':  // Only the deploy position
        PerformFlapServoDeployAction();
        openArmGripper();
        resetEncoderCount();
      break;
      case 't':case 'T':  // Throwing point set
        del = Serial.parseInt();  // The count threshold
        if (DEBUGGER_MODE) {
          Serial.print("Encoder and gripper threshold set to ");
          Serial.println(del);
        }
        while(encoderCount < del) {
          printEncoderTicks();
        }
        openArmGripper();
      break;
      case 'o':case 'O':  // Open gripper arm
        if (DEBUGGER_MODE) {
          Serial.println("Opening the arm gripper");
        }
        openArmGripper();
      break;
      case 'c':case 'C':  // Close gripper arm
        if (DEBUGGER_MODE) {
          Serial.println("Closing the arm gripper");
        }
        closeArmGripper();
      break;
      case 'm':case 'M':  // Motor actuate : DEFAULT (throwing direction)
        if (DEBUGGER_MODE) {
          Serial.println("Enter the PWM : ");
        }
        retractFlap();
        // az1500m110t2730z100m0
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
      case 'f':case 'F':  // Retract flap
        retractFlap();
      break;
      case 'g':case 'G':  // Deploy flap servo
        deployFlap();
      break;
      case 'z':case 'Z':  // Delay for the specified millis
        if (DEBUGGER_MODE) {
          Serial.println("Enter the amount of milliseconds for delay : ");
        }
        del = Serial.parseInt();
        delay(del);
      break;
      case 'e':case 'E': case 'a': case 'A': // Perform the entire process of receiving
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
              resetEncoderCount();
              shuttlecockTransferComplete = true;
              break;
            }
            if (digitalRead(PHOTOELECTRIC_SENSOR_PIN) == LOW) {
              if (DEBUGGER_MODE) {
                Serial.println("Lost shuttlecock, retrying...");
              }
              break;
            }
          }
        }
        if (DEBUGGER_MODE) {
          Serial.println("Transfer completed");
        }
        resetEncoderCount(); // Reference encoder count
      break;
      default:
      if (DEBUGGER_MODE) {
        Serial.println("Invalid input");
      }
      break;
    }
  }
}
