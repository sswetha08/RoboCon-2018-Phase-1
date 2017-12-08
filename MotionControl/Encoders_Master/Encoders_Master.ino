#include <I2C_Anything.h>
#include<Wire.h>

//Motor Pins
int pwmPin[3] = {1, 3, 5};
int dirPin[3] = {2, 6, 7};

//Bot direction
int dir = 0, dirOpp = 1;

//PID
int kp[3] = {0, 0, 0};
int kd[3] = {0, 0, 0};
float p_error[3];
float d_error[3];
float pid_mag[3];
float prev_error[3] = {0};


//Wheel velocities,w
int idealVel[3];
int realVel[3];
float w[3];

// Time values (for dt)
long Time[3] = {0};
long oldTime[3] = {0};
long botTime = 0, oldBotTime = 0;

// Encoders
long encoderValue[3] = {0};
long oldEncoderValue[3] = {0};

// Bot speed nd distance specifics
float convert = 0; // Find conversion factor
float velSet, botVel;
float pwmSet = 60; // Desired pwm
int mag[3] = {0};
float distance = 0;
int ang = 0;
int setDistance = 0; // Distance need to be traversed
int r = 0.75; //wheel radius in metres


void setup()
{
  pwmToVel();
  setMotorPWM();
  idealVelocity();  //Setting motor PWMs
  Wire.begin();   //Start the I2C communication bus
  Serial.begin(9600);
}
void loop()
{
  moveMotors(); // Drives all motors  &Suggestion : Have this at the end of the loop, we want effectss after computation :  @ Avneesh Mishra

  // Read from all controllers
  for (int i = 1; i < 4; i++)
  {
    while (Wire.available())
    {
      oldEncoderValue[i - 1] = encoderValue[i - 1];
      oldTime[i - 1] = Time[i - 1];
      Wire.beginTransmission (i); // i is the slave address (interrupts @Avneesh Mishra)
      I2C_readAnything(encoderValue[i - 1]);
      Time[i] = millis();
      Serial.print(encoderValue[i - 1]);
    }
    // All encoder values received
  }

  calcw(); // calculate velocity and w for all wheels (calculate angular velocities)
  for (int i = 0; i < 3; i++)
  {
    do_pid(i);
  }
  for (int i = 0; i < 3; i++)
  {
    mag[i] += pid_mag[i];
  }

}


void calcw()
{
  botTime = millis();
  for (int i = 0; i < 3; i++)
  {
    w[i] = ((encoderValue[i] - oldEncoderValue[i]) / (Time[i] - oldTime[i])) * (360 / 400);   //w = dQ/dT : But, does it generate 400 pulses in a second though. And, the scaling factor is probably wrong. Make it 2 * PI / 400
    realVel[i] = w[i] * r;  //v = w * r (r -> radius of wheel)
  }

  // velocity of the bot #Check... @Avneesh Mishra
  botVel = sqrt(sq(realVel[0]) + sq(realVel[1]) + sq(realVel[2]) - (realVel[0] * realVel[1] + realVel[1] * realVel[2] + realVel[2] * realVel[0])); // where is angle ?

  //The distance bot has moved = v*dt
  distance += botVel * (botTime - oldBotTime);

  if (distance == setDistance)
  {
    clearMag();
    moveMotors();
    //Transfer control to throwing mechanism!
  }

  else if (distance >= setDistance) //if bot has overshot
  {
    int temp = dir; // change direction (swap dir and dirOpp)
    dir = dirOpp;   
    dirOpp = temp;
    setDistance = distance - setDistance; // reset distance
    distance = 0;
  }
  oldBotTime = botTime;
}

int do_pid(int i)         //Apply PID correction for the ith motor
{

  p_error[i] = idealVel[i] - realVel[i];
  d_error[i] = p_error[i] - prev_error[i];

  // Calculate the three components of the PID output.
  pid_mag[i] = kp[i] * p_error[i] + kd[i] * d_error[i];
  prev_error[i] = p_error[i];

}


void idealVelocity()    //Apply transformation matrix
{
  idealVel[0] = velSet * cos(2.6180 - ang);
  idealVel[1] = velSet * cos(0.5236 - ang);
  idealVel[2] = velSet * cos(4.7124 - ang);
}

void setMotorPWM()
{
  mag[0] = pwmSet * cos(2.6180 - ang);
  mag[1] = pwmSet * cos(0.5236 - ang);
  mag[2] = pwmSet * cos(4.7124 - ang);
}

void pwmToVel()
{
  velSet = pwmSet * convert;  //PWM to velocity conversion factor
}

void moveMotors()
{
  for (int i = 0; i < 3; i++)
  {
    analogWrite(pwmPin[i], abs(mag[i]));
    digitalWrite(dirPin[i], (mag[i] > 0) ? dir : dirOpp); // LOW:HIGH (Intially dir =0, dirOpp=1)
  }
}

void clearMag()
{
  for (int i = 0; i < 3; i++)
    mag[i] = 0;
}



