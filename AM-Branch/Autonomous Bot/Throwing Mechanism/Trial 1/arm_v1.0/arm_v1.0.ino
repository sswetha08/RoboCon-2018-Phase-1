#include <Servo.h>
#include <SoftwareSerial.h>
SoftwareSerial btserial(11, 10); //RX,TX
Servo ser;
int close_pos = 125;
int open_pos = 40;
int flag = 0;

int IR_recv = 2;

void setup()
{
  //Serial.begin(9600);
  //Serial.println("Hello World");
  btserial.begin(38400);
  pinMode(13, OUTPUT);
  pinMode(IR_recv, INPUT);
  digitalWrite(13, LOW);
  ser.attach(9);
  ser.write(open_pos);
  delay(1000);
  ser.write(close_pos);
  attachInterrupt(digitalPinToInterrupt(IR_recv), new_ser, FALLING);
}

void new_ser()
{
  flag = 1;
  ser.write(open_pos);
}

void loop()
{
  if (flag == 1) {
    btserial.write('K');
    flag = 0;
  }
  if (btserial.available())
  {
    char c = btserial.read();
    //Serial.println(c);
    if (c == 'x')
    {
      ser.write(open_pos);
      digitalWrite(13, HIGH);
    }
    else if (c == 'y')
    {
      ser.write(close_pos);
      digitalWrite(13, LOW);
    }
  }
}
