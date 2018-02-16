#include <Servo.h>
#include <SoftwareSerial.h>
SoftwareSerial btserial(11,10); //RX,TX
Servo ser;
int p_sens = 0;  //Digitalpin 2
int close_pos=90 ;
int open_pos=20;
int flag=0;



void setup()
{
  Serial.begin(9600);
  btserial.begin(38400);
  ser.attach(9);
  ser.write(open_pos);
  delay(1000);
  ser.write(close_pos);
  attachInterrupt(p_sens, new_ser, RISING);
}

void new_ser()
{
    Serial.println("WORKED!");
    flag=1;
}

void loop()
{
 if (flag == 1) {
  btserial.write('b'); 
  flag = 0;
 }
 if(btserial.available())
 {
 char c = btserial.read();
 if(c=='x')
 {
  ser.write(open_pos);
 }
 else if (c=='y')
 {
  ser.write(close_pos);
 }
 }
}
  
  

