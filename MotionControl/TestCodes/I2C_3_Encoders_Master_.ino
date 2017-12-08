
#include <Wire.h>

void setup() 
{
  Wire.begin();        
  Serial.begin(9600);  
  Serial3.begin(9600);
}
char control = 'S';
long int value[4];
void loop() 
{ 
 /* if(Serial3.available())
  control = Serial3.read(); // To receive instruction from the Main conrol
  Serial.print(control); */
  
if(control == 'S')
{
 for(int i=8;i<11;i++)
 {
 Wire.beginTransmission(i);
 Wire.requestFrom(i, 4);    
  if(Wire.available()) 
  { 
   byte b4 = Wire.read(); 
   byte b3= Wire.read();
   byte b2= Wire.read();
   byte b1= Wire.read();
   value[i-8] = (b4<<24) | (b3<<16) | (b2<<8) | b1;    
   Serial.print("Encoder");Serial.print(i);Serial.print(" :");Serial.println(value[i-8]);
   
  }
  Wire.endTransmission();
 }  
}
//delay(1000);
}
 
