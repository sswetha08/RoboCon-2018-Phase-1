

byte input[2]; 
int val1;

void setup() {

  Serial2.begin(57600);
  Serial.begin(57600);

  Serial.flush();
  Serial2.flush();


 /* Serial1.readBytes(input, 4);
  val1 = (input[0] << 8 | input[1]);
  val2 = (input[2] << 8 | input[3]);
  initial = val1; */

}



void loop()
{
     Serial2.write("#f");

  while (Serial2.available())
   {
    Serial2.readBytes(input, 2);
    val1 = (input[0] << 8 | input[1]);
  
  }
 // if(val1<0)
  //val1= 180+(180+val1);
  Serial.print("Real : ");
    Serial.println(val1);

  delay(40);
}


