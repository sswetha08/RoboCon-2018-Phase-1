char hash;
void setup() 
{
  Serial1.begin(9600);
  Serial2.begin(9600);
}

void loop() 
{
  if(Serial2.available())
  {
    hash = Serial2.read();
    if(hash == "#bm")
    {
      if(Serial1.available())
      {
        Serial1.write("#bm");
      }
    }
    else if(hash == "#l")
    {
      if(Serial1.available())
      {
        Serial1.write("#l");
      }
    }
  }
}
