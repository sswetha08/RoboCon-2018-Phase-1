int S0 = 48;
int S1 = 46;
int S2 = 44;
int S3 = 42;
int LED = 52;
int sensorOut = 50; 
long rfrequency = 0;
long gfrequency = 0;
long bfrequency = 0;
long r=0,g=0,b=0;
int cnt=0;

void setup() {
  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);
  pinMode(sensorOut, INPUT);
  pinMode(LED, OUTPUT);
  digitalWrite(LED, HIGH);  // Setting frequency-scaling to 20%
  digitalWrite(S0,HIGH);
  digitalWrite(S1,LOW);
  Serial.begin(9600);
}

void loop() 
{
  Serial.print(cnt);
  Serial.print("  ");
  
  // Setting red filtered photodiodes to be read
  digitalWrite(S2,LOW);
  digitalWrite(S3,LOW);
  // Reading the output frequency
  rfrequency = pulseIn(sensorOut, LOW);
  Serial.print("R= ");//printing name
  Serial.print(rfrequency);//printing RED color frequency
  Serial.print("  ");
  //delay(100);
  
  // Setting Green filtered photodiodes to be read
  digitalWrite(S2,HIGH);
  digitalWrite(S3,HIGH);
  // Reading the output frequency
  gfrequency = pulseIn(sensorOut, LOW);
  Serial.print("G= ");//printing name
  Serial.print(gfrequency);//printing RED color frequency
  Serial.print("  ");
  //delay(100);
  
  // Setting Blue filtered photodiodes to be read
  digitalWrite(S2,LOW);
  digitalWrite(S3,HIGH);
  // Reading the output frequency
  bfrequency = pulseIn(sensorOut, LOW);
  Serial.print("B= ");//printing name
  Serial.print(bfrequency);//printing RED color frequency
  Serial.println("  ");
  //delay(100);
  
  
  if(cnt%100==0&&cnt!=0)
  {  
   r/=100;
   g/=100;
   b/=100;   
   Serial.print("R=");
   Serial.println(r);
   Serial.print("G=");
   Serial.println(g);
   Serial.print("B=");
   Serial.println(b);   
   delay(10000);
   r=g=b=0;
  }
  else
  {
   r+=rfrequency;
   g+=gfrequency;
   b+=bfrequency;
  }
  cnt++;
  rfrequency=0;
  gfrequency=0;
  bfrequency=0;
} 

/*
void setColor(int redValue, int greenValue, int blueValue) {
  analogWrite(redPin, redValue);
  analogWrite(greenPin, greenValue);
  analogWrite(bluePin, blueValue);
}*/
