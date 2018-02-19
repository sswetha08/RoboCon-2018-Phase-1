#define S0 48
#define S1 46
#define S2 44
#define S3 42
#define LED 52
#define sensorOut 50
int rfrequency = 0;
int gfrequency = 0;
int bfrequency = 0;
int r,g,b;
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
  
  // Setting Green filtered photodiodes to be read
  digitalWrite(S2,HIGH);
  digitalWrite(S3,HIGH);
  // Reading the output frequency
  gfrequency = pulseIn(sensorOut, LOW);
  Serial.print("G= ");//printing name
  Serial.print(gfrequency);//printing RED color frequency
  Serial.print("  ");
  
  // Setting Blue filtered photodiodes to be read
  digitalWrite(S2,LOW);
  digitalWrite(S3,HIGH);
  // Reading the output frequency
  bfrequency = pulseIn(sensorOut, LOW);
  Serial.print("B= ");//printing name
  Serial.print(bfrequency);//printing RED color frequency
  Serial.println("  ");
  
  if(cnt%100!=0)
  {
   r+=rfrequency;
   g+=gfrequency;
   b+=bfrequency;
  }
  else
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
   delay(100000);
   r=0;
   g=0;
   b=0;
  }
  cnt++;
  rfrequency=0;
  gfrequency=0;
  bfrequency=0;
} 
