#define S0 48
#define S1 46
#define s2 44
#define s3 42
#define LED 52
#define sensorOut 50
double rfrequency = 0;
double gfrequency = 0;
double bfrequency = 0;
double var_red = 0;

void RGB_Sensor_Out(int s_2, int s_3)
{
  digitalWrite(s2, s_2);
  digitalWrite(s3, s_3);
}

void RGB_Sensor_Response()
{
  double dr, dg, db;
  RGB_Sensor_Out(0, 0);
  rfrequency = pulseIn(sensorOut, LOW);
  RGB_Sensor_Out(1, 1);
  gfrequency = pulseIn(sensorOut, LOW);
  RGB_Sensor_Out(0, 1);
  bfrequency = pulseIn(sensorOut, LOW);
  dr = rfrequency - 118.38;
  dg = gfrequency - 474.8;
  db = bfrequency - 399.73;
  var_red = sqrt(dr * dr + db * db + dg * dg);

}

void setup()
{
  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(s2, OUTPUT);
  pinMode(s3, OUTPUT);
  pinMode(sensorOut, INPUT);
  pinMode(LED, OUTPUT);
  digitalWrite(LED, HIGH); // Setting frequency-scaling to 20%
  digitalWrite(S0, HIGH);
  digitalWrite(S1, LOW);
  Serial.begin(9600);
}

void loop() 
{
  RGB_Sensor_Response();
  if (var_red < 50)
  {
    Serial.println("RED");
  }
}
