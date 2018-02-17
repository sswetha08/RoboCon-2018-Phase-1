#define r 10
#define g 11
#define b 12
#define S0 48
#define S1 46
#define s2 44
#define s3 42
#define LED 52
#define sensorOut 50
double rfrequency = 0;
double gfrequency = 0;
double bfrequency = 0;
char stat = 'A';
double var_lg, var_dg, var_red;

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
  Serial.print("R= ");//printing name
  Serial.print(rfrequency);//printing RED color frequency
  Serial.print("  ");
  Serial.print("G= ");//printing name
  Serial.print(gfrequency);//printing RED color frequency
  Serial.print("  ");
  Serial.print("B= ");//printing name
  Serial.print(bfrequency);//printing RED color frequency
  Serial.println("  ");

  /*dr=rfrequency-60;
    dg=gfrequency-73;
    db=bfrequency-147;
    var_lg=sqrt(dr*dr+db*db+dg*dg);
    dr=rfrequency-147;
    dg=gfrequency-149;
    db=bfrequency-176;
    var_dg=sqrt(dr*dr+db*db+dg*dg);*/
  dr = rfrequency - 109;
  dg = gfrequency - 476;
  db = bfrequency - 397;
  var_red = sqrt(dr * dr + db * db + dg * dg);
}



void setup()
{
  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(s2, OUTPUT);
  pinMode(s3, OUTPUT);
  pinMode(r, OUTPUT);
  pinMode(g, OUTPUT);
  pinMode(b, OUTPUT);
  pinMode(sensorOut, INPUT);
  pinMode(LED, OUTPUT);
  digitalWrite(LED, HIGH); // Setting frequency-scaling to 20%
  digitalWrite(S0, HIGH);
  digitalWrite(S1, LOW);
  Serial.begin(9600);
}

void loop()
{
  stat = 'A';
  RGB_Sensor_Response();
  if (var_red < 50)
    stat = 'R';
  if (stat == 'R')
    Serial.println("Red");
}
