#define LSA1 A13
#define LSA2 A15
#define LSA3 A14

/*
*   NOTES
NORTH is the NORTHBRIDGE of wires
LSA2 ---- A15 ----- EAST
LSA1 ---- A13 ----- SOUTH
LSA3 ---- A14 ----- WEST
LSAs give (analogRead Vs LSA Display Value)
0 for 0
450 for 35
900 for 70
*/


void setup() {
  Serial.begin(9600);

  pinMode(LSA1, INPUT);
  pinMode(LSA2, INPUT);
  pinMode(LSA3, INPUT);
}

void loop() {
  /*  // Uncomment for all LSA Value Mode
  Serial.print("LSA 1 : ");
  Serial.println(analogRead(LSA1));
  Serial.print("LSA 2 : ");
  Serial.println(analogRead(LSA2));
  Serial.print("LSA 3 : ");
  Serial.println(analogRead(LSA3));
  */

  
  Serial.print("LSA 1 : ");
  Serial.println(analogRead(LSA1));

}
