#define RED_PIN 2
#define GREEN_PIN 3
#define BLUE_PIN 4

#define COMMON_CATHODE 0
#define COMMON_ANODE 1

int CONNECTION_MODE = COMMON_ANODE;

#define COMPLEMENTARY 0
#define ORIGINAL 1

int MODE = ORIGINAL;

int Brightness[3] = {255, 255, 255};

void setup() {
  for (int i = 0 ; i < 3 ; i++)
    Brightness[i] = CONNECTION_MODE * 255;

  pinMode(RED_PIN, OUTPUT);
  pinMode(GREEN_PIN, OUTPUT);
  pinMode(BLUE_PIN, OUTPUT);

  analogWrite(RED_PIN, Brightness[0]);
  analogWrite(GREEN_PIN, Brightness[1]);
  analogWrite(BLUE_PIN, Brightness[2]);
  Serial.begin(9600);
}

void loop() {
  while (Serial.available()) {
    int a = 0;
    char c = Serial.read();
    switch (c) {
      case 'R':
        a = Serial.parseInt();
        Brightness[0] = constrain(a, 0, 255) * ((CONNECTION_MODE == COMMON_ANODE)? -1 : 1) + ((CONNECTION_MODE == COMMON_ANODE)? 255 : 0);
        break;
      case 'G':
        a = Serial.parseInt();
        Brightness[1] = constrain(a, 0, 255) * ((CONNECTION_MODE == COMMON_ANODE)? -1 : 1) + ((CONNECTION_MODE == COMMON_ANODE)? 255 : 0);
        break;
      case 'B':
        a = Serial.parseInt();
        Brightness[2] = constrain(a, 0, 255) * ((CONNECTION_MODE == COMMON_ANODE)? -1 : 1) + ((CONNECTION_MODE == COMMON_ANODE)? 255 : 0);
        break;
      case '~':
        MODE = COMPLEMENTARY;
        Generate_Complimentary_RGB(Brightness);
        break;
      case 'o':
        MODE = ORIGINAL;
        
        break;
      case 'C':
        CONNECTION_MODE = COMMON_CATHODE;
        break;
      case 'A':
        CONNECTION_MODE = COMMON_ANODE;
        break;
    }
    if (c == 'C' || c == 'A') {
      for (int i = 0 ; i < 3 ; i++)
        Brightness[i] = CONNECTION_MODE * 255;
    }
    //c = Serial.read();
  }
  analogWrite(RED_PIN, Brightness[0]);
  analogWrite(GREEN_PIN, Brightness[1]);
  analogWrite(BLUE_PIN, Brightness[2]);
  Serial.print(Brightness[0]);
  Serial.print("-");
  Serial.print(Brightness[1]);
  Serial.print("-");
  Serial.print(Brightness[2]);
  Serial.println("");
}

void Generate_Complimentary_RGB(int *Brightness) {
  int MIN = min(min(Brightness[0],Brightness[1]),Brightness[2]),MAX = max(max(Brightness[0],Brightness[1]),Brightness[2]);
  int SUM = MIN + MAX;
  for (int i = 0; i < 3 ; i ++) {
    Brightness[i] = SUM - Brightness[i];
  }
}

