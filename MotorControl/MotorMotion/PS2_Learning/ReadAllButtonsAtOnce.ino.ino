#include <Cytron_PS2Shield.h>

Cytron_PS2Shield PS2(2, 3); //2 - Rx and 3 - Tx

void setup() {
  // put your setup code here, to run once:
  PS2.begin(57600);
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  PS2.readAllButton();  
  //The data is now stored in PS2.ps_data[]. Check the byte architecture to know more...
  Serial.print(PS2.ps_data[5], BIN);
  Serial.print(" ");
  Serial.println(PS2.readButton(PS2_JOYSTICK_LEFT_Y_AXIS));
}

/*
   The Byte architecture
   Data stored in ps_data[6] which is inside 'Cytron_PS2Shield.h'
    -ps_data[0] : (Byte 1 to 8)
      PS2_SELECT,
      PS2_JOYSTICK_LEFT,
      PS2_JOYSTICK_RIGHT,
      PS2_START,
      PS2_UP,
      PS2_RIGHT,
      PS2_DOWN,
      PS2_LEFT
    -ps_data[1] : (Byte 1 to 8)
      PS2_LEFT_2,
      PS2_RIGHT_2,
      PS2_LEFT_1,
      PS2_RIGHT_1,
      PS2_TRIANGLE,
      PS2_CIRCLE,
      PS2_CROSS,
      PS2_SQUARE
    -ps_data[2] : (Byte 1 to 8)
      PS2_JOYSTICK_RIGHT_X_AXIS : 128 when left idle
    -ps_data[3] : (Byte 1 to 8)
      PS2_JOYSTICK_RIGHT_Y_AXIS : 127 when left idle
    -ps_data[4] : (Byte 1 to 8)
      PS2_JOYSTICK_LEFT_X_AXIS : 144 when left idle
    -ps_data[5] : (Byte 1 to 8)
      PS2_JOYSTICK_LEFT_Y_AXIS : 127 when left idle
*/
