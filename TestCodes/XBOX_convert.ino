/*
  Example sketch for the Xbox 360 USB library - developed by Kristian Lauszus
  For more information visit my blog: http://blog.tkjelectronics.dk/ or
  send me an e-mail:  kristianl@tkjelectronics.com
*/

#include <XBOXUSB.h>

// Satisfy the IDE, which needs to see the include statment in the ino too.
#ifdef dobogusinclude
#include <spi4teensy3.h>
#endif
#include <SPI.h>

USB Usb;
XBOXUSB Xbox(&Usb);

int value, LeftX, LeftY, prevLeftY = 0, prevLeftX = 0;

void setup() {
  Serial.begin(115200);
#if !defined(__MIPSEL__)
  while (!Serial); // Wait for serial port to connect - used on Leonardo, Teensy and other boards with built-in USB CDC serial connection
#endif
  if (Usb.Init() == -1) {
    Serial.print(F("\r\nOSC did not start"));
    while (1); //halt
  }
  Serial.print(F("\r\nXBOX USB Library Started"));
}
void loop() {
  Usb.Task();
  if (Xbox.Xbox360Connected)
  {
    if (Xbox.getAnalogHat(RightHatX) > 8500 || Xbox.getAnalogHat(RightHatX) < -8500 || Xbox.getAnalogHat(RightHatY) > 8500 || Xbox.getAnalogHat(RightHatY) < -8500 )
    {
      if (Xbox.getAnalogHat(RightHatX) > 8500 || Xbox.getAnalogHat(RightHatX) < -8500)
      {
        value = Xbox.getAnalogHat(RightHatX);
        if (value > 0)
          LeftX = map(value, 8500, 32786, 0, 128);

        if (value < 0)
          LeftX = -( map(-value, 8500, 32786, 0, 128));
        if (LeftX == 203 || LeftX==217)
          LeftX = -127;

        Serial.print(LeftX); Serial.print(';');
        // LeftX=prevLeftX;
      }
      else
      {
        LeftX = 0;
        Serial.print(LeftX); Serial.print(';');
      }
      if (Xbox.getAnalogHat(RightHatY) > 8500 || Xbox.getAnalogHat(RightHatY) < -8500)
      {
        value = Xbox.getAnalogHat(RightHatY);
        if (value > 0)
          LeftY = map(value, 8500, 32786, 0, 128);

        if (value < 0)
          LeftY = -( map(-value, 8500, 32786, 0, 128));

        if (LeftY == 203 || LeftY ==217)
          LeftY = -127;
        Serial.println(LeftY);
        //prevLeftY = LeftY;
      }
      else
      {
        LeftY = 0;

        Serial.println(LeftY);
      }
    }
    else
    {
      LeftX=0;LeftY=0;
    }

  }
//delay(200);
}
