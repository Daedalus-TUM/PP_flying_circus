#include <Wire.h>


void setup()
{
  Wire.begin();                // join i2c bus (address optional for master)
}

void loop()
{ if (i>9){                    //Schleife für Buffer-Overlapping
  i=0;}
  temp=0;
  // step 1: instruct sensor to read echoes
  Wire.beginTransmission(114); // transmit to device #112 (0x70)
                               // the address specified in the datasheet is 224 (0xE0)
                               // but i2c adressing uses the high 7 bits so it's 112
  Wire.write(byte(0x00));      // sets register pointer to the command register (0x00)  
  Wire.write(byte(0x51));      // command sensor to measure in "inches" (0x50)
                               // use 0x51 for centimeters
                               // use 0x52 for ping microseconds
  Wire.endTransmission();      // stop transmitting

}

