// I2C SRF10 or SRF08 Devantech Ultrasonic Ranger Finder
// by Nicholas Zambetti <http://www.zambetti.com>
// and James Tichenor <http://www.jamestichenor.net>

// Demonstrates use of the Wire library reading data from the
// Devantech Utrasonic Rangers SFR08 and SFR10

// Created 29 April 2006

// This example code is in the public domain.


#include <Wire.h>

void setup()
{
  Wire.begin();                // join i2c bus (address optional for master)
  Serial.begin(38400);          // start serial communication at 9600bps
}

int reading = 0, i=0;     //Hoehenrechnung
float hoehe_u[10];
float HOEHE_U=0, temp=0;

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

  // step 2: wait for readings to happen
  delay(70);                   // datasheet suggests at least 65 milliseconds

  // step 3: instruct sensor to return a particular echo reading
  Wire.beginTransmission(114); // transmit to device #112
  Wire.write(byte(0x02));      // sets register pointer to echo #1 register (0x02)
  Wire.endTransmission();      // stop transmitting

  // step 4: request reading from sensor
  Wire.requestFrom(114, 2);    // request 2 bytes from slave device #114

  // step 5: receive reading from sensor
  if(2 <= Wire.available())    // if two bytes were received
  {
    reading = Wire.read();  // receive high byte (overwrites previous reading)
    reading = reading << 8;    // shift high byte to be high 8 bits
    reading |= Wire.read(); // receive low byte as lower 8 bits
    Serial.println(reading);   // print the reading
    
    hoehe_u[i]=reading;
    i=i+1;
   
    
  }
  
  for(int l=0; l<10; l++){     
      temp=temp + hoehe_u[l];}
  HOEHE_U=temp/10;
    
   
   
  Serial.println("Hoehe:");
  Serial.println(HOEHE_U);
  Serial.println("");
  
  
 // delay(250);                  // wait a bit since people have to read the output :)
}


/*

// The following code changes the address of a Devantech Ultrasonic Range Finder (SRF10 or SRF08)
// usage: changeAddress(0x70, 0xE6);

void changeAddress(byte oldAddress, byte newAddress)
{
  Wire.beginTransmission(112);
  Wire.write(byte(0x00));
  Wire.write(byte(0xA0));
  Wire.endTransmission();

  Wire.beginTransmission(o);
  Wire.write(byte(0x00));
  Wire.write(byte(0xAA));
  Wire.endTransmission();

  Wire.beginTransmission(oldAddress);
  Wire.write(byte(0x00));
  Wire.write(byte(0xA5));
  Wire.endTransmission();

  Wire.beginTransmission(114);
  Wire.write(byte(0x00));
  Wire.write(112);
  Wire.endTransmission();
}
*/
