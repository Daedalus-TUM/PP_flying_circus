#include <SPI.h>
#include <Mirf.h>
#include <nRF24L01.h>
#include <MirfHardwareSpiDriver.h>

// NRF24 settings
#define RFADDR "Base"
#define RFCHANNEL 15

byte Nummer[4];
byte Rec[4];
byte check1;
byte check2;

void setup()
{
  Serial.begin(115200);
  //init NRF24
  Mirf.spi = &MirfHardwareSpi;
  Mirf.cePin = 9;
  Mirf.csnPin = 10;
  Mirf.init();
  Mirf.setRADDR((byte *)RFADDR);
  Mirf.payload = 4;
  Mirf.channel = RFCHANNEL;
  Mirf.config();
  
  Nummer[0] = 0;
  Nummer[1] = 4;
  Nummer[2] = 8;
  Nummer[3] = 12;
  
  Rec[0] = 0;
  Rec[1] = 0;
  Rec[2] = 0;
  Rec[3] = 0;
}


// send and repeat data until ack
// save ack data in array ack, ack[0] != 2 signals correct exchange
void consend(byte data[4])
{
  byte ack[4] = {0};
  ack[0] = 2;
  
    
  Mirf.send(data);
  while(Mirf.isSending()){
  }
    
  Serial.println("Sending...");
  
  if(Mirf.dataReady())
  {
    Mirf.getData(ack);
    Serial.println("Received Ack");
  }
}



void loop()
{
  
  // read char from console:
        if (Serial.available() > 0) 
        {
                // read the incoming byte:
                Nummer[0] = Serial.read();

                // say what you got:
                Serial.println("I send: ");
                
                Serial.print(Nummer[0], DEC);
                Serial.print(" ");
                Serial.print(Nummer[1], DEC);
                Serial.print(" ");
                Serial.print(Nummer[2], DEC);
                Serial.print(" ");
                Serial.println(Nummer[3], DEC);
        }
        
  //send char
  if(Nummer[0] != check1)
    consend(Nummer);
    
  //receive without ack
  if(Mirf.dataReady())
  {
    Mirf.getData(Rec);
    if(Rec[0]!=check2)
    {
      Serial.println("Received:");
      for(int i=0; i<4; i++)
      {
        Serial.print(Rec[i]);
        Serial.print(" ");
      }
      Serial.println();
    }
  }
  Nummer[0] = check1;
  Rec[0] = check2;
  delay(500);
}
  
  
