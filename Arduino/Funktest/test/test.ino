#include <SPI.h>
#include <Mirf.h>
#include <nRF24L01.h>
#include <MirfHardwareSpiDriver.h>

// NRF24 settings
#define RFADDR "alex1"
#define RFBASE "alex0"
#define RFCHANNEL 3

//besser als digitalWrite
#define CLR(x,y) (x&=(~(1<<y)))
#define SET(x,y) (x|=(1<<y))

byte Nummer; 

void setup() {
  //Serial.begin(115200);
  //init NRF24
  Mirf.spi = &MirfHardwareSpi;
  Mirf.cePin = 9;
  Mirf.csnPin = 10;
  Mirf.init();
  Mirf.setRADDR((byte *)RFADDR);
  Mirf.channel = RFCHANNEL;
  Mirf.config();
}


void loop(){
  Mirf.getData(Nummer);
  if(Nummer > 6)
    Nummer = 0;
  Nummer = Nummer + 1;
  Mirf.send(Nummer);
  
  Serial.println(Nummer);
  
}
  
  
