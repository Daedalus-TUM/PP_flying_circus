#include <SPI.h>
#include <Mirf.h>
#include <nRF24L01.h>
#include <MirfHardwareSpiDriver.h>

// NRF24 settings
#define RFADDR "rec1"
#define RFBASE "rec0"
#define RFCHANNEL 3

#define TADDR "t1"
#define TBASE "t0"

//besser als digitalWrite
#define CLR(x,y) (x&=(~(1<<y)))
#define SET(x,y) (x|=(1<<y))

byte Nummer[4]; 

void setup() {
  //Serial.begin(115200);
  //init NRF24
  Mirf.spi = &MirfHardwareSpi;
  Mirf.cePin = 9;
  Mirf.csnPin = 10;
  Mirf.init();
  Mirf.setRADDR((byte *)RFADDR);
  Mirf.setTADDR((byte *)TADDR);
  Mirf.payload = 4;
  Mirf.channel = RFCHANNEL;
  Mirf.config();
  
  Nummer[1] = 0;
}


void loop(){
  Mirf.getData(Nummer);
  if(Nummer[1] > 6)
    Nummer[1] = 0;
  Nummer[1] = Nummer[1] + 1;
  Mirf.send(Nummer);
  
  Serial.println(Nummer[1]);
  
}
  
  
