#include <SPI.h>
#include <Mirf.h>
#include <nRF24L01.h>
#include <MirfHardwareSpiDriver.h>

// NRF24 settings
#define RFADDR "Bloon"
#define RFCHANNEL 15

//besser als digitalWrite
#define CLR(x,y) (x&=(~(1<<y)))
#define SET(x,y) (x|=(1<<y))

byte Nummer[4]; 
byte ack[4] = {1};
byte check;

void setup() {
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
  Nummer[1] = 0;
  Nummer[2] = 0;
  Nummer[3] = 0;
}


void loop(){
  
  if(Mirf.dataReady())
  {
    Mirf.getData(Nummer);
     
    Mirf.send(ack);
    while(Mirf.isSending()){
    }
    Serial.println("Ack sent");
  }
  
  delay(500);
  
  if(check != Nummer[0])
  {
    Serial.print(Nummer[0], DEC);
    Serial.print(" ");
    Serial.print(Nummer[1], DEC);
    Serial.print(" ");
    Serial.print(Nummer[2], DEC);
    Serial.print(" ");
    Serial.println(Nummer[3], DEC);
          
    Mirf.send(Nummer);
    while(Mirf.isSending()){
    }
    
    Serial.println("Data sent");
    Serial.println();
  }
  check = Nummer[0];
  delay(100);
}
  
  
