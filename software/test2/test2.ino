#include <SPI.h>
#include <Mirf.h>
#include <nRF24L01.h>
#include <MirfHardwareSpiDriver.h>

// NRF24 settings
#define RFADDR "Bloon"
#define RFCHANNEL 30

/*data arrays to be sent and received
float waypoint[3] = {0};
float send_data[3] = {1};
*/
byte sending[4] = {0};
byte receiving[4] = {0};

//timer used in the ack process: timeout restarts sending process
unsigned long timer = 0;
unsigned long interval = 2000;

/*pointers to the data arrays
byte *sendbytes = (byte*)send_data;
byte *recbytes = (byte*)waypoint;
*/

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
}


void loop(){
  
  if(Mirf.dataReady())
  {
    Mirf.getData(receiving);
    
    //return part of incoming message as communication test
    sending[2] = receiving[0];
     
    Mirf.send(sending);
    while(Mirf.isSending()){
    }
    Serial.println("Answer sent");
  
    Serial.println("Data received:");
    Serial.print(receiving[0]);
    Serial.print(" ");
    Serial.print(receiving[1]);
    Serial.print(" ");
    Serial.print(receiving[2]);
    Serial.print(" ");
    Serial.print(receiving[3]);
    Serial.println();
    Serial.println();
    
  }
  delay(200);
}
  
  
