#include <SPI.h>
#include <Mirf.h>
#include <nRF24L01.h>
#include <MirfHardwareSpiDriver.h>

// NRF24 settings
#define RFADDR "Bloon"
#define RFCHANNEL 30

//data arrays to be sent and received
float waypoint[3] = {0};
float send_data[3] = {1};

//checks
float check;

//timer used in the ack process: timeout restarts sending process
unsigned long timer = 0;
unsigned long interval = 2000;

//pointers to the data arrays
byte *sendbytes = (byte*)send_data;
byte *recbytes = (byte*)waypoint;


void setup() {
  Serial.begin(115200);
  //init NRF24
  Mirf.spi = &MirfHardwareSpi;
  Mirf.cePin = 21;
  Mirf.csnPin = 20;
  Mirf.init();
  Mirf.setRADDR((byte *)RFADDR);
  Mirf.payload = 12;
  Mirf.channel = RFCHANNEL;
  Mirf.config();
}


void loop(){
  
  if(Mirf.dataReady())
  {
    Mirf.getData(recbytes);
    
    //return part of incoming message as communication test
    send_data[2] = waypoint[0];
     
    Mirf.send(sendbytes);
    while(Mirf.isSending()){
    }
    Serial.println("Answer sent");
  
    Serial.println("Data received:");
    Serial.print(waypoint[0]);
    Serial.print(" ");
    Serial.print(waypoint[1]);
    Serial.print(" ");
    Serial.print(waypoint[2]);
    Serial.println();
    Serial.println();
    
  }
  check = waypoint[0];
  delay(200);
}
  
  
