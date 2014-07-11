#include <SPI.h>
#include <Mirf.h>
#include <nRF24L01.h>
#include <MirfHardwareSpiDriver.h>
#include <stdio.h>
#include <stdlib.h>

// NRF24 settings
#define RFADDR "Base"
#define RFCHANNEL 30

/*data arrays to be sent and received
float waypoint[3] = {0};
float rec_data[3] = {0};
*/
byte sending[4] = {0};
byte receiving[4] = {0};
//checks and states and inputs
float check1;
int state = 1;
char menu;
byte count;

//timer used in the ack process: timeout restarts sending process
unsigned long timer = 0;
unsigned long interval = 2000;

/*pointers to the data arrays
byte *sendbytes = (byte*)waypoint;
byte *recbytes = (byte*)rec_data;
*/
//===================================//

// send and repeat data until the answer returns
// save return data in array rec_data, recdata[0] != 2 signals correct exchange
void consend(byte data[4])
{
  receiving[0] = 2;
  
  Mirf.send(data);
  while(Mirf.isSending()){
  }
    
  Serial.println("Sending...");
  
  timer = millis();
  
  //wait for ack for "interval" amount of ms
  while(receiving[0] == 2 && ((millis() - timer) < interval))
  {
    if(Mirf.dataReady())
    {
      Mirf.getData(receiving);
      Serial.println("Received Ack");
    }
  }
  //if no ack, restart transmission
  if(receiving[0] == 2)
    consend(data);
}

//=========================//

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
  
}

//==========================//

void loop()
{
  switch (state)
  {
    case 0: //initialize coordinates
      sending[0] = 0;
      sending[1] = 0;
      sending[2] = 0;
      count = 0;
      state = 1;
      break;
      
    case 1: //receive data from serial input
      
      while(sending[3] == 0)
      {
        while(sending[count]!=0)
        {
          count++;
        }
        if (Serial.available() > 0) 
        {
          sending[count] = Serial.parseInt();
          Serial.print(count+1);
          Serial.print(". Koordinate ist: ");
          Serial.print(sending[count]);
          Serial.println(" ");
          
        }
      }
      
      state = 2;  
      break;
    
    case 2: //communicate
      
      menu = 0;
      
      Serial.println("Data sent:");
      Serial.print(sending[0]);
      Serial.print(" ");
      Serial.print(sending[1]);
      Serial.print(" ");
      Serial.print(sending[2]);
      Serial.print(" ");
      Serial.print(sending[3]);
      Serial.println();
      
      
      //send char and receive answer
      consend(sending);
        
      //print received answer to serial
      
      Serial.println("Received:");
      for(int i=0; i<4; i++)
      {
        Serial.print(receiving[i]);
        Serial.print(" ");
      }
      Serial.println();
      Serial.println();
      
      
      //input q to leave this state and get new coordinates
      if (Serial.available() > 0)
      {
        menu = Serial.read();
      }
      if(menu == 'q')
      {
        state = 0;
      }
      
      break;
     
  }
  delay(1000);
}
  
  
