#include <SPI.h>
#include <Mirf.h>
#include <nRF24L01.h>
#include <MirfHardwareSpiDriver.h>
#include <stdio.h>
#include <stdlib.h>

// NRF24 settings
#define RFADDR "Base"
#define RFCHANNEL 30
#define PAYLOAD 5

/*data arrays to be sent and received
float waypoint[3] = {0};
float rec_data[3] = {0};
*/
byte sending[PAYLOAD] = {1};
byte receiving[PAYLOAD] = {0};
//checks and states and inputs
float check1;
int state = 0;
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
void consend(byte data[PAYLOAD])
{
  receiving[3] = 2;
  
  Mirf.send(data);
  while(Mirf.isSending()){
  }
    
  Serial.println("Sending...");
  
  timer = millis();
  
  //wait for ack for "interval" amount of ms
  while(receiving[3] == 2 && ((millis() - timer) < interval))
  {
    if(Mirf.dataReady())
    {
      Mirf.getData(receiving);
      Serial.println("Received Ack");
    }
  }
  //if no ack, restart transmission
  if(receiving[3] == 2)
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
  Mirf.payload = PAYLOAD;
  Mirf.channel = RFCHANNEL;
  Mirf.config();
  
}

//==========================//

void loop()
{
  switch (state)
  {
    case 0: //initialize coordinates

      Serial.println("Waehrend des Betriebs sind die folgenden Werte mit 'Enter' einzugeben:");
      Serial.println("Q, W, E: erhoehen die Werte des linken, mittleren und rechten Motors");
      Serial.println("A, S, D: senken die Werte des linken, mittleren und rechten Motors");
      Serial.println("F        setzt alle Motoren auf Null.");
      Serial.println("R        Return zu diesem Menu");
      Serial.println("X        wirft Paket ab");
      Serial.println();
      Serial.println("Bitte Startwerte eingeben");
          
      sending[0] = 1;
      sending[1] = 1;
      sending[2] = 1;
      sending[3] = 1;
      sending[4] = 0;
      count = 0;
      state = 1;
      break;
      
    case 1: //receive data from serial input
      
      while(sending[3] == 1)
      {
        while(sending[count]!=1)
        {
          count++;
        }
        if (Serial.available() > 0) 
        {
          sending[count] = Serial.parseInt();
          Serial.print(count+1);
          Serial.print(". Wert ist: ");
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
      Serial.print(" ");
      Serial.print(sending[4]);
      Serial.println();
      
      
      //send char and receive answer
      consend(sending);
        
      //print received answer to serial
      
      Serial.println("Received:");
      for(int i=0; i<5; i++)
      {
        Serial.print(receiving[i]);
        Serial.print("  ");
      }
      Serial.println();
      Serial.println();
      
      
      //input r to leave this state and get new coordinates
      //
      if (Serial.available() > 0)
      {
        menu = Serial.read();
      }
      if(menu == 'q')
      {
        sending[0] = sending[0] + 10;
      }
      if(menu == 'a')
      {
        sending[0] = sending[0] - 10;
      }
      if(menu == 'w')
      {
        sending[2] = sending[2] + 10;
      }
      if(menu == 's')
      {
        sending[2] = sending[2] - 10;
      }
      if(menu == 'e')
      {
        sending[1] = sending[1] + 10;
      }
      if(menu == 'd')
      {
        sending[1] = sending[1] - 10;
      }
      if(menu == 'r')
      {
        state = 0;
      }
      if(menu == 'x')
      {
        sending[4] = 1;
      }
      if(menu == 'f')
      {
        sending[0] =0;
        sending[1] =0;
        sending[2] =0;
        sending[4] =0;
      }
      
      break;
     
  }
  delay(500);
}
  
  
