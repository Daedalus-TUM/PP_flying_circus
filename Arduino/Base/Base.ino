//Base station v0.9.1  20130704 1300
#define VERSION "alpha"
#define DEVICEID 0

#define VERSIONMAJOR 0
#define VERSIONMINOR 9

#include <SPI.h>
#include <Mirf.h>
#include <nRF24L01.h>
#include <MirfHardwareSpiDriver.h>
#include <stdio.h>
#include <stdlib.h>

// NRF24 settings
#define RFADDR "base"
#define RFBASE "Bloon"
#define RFCHANNEL 30
#define PAYLOAD 12

//constants
const byte MYID = DEVICEID;
const byte Version[2] = {VERSIONMAJOR,VERSIONMINOR};
const byte MAXSTATIONS = 15;

// Global Variables
int PID = 5;
int state = 0;

byte sending[PAYLOAD] = {1};
byte receiving[PAYLOAD] = {0};
unsigned long timer = 0;
unsigned long interval = 2000;
char menu;
byte count;
int wpx, wpy;
byte sollh, Pack;


void consend(byte data[PAYLOAD])
{
  Mirf.send(data);
  while(Mirf.isSending()){
  }
  Serial.println("Sending...");
  for(int i=0;i<12;i++)
  {
    Serial.print(" ");
    Serial.print(data[i]);
  }
  /*
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
  */
}


void setup() {
  Serial.begin(115200);
  Serial.print("Balloon Control ");
  Serial.println(VERSION);
  //init NRF24
  Mirf.spi = &MirfHardwareSpi;
  Mirf.cePin = 9;
  Mirf.csnPin = 10;
  Mirf.init();
  Mirf.setRADDR((byte *)RFADDR);
  Mirf.payload = 12;
  Mirf.channel = RFCHANNEL;
  Mirf.config();
  state = 0;
  
}


void loop()
{
  switch (state)
  {
    case 0: //initialize coordinates
      
      /*Serial.println("Waehrend des Betriebs sind die folgenden Werte mit 'Enter' einzugeben:");
      Serial.println("Q, W, E: erhoehen die Werte des linken, mittleren und rechten Motors");
      Serial.println("A, S, D: senken die Werte des linken, mittleren und rechten Motors");
      Serial.println("F        setzt alle Motoren auf Null.");
      Serial.println("R        Return zu diesem Menu");
      Serial.println("X        wirft Paket ab");
      */
      sending[0] = 30;
      sending[1] = 0;
      sending[2] = 0;
      sending[3] = 0;
      sending[4] = 0;
      sending[5] = 0;
      count = 0;
      state = 3;
      break;
      
    /*case 1: //receive data from serial input
      
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
      break;*/
    
    case 2: //send data
      
      Serial.println("Data sent:");
      /*Serial.print(sending[0]);
      Serial.print(" ");
      Serial.print(sending[1]);
      Serial.print(" ");
      Serial.print(sending[2]);
      Serial.print(" ");
      Serial.print(sending[3]);
      Serial.print(" ");
      Serial.print(sending[4]);
      Serial.println();*/
      
      
      //send char and receive answer
      consend(sending);
      
      state = 3;      
      break;
      
    case 3: //listen and relay
      while(Mirf.isSending()) {
      }
      
    
      if (Serial.available() > 0) 
      {
          state = 4;
      }
      break;
    
    case 4: //read serial
      
      if(Serial.available() > 3) 
      {
        for(int n=0; n<4; n++)
          sending[n+2]= Serial.read();
      }
      
      state = 2;
      break;
  }
}
