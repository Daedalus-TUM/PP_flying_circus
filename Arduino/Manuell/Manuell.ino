#include <SPI.h>
#include <Mirf.h>
#include <nRF24L01.h>
#include <MirfHardwareSpiDriver.h>
#include <stdio.h>
#include <stdlib.h>

// NRF24 settings
// NRF24 settings
#define RFADDR "base"
#define RFBASE "Bloon"
#define RFCHANNEL 30
#define PAYLOAD 12

/*data arrays to be sent and received
float waypoint[3] = {0};
float rec_data[3] = {0};
*/

//REFERENCE: Funkprotokoll |  30  |  0  |Distanz|Distanz|Nordwinkel|Nordwinkel|Sollhoehe|Abwurf|Gondelwinkel|Gondelwinkel|
//                         |   0  |  1  |   2   |   3   |     4    |      5   |     6   |   7  |      8     |     9      |
//
//Nordwinkel:    Winkel zwischen norwärts gerichtetem Vektor und Vektor zum nächsten Wegpunkt
//Distanz:       absolute Distanz zwischen Gondel und nächstem Wegpunkt
//Gondelwinkel:  Winkel zwischen norwärts gerichtetem Vektor und Richtungswinkel der Gondel

byte sending[PAYLOAD] = {0};
byte receiving[PAYLOAD] = {0};
//checks and states and inputs
float check1;
int state = 0;
int nordwinkel, distanz, gondelwinkel;
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

// send and repeat data until the answer returns (update: ack obsolete, just keep sending)
// save return data in array rec_data, recdata[0] != 2 signals correct exchange
void consend(byte data[PAYLOAD])
{
  //receiving[3] = 2;
  
  Mirf.send(data);
  while(Mirf.isSending()){
  }
    
  Serial.println("Sending...");
  
  //timer = millis();
  
  //wait for ack for "interval" amount of ms
  //while(receiving[3] == 2 && ((millis() - timer) < interval))
  //{
  if(Mirf.dataReady())
  {
    Mirf.getData(receiving);
    Serial.println("Received Ack");
  }
  //}
  ///if no ack, restart transmission
  //if(receiving[3] == 2)
  //consend(data);
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
  Mirf.setTADDR((byte *)RFBASE);
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
      Serial.println("W, S:     Einstellen der Sollhoehe");
      Serial.println("R, F:     Eingabe von Distanz und Nordwinkel.");
      Serial.println("Y:        Reset & Return zu diesem Menu");
      Serial.println("X:        wirft Paket ab");
      Serial.println();
      Serial.println("Bitte Startwerte eingeben");
          
      sending[0] = 30;
      sending[1] = 0;
      sending[2] = 0;
      sending[3] = 0;
      sending[4] = 0;
      sending[5] = 0;
      sending[6] = 100;
      sending[7] = 0;
      count = 0;
      state = 2;
      distanz = 0;
      nordwinkel = 0;
      gondelwinkel = 0;
      break;
      
    case 1: //distanz from serial input
      
      Serial.println("Distanz zum Wegpunkt:");
      
      if (Serial.available() > 0) 
      {
        distanz = Serial.parseInt();
        Serial.print("neue Distanz: ");
        Serial.println(distanz);
        Serial.println();
        
        if(distanz < 0)
          distanz = distanz + 65536;
        
        sending[3] = distanz & 0xFF;
        sending[2] = distanz >> 8;
        
        state = 2;     
      }
      
      break;
    
    case 5: //nordwinkel from serial input
      
      Serial.println("Nord-Winkel zum Wegpunkt:");
      
      if (Serial.available() > 0) 
      {
        nordwinkel = Serial.parseInt();
        Serial.print("neuer Nord-Winkel: ");
        Serial.println(nordwinkel);
        Serial.println();
        
        if(nordwinkel < 0)
          nordwinkel = nordwinkel + 65536;
        
        sending[5] = nordwinkel & 0xFF;
        sending[4] = nordwinkel >> 8;
        
        state = 2;     
      }
      
      break;
    
    case 2: //communicate
      
      menu = 0;
      
      Serial.println("Data sent:");
      Serial.print(sending[0]);
      Serial.print(" ");
      Serial.print(sending[1]);
      Serial.print(" | ");
      Serial.print(sending[2]);
      Serial.print(" ");
      Serial.print(sending[3]);
      Serial.print(" | ");
      Serial.print(sending[4]);
      Serial.print(" ");
      Serial.print(sending[5]);
      Serial.print(" | ");
      Serial.print(sending[6]);
      Serial.print(" | ");
      Serial.print(sending[7]);
      Serial.print(" | ");
      Serial.print(sending[8]);
      Serial.print(" ");
      Serial.print(sending[9]);
      Serial.print(" ");
      Serial.print(sending[10]);
      Serial.print(" ");
      Serial.print(sending[11]);
      Serial.println();
      
      
      //send char and receive answer
      consend(sending);
        
      //print received answer to serial
      
      Serial.println("Received:");
      for(int i=0; i<10; i++)
      {
        Serial.print(receiving[i]);
        Serial.print("  ");
      }
      Serial.println();
      Serial.println();
      
      
      //input chars to change values
      
      if (Serial.available() > 0)
      {
        menu = Serial.read();
      }
      switch(menu)
      {
        case 'w': //Sollhoehe erhoehen
          sending[3] = sending[6] + 10;
          break;

        case 's': //Sollhoehe senken
          sending[3] = sending[6] - 10;
          break;

        case 'r': //Distanz
          state = 1;
          break;

        case 'f': //Nordwinkel
          state = 5;
          break;

        case 'x': //Paketabwurf
          if(sending[7] == 1)
            sending[7] = 0;
          else
            sending[7] = 1;
          break;

        case 'y': //Reset
          state = 0;
          break;

        case 'c': //Clear
          sending[0] =30;
          sending[1] =0;
          sending[2] =0;
          sending[4] =0;
          sending[5] =0;
          sending[6] =100;
          sending[7] =0;
          sending[8] =0;
          break;
      }
      break;
     
  }
  delay(500);
}
  
  
