//Base station v0.9.1  20130704 1300
#define VERSION "v0.9"
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
#define RFBASE "bloon"
#define RFCHANNEL 3
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

class Packet {
  //byte time;
  unsigned long time;
  byte sent;
  byte data[12]={0};
  
  public:
    Packet (byte dest, byte type, byte *payload) {
      sent = 0;
      //time = millis() % 128;
      time = millis();
      data[0] = dest;
      data[1] = MYID;
      data[2] = type;
      data[3] = (byte)((PID & 0xFF00) >> 8);
      data[4] = (byte)(PID & 0x00FF);
      PID++;
      for (int i = 0; i<7;i++){
        data[i+5] = payload[i];
      }
    }
    ~Packet () {
    }
    byte send (void){
      if(millis() - time < data[2]) {
        if (millis()-time >= sent*5){
          data[11] = sent;
          Mirf.setTADDR((byte *)RFBASE);
          Mirf.send(data);
          sent++;
        }
        if(data[2] == 192) return 1;
        return 0;
      } else {
        return 1;
      }
    }
    
    unsigned int getPID() {
      return (unsigned int)((unsigned int)data[3]<< 8 + (unsigned int)data[4]);
    }
};
Packet *Packages[5];
byte busy=0;
int devicePid [MAXSTATIONS][5] = {{0}};
int deviceLastPid [MAXSTATIONS] = {0};

void sendPackages(void) {
    if ((busy&1)) {
      if (Packages[0]->send()) {
        busy &= 0b11111110;
        delete Packages[0];
      }
    } else if ((busy&2)) {
      if (Packages[1]->send()) {
        busy &= 0b11111101;
        delete Packages[1];
      }
    } else if ((busy&4)) {
      if (Packages[2]->send()) {
        busy &= 0b11111011;
        delete Packages[2];
      }
    } else if ((busy&8)) {
      if (Packages[3]->send()) {
        busy &= 0b11110111;
        delete Packages[3];
      }
    } else if ((busy&16)) {
      if (Packages[4]->send()) {
        busy &= 0b11101111;
        delete Packages[4];
      }
    }
}
boolean newPacket (byte dest, byte type, byte *payload) {
  if (busy<31){
    if (!(busy&1)) {
      Packages[0] = new Packet (dest, type,payload);
      busy |= 1;
    } else if (!(busy&2)) {
      Packages[1] = new Packet (dest, type,payload);
      busy |= 2;
    } else if (!(busy&4)) {
      Packages[2] = new Packet (dest, type,payload);
      busy |= 4;
    } else if (!(busy&8)) {
      Packages[3] = new Packet (dest, type,payload);
      busy |= 8;
    } else if (!(busy&16)) {
      Packages[4] = new Packet (dest, type,payload);
      busy |= 16;
    }
    return true;
  } else return false;
}

void deletePID (int pid) {
  Serial.println(busy);
    if ((busy&1)) {
      if (Packages[0]->getPID() == pid) {
        busy &= 0b11111110;
        delete Packages[0];
      }
    } else if ((busy&2)) {
      if (Packages[1]->getPID() == pid) {
        busy &= 0b11111101;
        delete Packages[1];
      }
    } else if ((busy&4)) {
      if (Packages[2]->getPID() == pid) {
        busy &= 0b11111011;
        delete Packages[2];
      }
    } else if ((busy&8)) {
      if (Packages[3]->getPID() == pid) {
        busy &= 0b11110111;
        delete Packages[3];
      }
    } else if ((busy&16)) {
      if (Packages[4]->getPID() == pid) {
        busy &= 0b11101111;
        delete Packages[4];
      }
    }
    Serial.println(busy);
}
byte parseMsg() {
  byte data[12];
  Mirf.getData(data);
  unsigned int pid;
  pid = ((((unsigned int)data[3])<< 8) + (unsigned int)data[4]);
  byte from = data[1];
  if(from = 30)
  {
    Serial.print("t");
    Serial.print(data[1]);
    Serial.print('0');    
    Serial.print('0');    
    Serial.print('0');    
    Serial.print('0');    
    Serial.print('0');    
    Serial.print('0');    
    Serial.print('0');    
    Serial.print('0');    
    Serial.print('0');    
    Serial.print(data[2]);  
  }
  if(data[0] == MYID ) {
    if (isNewPid(from,pid)) {
      switch(data[2]) {
        case 192: //ACK
          unsigned int ackpid;
          ackpid = ((unsigned int)data[5]<< 8 + (unsigned int)data[6]);
          deletePID(ackpid);
          break;
        case 193: {//REG
          byte pid[2];
          pid[0] = data[3];
          pid[1] = data[4];
          byte from = data[1];
          newPacket(from, (byte)192, pid);
          }
          break;
        case 64: {//deltat
          unsigned long deltat;
          deltat = (unsigned long)data[8];
          deltat += (unsigned long)data[7]<<8;
          deltat += (unsigned long)data[6]<<16;
          deltat += (unsigned long)data[5]<<24;
          byte from = data[1];
          sync(from,data[9]);
          Serial.print("t");
          if(from<10) Serial.print('0');
          Serial.print(from);
          if(deltat<10) Serial.print('0');
          if(deltat<100) Serial.print('0');
          if(deltat<1000) Serial.print('0');
          if(deltat<10000) Serial.print('0');
          if(deltat<100000) Serial.print('0');
          if(deltat<1000000) Serial.print('0');
          if(deltat<10000000) Serial.print('0');
          if(deltat<100000000) Serial.print('0');
          if(deltat<1000000000) Serial.print('0');
          Serial.print(deltat);
          Serial.print("\n");
          byte pid[2];
          pid[0] = data[3];
          pid[1] = data[4];
          
          
          newPacket(from, (byte)192, pid);
          }
          break;
        default:
          return 0; //kenne Datentyp nicht
      }
    } else {// Paket wurde bereits empfangen, wird nicht verarbeitet sondern nur bestätigt
      byte pid[2];
      pid[0] = data[3];
      pid[1] = data[4];
      newPacket(from, (byte)192, pid);
    }
  } else {
    return 0; //Nachricht nicht für mich
  }
  return 1;
}

// Prüft ob PID neu oder bereits empfangen wurde.
// Rückgabewert 0, falls bereits empfangen
//              1, falls neue PID
byte isNewPid(byte from, unsigned int pid) {
  unsigned int minpid = 65535;
  byte minpidi =0;
  for(byte i = 0 ; i<5 ; i++) {
    if(devicePid[from][i] == pid) {
      return 0; //PID bereits empfangen
    }
    if(devicePid[from][i] < minpid) {
      minpid = devicePid[from][i];
      minpidi = i;
    }
  }
  if (minpid < pid) {
    devicePid[from][minpidi] = pid;
    deviceLastPid[from] = pid;
    return 1;
  } else if (pid <15) {
    for(byte i = 0 ; i<5 ; i++) {
      if(devicePid[from][i] > 15) {
        devicePid[from][i] = 0;
        minpidi = i;
      }
    }
    devicePid[from][minpidi] = pid;
    deviceLastPid[from] = pid;
    return 1;
  } else if (deviceLastPid[from] < minpid && deviceLastPid[from] != pid){
    devicePid[from][minpidi] = pid;
    deviceLastPid[from] = pid;
    return 1;
  }
  deviceLastPid[from] = pid;
  return 2;
}


byte devicesync[MAXSTATIONS] = {0};
// sync
void sync(byte from, byte syn) {
  if (devicesync[from]) {
    Serial.println("sync");
    for (int i=0;i<MAXSTATIONS;i++) {
      devicesync[i] = 0;
    }
  }
  devicesync[from] = 1;
}

// send and repeat data until the answer returns
// save return data in array receiving, receiving[3] != 2 signals correct exchange
void consend(byte data[PAYLOAD])
{
  //receiving[3] = 2;
  
  Mirf.send(data);
  while(Mirf.isSending()){
  }
    
  Serial.println("Sending...");
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
  Serial.print("IPS ");
  Serial.println(VERSION);
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
      
      /*Serial.println("Data sent:");
      Serial.print(sending[0]);
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
      sendPackages();
      while(Mirf.isSending()) {
      }
      
      if(Mirf.dataReady())
      {
        parseMsg();
      }
      
      if (Serial.available() > 0) 
      {
          state = 4;
      }
      break;
    
    case 4: //read serial
      
      while (Serial.available()<4) {}
      for(int n=0; n<4; n++)
        sending[n+2]= Serial.read();
      
      /*if(menu == 'q')
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
      */
      
      state = 2;
      
      break;
  }
}
