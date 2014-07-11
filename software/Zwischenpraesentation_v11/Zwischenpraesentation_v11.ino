//#include <digitalWriteFast.h>
#include <PID_v1.h>
#include <SPI.h>
#include <Mirf.h>
#include <nRF24L01.h>
#include <MirfHardwareSpiDriver.h>
#include <SimpleTimer.h>
#include <Wire.h>

// NRF24 settings
#define RFADDR "Bloon"
#define RFCHANNEL 3
#define RFBASE "base"

//Pinout:
#define out_R_PWM 6
#define out_L_PWM 5
#define out_H_PWM 9
#define out_R_IN 7
#define out_L_IN 4
#define out_H_IN 0
#define out_Abwurf 1

//Pinout IPS:
#define out_I_PWM 10
#define out_I_L 19
#define out_I_R 18
#define out_I_IR 8

#define motor_R 0
#define motor_L 1
#define motor_H 2

#define periodendauer 50  //Periodendauer für timer in ms

SimpleTimer timer;

byte sending[12] = {0};
byte receiving[12] = {0};

double sollH = 150;     //Sollhoehe für Hoehenregelung
double istH = 150;      //Isthoehe aus der Hoehenmessung vom Ultraschallsensor
//double sprungH = 100;
byte idummy = 0;        //Laufvariable für Hoehenmessung zur Mittelwertbildung in Ringbuffer
byte hoehe[5] = {150};  //Speicher für die aktuellsten 5 Messwerte aus der Hoehenmessung vom Ultraschallsensor
int reading = 0;        //Zwischenspeicher für Messwerte von Ultraschallsensor
//int offsetH = 0;      //Offsetkorrektur für Sprungwerte von Ultraschallsensor
double geschwindigkeitH = 0;  
byte IPS = 0;           //welche Seite gerade beim IPS sendet
int handler = 0;        //Handler für das Timing
int temp = 0;           //Mittelwertbildung bei Höhenmessung

//Richtungsregelung Variablen:
int sollX = 0;         //Sollkoordinate für x-Richtung
int sollY = 0;         //Sollkoordinate für y-Richtung
double sollPhi = 0;    //Sollrichtung
double geschwindigkeit_r = 0; //Schub für die Richtungsmotoren


//PID-Regler für Höhenregelung
PID hoehePID(&istH, &geschwindigkeitH, &sollH, 5, 0.6, 13, DIRECT);    //funktioniert ganz gut in Kombination mit Schranken für Motorleistung

//PID-Regler für Richtungsregelung
PID richtungsPID(0, &geschwindigkeit_r, &sollPhi, 2.7, 0.1, 0.6, DIRECT);

void NRF() {
  if(Mirf.dataReady()) {
    Mirf.getData(receiving);
    
    //Abfrage, ob Nachricht für Gondel bestimmt ist
    if((receiving[0] == 0) && (receiving[1] == 30)) {
      sollH = receiving[6];    //Uebernehmen der neuen Sollhoehe
      //Übernehmen der neuen X und Y Koordinaten
      sollX = receiving[2];    // receive high byte
      sollX = sollX << 8;      // shift high byte to be high 8 bits
      sollX |= receiving[3];   // receive low byte as lower 8 bits
      sollY = receiving[4];    // receive high byte
      sollY = sollY << 8;      // shift high byte to be high 8 bits
      sollY |= receiving[5];   // receive low byte as lower 8 bits
      
      //Abwurf ausführen, wenn Wegpunkt mit Abwurf erreicht
      if(receiving[7] == 1) digitalWrite(out_Abwurf,HIGH);  //Pin für Abwurf auf High setzen um Paket abzuwerfen
      else if(receiving[7] == 50) sollH = 0;                //"deaktiviert" Hoehenregelung
      else if(receiving[7] == 0) digitalWrite(out_Abwurf,LOW);  //Pin für Abwurf auf Low setzen
    }
  }
    
  sending[2] = IPS;
  sending[3] = istH;
      
  Mirf.send(sending);
  while(Mirf.isSending()){
  } 
}

//Motoransteuerung für alle drei Motoren
void motor(byte motor, boolean direction, byte speed) { //speed from 0 to 255
  if (motor == motor_R) {
    if (direction == 0) digitalWrite(out_R_IN,HIGH);
    else digitalWrite(out_R_IN,LOW);
    analogWrite(out_R_PWM,speed);
    }
  if (motor == motor_L) {
    if (direction == 0) digitalWrite(out_L_IN,HIGH);
    else digitalWrite(out_L_IN,LOW);
    analogWrite(out_L_PWM,speed);
  }
  if (motor == motor_H) {
    if (direction == 0) digitalWrite(out_H_IN,HIGH);
    else digitalWrite(out_H_IN,LOW);
    analogWrite(out_H_PWM,speed);
  }
}

//Erzeugen des IPS-Signals für Ultraschallwandler und IR-LED
void IPS_Sender(byte Seite) {
  int long time, t;
  digitalWrite(out_I_IR, LOW);     //aktiviert IR-LED
  digitalWrite(Seite, HIGH);        //aktiviert entsprechenden Ultraschallwandler
  time = micros();
  for(t=0; t<=165;){                // 8 Pulse mit 40kHz
    t = micros()-time;
    digitalWrite(out_I_PWM, 1);
    delayMicroseconds(6);
    digitalWrite(out_I_PWM, 0);
    delayMicroseconds(3);
  } 
  digitalWrite(Seite, LOW);        //deaktiviert entsprechenden Ultraschallwandler
  digitalWrite(out_I_IR, HIGH);     //deaktiviert IR-LED
}

void Periode() {
  if (handler == 0) {
    handler = 1;
    //IPS-Signal erzeugen
    if (IPS == 0) {  //rechter Sender im letzten Durchlauf
      IPS = 1;
      IPS_Sender(out_I_L);
    }
    else {           //linker Sender im letzten Durchlauf
      IPS = 0;
      IPS_Sender(out_I_R);
    }
  }
  
  else if (handler == 1) {
    handler = 0;
    // Sensor auf auslesen von bestimmten Sensorwert vorbereiten
    Wire.beginTransmission(114);// transmit to device #114
    Wire.write(byte(0x02));     // sets register pointer to echo #1 register (0x02)
    Wire.endTransmission();     // stop transmitting
  
    // Anfrage für Auslesen des Sensorwertes
    Wire.requestFrom(114, 2);   // request 2 bytes from slave device #114
  
    // Empfangen des neuen Höhenmesswerts
    if(2 <= Wire.available())   // if two bytes were received
    {
      reading = Wire.read();    // receive high byte (overwrites previous reading)
      reading = reading << 8;   // shift high byte to be high 8 bits
      reading |= Wire.read();   // receive low byte as lower 8 bits
      if((reading > 0) && (reading < 300)) {
        //Filter für Sprünge in der Höhenregelung
        //funktioniert ganz gut... vll rücksetzten nach bestimmter Zeit auf Anfangswert???
        //if(reading < (istH - 20)) sprungH = sollH - (istH - reading);
        //else if(reading > (istH + 20)) sprungH = sollH + (reading - istH);
        idummy++;
        if (idummy>4)  idummy=0;    // Abfrage für Buffer-Overlapping
        hoehe[idummy] = reading;

        //Mittelwertbildung der aktuellen 5 Hoehenwerte vom Ultraschallsensor
        temp = 0;
        for(int l=0; l<5; l++) temp = temp + hoehe[l];
        istH = temp/5;
      }
    }
    
    // Ultraschallsensor eine neue Messung anordnen
    Wire.beginTransmission(114); // transmit to device #114 (0x70)
                                 // the address specified in the datasheet is 224 (0xE0)
                                 // but i2c adressing uses the high 7 bits so it's 112
    Wire.write(byte(0x00));      // sets register pointer to the command register (0x00)  
    Wire.write(byte(0x51));      // use 0x51 for centimeters
    Wire.endTransmission();      // stop transmitting
    
    NRF();  //ruft die Kommunikationsfunktion mit dem NRF auf
    
    //neue Werte für den Höhenmotor übergeben
    if(geschwindigkeitH > 0) motor(motor_H, 0, geschwindigkeitH);
    else motor(motor_H, 1, 0-geschwindigkeitH);
 
    //Berechnungen für Richtungsregelung
    sollPhi = atan2(sollY, sollX)*180/3.1415;
    
    //Verhältnis von R:L = 11:10
    motor(motor_R, 0, geschwindigkeit_r+0);
    motor(motor_L, 0, 0-geschwindigkeit_r);
  }
}

//init für NRF-Modul
void init_NRF() {
  Mirf.spi = &MirfHardwareSpi;
  Mirf.cePin = 21;
  Mirf.csnPin = 20;
  Mirf.init();
  Mirf.setRADDR((byte *)RFADDR);
  Mirf.setTADDR((byte *)RFBASE);
  Mirf.payload = 12;
  Mirf.channel = RFCHANNEL;
  Mirf.config();
  
}

void setup() {
  pinMode(out_R_PWM,OUTPUT);
  pinMode(out_R_IN,OUTPUT);
  pinMode(out_L_PWM,OUTPUT);
  pinMode(out_L_IN,OUTPUT);
  pinMode(out_H_PWM,OUTPUT);
  pinMode(out_H_IN,OUTPUT);
  pinMode(out_I_PWM,OUTPUT);
  pinMode(out_I_R,OUTPUT);
  pinMode(out_I_L,OUTPUT);
  pinMode(out_I_IR,OUTPUT);
  pinMode(out_Abwurf,OUTPUT);
 
  Serial.begin(115200);
  Wire.begin();                // join I2C bus
  
  // Ultraschallsensor erste Messung anordnen
  Wire.beginTransmission(114); // transmit to device #114 (0x70)
                               // the address specified in the datasheet is 224 (0xE0)
                               // but i2c adressing uses the high 7 bits so it's 112
  Wire.write(byte(0x00));      // sets register pointer to the command register (0x00)  
  Wire.write(byte(0x51));      // use 0x51 for centimeters
  Wire.endTransmission();      // stop transmitting
  
  //Identität der Gondel festlegen
  sending[0] = 0;
  sending[1] = 30;
  
  //init PID-Regler für Höhenregelung
  hoehePID.SetOutputLimits(-80, 160);    //funktioniert ganz gut
  hoehePID.SetMode(AUTOMATIC);  //aktiviert PID-Regler
  hoehePID.SetSampleTime(200);  //Sample Time des PID für die Höhenregelung
  
  //init PID-Regler für Richtungsregelung
  richtungsPID.SetOutputLimits(-127, 127);
  richtungsPID.SetMode(AUTOMATIC); //aktiviert PID-Regler
  
  //init Timer für Timingverhalten
  timer.setInterval(periodendauer, Periode);  //Wiederholrate von 100ms für Timing sämtlicher Programmteile
  
  //NRF initialisieren
  init_NRF();

  // Sensor auf auslesen von bestimmten Sensorwert vorbereiten
  Wire.beginTransmission(114);// transmit to device #114
  Wire.write(byte(0x02));     // sets register pointer to echo #1 register (0x02)
  Wire.endTransmission();     // stop transmitting
  // Anfrage für Auslesen des Sensorwertes
  Wire.requestFrom(114, 2);   // request 2 bytes from slave device #114
  
  //Ringspeicher für Höhenwerte anfangs mit erstem Messwert füllen
  if(2 <= Wire.available())   // if two bytes were received
  {
    reading = Wire.read();    // receive high byte (overwrites previous reading)
    reading = reading << 8;   // shift high byte to be high 8 bits
    reading |= Wire.read();   // receive low byte as lower 8 bits
    for(int l=0; l<5; l++) hoehe[l] = reading;
  }  
}

void loop(){
  timer.run();
  hoehePID.Compute();
  //richtungsPID.Compute();
}
