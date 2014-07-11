#include <PID_v1.h>
#include <SPI.h>
#include <Mirf.h>
#include <nRF24L01.h>
#include <MirfHardwareSpiDriver.h>
#include <SimpleTimer.h>
#include <Wire.h>

// NRF24 settings
#define RFADDR "Bloon"
#define RFCHANNEL 30

//Pinout:
#define out_R_PWM 6
#define out_L_PWM 5
#define out_H_PWM 9
//#define out_I_PWM 10
#define out_R_IN 7
#define out_L_IN 4
#define out_H_IN 0
#define out_Abwurf 1

SimpleTimer timer;

#define motor_R 0
#define motor_L 1
#define motor_H 2

byte sending[5] = {0};
byte receiving[5] = {0};

double sollH = 0; //Sollhoehe für Hoehenregelung
double istH = 150;  //Isthoehe aus der Hoehenmessung vom Ultraschallsensor
byte idummy = 0;  //Laufvariable für Hoehenmessung zur Mittelwertbildung in Ringbuffer
byte hoehe[5] = {150};  //Speicher für die aktuellsten 10 Messwerte aus der Hoehenmessung vom Ultraschallsensor
int reading = 0;
double geschwindigkeit = 0;

//PID-Regler für Höhenregelung
PID myPID(&istH, &geschwindigkeit, &sollH,2.7,0.1,0.6, DIRECT);

//Motoransteuerung für alle drei Motoren
void motor(byte motor, boolean direction, byte speed) { //speed from 0 to 255
  if (motor == motor_R) {
    if (direction == 0) {
      digitalWrite(out_R_IN,HIGH);
    } else {
      digitalWrite(out_R_IN,LOW);
    }
    analogWrite(out_R_PWM,speed);
    }
  if (motor == motor_L) {
    if (direction == 0) {
      digitalWrite(out_L_IN,HIGH);
    } else {
      digitalWrite(out_L_IN,LOW);
    }
    analogWrite(out_L_PWM,speed);
  }
  if (motor == motor_H) {
    if (direction == 0) {
      digitalWrite(out_H_IN,HIGH);
    } else {
      digitalWrite(out_H_IN,LOW);
    }
    analogWrite(out_H_PWM,speed);
  }
}

void Periode() {
  // Sensor auf auslesen von bestimmten Sensorwert vorbereiten
  Wire.beginTransmission(114);// transmit to device #114
  Wire.write(byte(0x02));     // sets register pointer to echo #1 register (0x02)
  Wire.endTransmission();     // stop transmitting

  // Anfrage für Auslesen des Sensorwertes
  Wire.requestFrom(114, 2);   // request 2 bytes from slave device #114

  if (idummy>9)  idummy=0;    // Abfrage für Buffer-Overlapping

  // Empfangen des neuen Messwertes
  if(2 <= Wire.available())   // if two bytes were received
  {
    reading = Wire.read();    // receive high byte (overwrites previous reading)
    reading = reading << 8;   // shift high byte to be high 8 bits
    reading |= Wire.read();   // receive low byte as lower 8 bits
    hoehe[idummy]=reading;
    idummy++;
  }
    
  //Mittelwertbildung der aktuellen 10 Hoehenwerte vom Ultraschallsensor
  int temp = 0;
  for(int l=0; l<5; l++){     
      temp=temp + hoehe[l];
    }
    istH=temp/5;
    
    motor(motor_H, 0, geschwindigkeit);

  
    if(Mirf.dataReady()) {
    Mirf.getData(receiving);
    
    //Uebertragen der empfangenen Werte für die Motoransteuerung
    motor(motor_R,0,receiving[0]);
    motor(motor_L,0,receiving[1]);
    motor(motor_H,0,receiving[2]);
    sollH = receiving[3];  //Uebernehmen der Sollhoehe
    if(receiving[4] == 1) digitalWrite(out_Abwurf,HIGH);  //Pin für Abwurf auf High setzen um Paket abzuwerfen
    else digitalWrite(out_Abwurf,LOW);  //Pin für Abwurf auf Low setzen
    
    //return part of incoming message as communication test
    sending[0] = istH;
    sending[3] = 1;
    
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
    Serial.print(" ");
    Serial.print(receiving[4]);
    Serial.println();
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
    Serial.println();
  }
  
  // Ultraschallsensor eine neue Messung anordnen
  Wire.beginTransmission(114); // transmit to device #114 (0x70)
                               // the address specified in the datasheet is 224 (0xE0)
                               // but i2c adressing uses the high 7 bits so it's 112
  Wire.write(byte(0x00));      // sets register pointer to the command register (0x00)  
  Wire.write(byte(0x51));      // use 0x51 for centimeters
  Wire.endTransmission();      // stop transmitting
  
}

void setup() {
  pinMode(out_R_PWM,OUTPUT);
  pinMode(out_R_IN,OUTPUT);
  pinMode(out_L_PWM,OUTPUT);
  pinMode(out_L_IN,OUTPUT);
  pinMode(out_H_PWM,OUTPUT);
  pinMode(out_H_IN,OUTPUT);
  //pinMode(out_I_PWM,OUTPUT);
  pinMode(out_Abwurf,OUTPUT);
  
  //Testlauf für die Motoren um korrekte Funktion festzustellen
  for (int i=0; i<128; i++) {
    motor(motor_R,0,i);
    motor(motor_L,0,i);
    motor(motor_H,0,i);
    delay(30);
  }
  delay(300);
  motor(motor_R,0,0);
  motor(motor_L,0,0);
  motor(motor_H,0,0);
  delay(500);
  
  Serial.begin(115200);
  Wire.begin();                // join I2C bus
  
  // Ultraschallsensor erste Messung anordnen
  Wire.beginTransmission(114); // transmit to device #114 (0x70)
                               // the address specified in the datasheet is 224 (0xE0)
                               // but i2c adressing uses the high 7 bits so it's 112
  Wire.write(byte(0x00));      // sets register pointer to the command register (0x00)  
  Wire.write(byte(0x51));      // use 0x51 for centimeters
  Wire.endTransmission();      // stop transmitting
    
  //init NRF24
  Mirf.spi = &MirfHardwareSpi;
  Mirf.cePin = 21;
  Mirf.csnPin = 20;
  Mirf.init();
  Mirf.setRADDR((byte *)RFADDR);
  Mirf.payload = 5;
  Mirf.channel = RFCHANNEL;
  Mirf.config();
  
  myPID.SetMode(AUTOMATIC);  //aktiviert PID-Regler
  myPID.SetOutputLimits(0, 255);
  
  timer.setInterval(200, Periode);  //Wiederholrate für NRF auf 200ms
}


void loop(){
  timer.run();
  myPID.Compute();
}
