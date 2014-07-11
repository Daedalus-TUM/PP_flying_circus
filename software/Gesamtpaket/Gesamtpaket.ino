//Pinout:
#define out_R_PWM 6
#define out_L_PWM 5
#define out_H_PWM 9
#define out_R_IN 7
#define out_L_IN 4
#define out_H_IN 1

#define motor_R 0
#define motor_L 1
#define motor_H 2

#include <Wire.h>

int reading = 0, i=0;          //Hoehenrechnung
float hoehe_u[10];
float HOEHE_U=0, temp=0;

void setup()
{
  pinMode(out_R_PWM,OUTPUT);
  pinMode(out_R_IN,OUTPUT);
  pinMode(out_L_PWM,OUTPUT);
  pinMode(out_L_IN,OUTPUT);
  pinMode(out_H_PWM,OUTPUT);
  pinMode(out_H_IN,OUTPUT);
  
  Wire.begin();                // join i2c bus (address optional for master)
  
  // Erste Messung des Ultraschallsensors starten
  Wire.beginTransmission(114); // transmit to device #114 (0x69)
  Wire.write(byte(0x00));      // sets register pointer to the command register (0x00)  
  Wire.write(byte(0x51));      // command sensor to measure in "centimeters" (0x51)
  Wire.endTransmission();      // stop transmitting
  
  for (int i=0; i<256; i++) {
    motor(motor_R,0,i);
    motor(motor_L,0,i);
    delay(30);
  }
  delay(1000);
  motor(motor_R,0,0);
  motor(motor_L,0,0);
}

void loop()
{
 
}

void ultraschall() {
  if (i>9) i=0;                //Schleife für Buffer-Overlapping
  temp=0;
  
  // step 1: instruct sensor to return a particular echo reading
  Wire.beginTransmission(114); // transmit to device #114
  Wire.write(byte(0x02));      // sets register pointer to echo #1 register (0x02)
  Wire.endTransmission();      // stop transmitting

  // step 2: request reading from sensor
  Wire.requestFrom(114, 2);    // request 2 bytes from slave device #114

  // step 3: receive reading from sensor
  if(2 <= Wire.available())    // if two bytes were received
  {
    reading = Wire.read();  // receive high byte (overwrites previous reading)
    reading = reading << 8;    // shift high byte to be high 8 bits
    reading |= Wire.read(); // receive low byte as lower 8 bits
    
    hoehe_u[i]=reading;
    i=i+1;
  }
  
  for(int l=0; l<10; l++) temp=temp + hoehe_u[l]; //Mittelwertbildung über 10 Messwerte
  HOEHE_U=temp/10;
  
  // step 4: Sensor mitteilen, er soll die nächste Messung starten
  Wire.beginTransmission(114); // transmit to device #114 (0x69)
  Wire.write(byte(0x00));      // sets register pointer to the command register (0x00)  
  Wire.write(byte(0x51));      // command sensor to measure in "centimeters" (0x51)
  Wire.endTransmission();      // stop transmitting
}

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
