#include "Wire.h"



//Variablen für Höhenregelung

int reading = 0, i=0;
float hoehe_u[10];
float hoehe_soll=100;
float temp=0;
float HOEHE_U=0;
float zeit1=0, zeit2=0;
float e=0;
float Fz=0;
boolean richtung=0;
byte geschwindigkeit =0;
float esum=0, ealt=0;




//PID_Regler
float PID_REGLER(float e, float time) {
  float y=0;
  esum = esum + e;
  y=(2.7*e)+(0.1*time*esum)+(0.6*(e-ealt)/time);
  ealt = e;
  return y;
}




void setup()
{
 
  Wire.begin();
//  Serial.begin(115200);
  
}

void loop()
{
if (i>9){                 
  i=0;}
temp=0;
HOEHE_U=0;

  Wire.beginTransmission(114);
  Wire.write(byte(0x00));
  Wire.write(byte(0x51));
  Wire.endTransmission();
  delay(70);
  Wire.beginTransmission(114);
  Wire.write(byte(0x02));
  Wire.endTransmission();
  Wire.requestFrom(114, 2);
  if(2 <= Wire.available())
  {
    reading = Wire.read();
    reading = reading << 8;
    reading |= Wire.read();
    
    hoehe_u[i]=reading; 

     i=i+1;
       
  for(int l=0; l<10; l++){     
      temp=temp + hoehe_u[l];}
  HOEHE_U=temp/10;

  
  
   if(HOEHE_U-hoehe_soll<0){
   richtung=0;}
   else { richtung=1;}

  zeit1=millis();
  Fz=PID_REGLER(hoehe_soll-HOEHE_U, zeit1-zeit2);
  zeit2=millis();
  
//   Serial.println(HOEHE_U);
//   Serial.println(Fz);
  
//  geschwindigkeit=map(Fz, -256, 256, 0, 255);
  
//  motor(motor_H, richtung, Fz);
  
  }
}


