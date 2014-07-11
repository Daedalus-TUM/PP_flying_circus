#include "Wire.h"
#include "I2Cdev.h"

float ge=0;
float hoehe=0, hoehe_soll=150;
float A=0.001;
float incr=1;
float ealt=0;
float esum=0;


float PID_REGLER(float e) {
  float y=0;
 
  esum = esum + e;
  y=(2.7*e)+(0.1*0.001*esum)+(0.6*(e-ealt)/0.001);
  ealt = e;
 
  return y;
}



void setup() {
  Serial.begin(115200) ;
}

void loop() {
  if(hoehe==hoehe_soll) {
    incr=0;}

  ge = PID_REGLER (hoehe_soll-hoehe);
  Serial.println(ge);
  hoehe=hoehe+incr;
}
