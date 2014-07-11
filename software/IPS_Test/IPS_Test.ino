#include <SimpleTimer.h>

//Pinout:
#define out_R_PWM 6
#define out_L_PWM 5
#define out_H_PWM 9
#define out_I_PWM 10
#define out_R_IN 7
#define out_L_IN 4
#define out_H_IN 0

SimpleTimer timer;

#define motor_R 0
#define motor_L 1
#define motor_H 2

void IPS_Sender() {
  int long time, t;
  time = micros();
  for(t=0; t<=160;){                // 8 Pulse mit 40kHz
    t = micros()-time;
    
    digitalWrite(out_I_PWM, HIGH);
    delayMicroseconds(6);
    digitalWrite(out_I_PWM, LOW);
    delayMicroseconds(3);
  }  
}

void setup()
{
  pinMode(out_R_PWM,OUTPUT);
  pinMode(out_R_IN,OUTPUT);
  pinMode(out_L_PWM,OUTPUT);
  pinMode(out_L_IN,OUTPUT);
  pinMode(out_H_PWM,OUTPUT);
  pinMode(out_H_IN,OUTPUT);
  pinMode(out_I_PWM,OUTPUT);
  
  /*for (int i=0; i<256; i++) {
    motor(motor_R,0,i);
    motor(motor_L,0,i);
    motor(motor_H,0,i);
    delay(30);
  }
  delay(1000);
  motor(motor_R,0,0);
  motor(motor_L,0,0);
  motor(motor_H,0,0);*/
  
  //TCCR1B = TCCR1B & 0b11111000 | 0x01; //ändert die PWM-Frequenz
  
  timer.setInterval(100, IPS_Sender);
}

void loop()
{
  timer.run();
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
