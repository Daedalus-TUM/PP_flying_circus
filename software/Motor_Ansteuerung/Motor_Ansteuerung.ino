//Pinout:
#define out_R_PWM 6
#define out_L_PWM 5
#define out_H_PWM 9
#define out_R_IN 7
#define out_L_IN 4
#define out_H_IN 0



#define motor_R 0
#define motor_L 1
#define motor_H 2

void setup()
{
  pinMode(out_R_PWM,OUTPUT);
  pinMode(out_R_IN,OUTPUT);
  pinMode(out_L_PWM,OUTPUT);
  pinMode(out_L_IN,OUTPUT);
  pinMode(out_H_PWM,OUTPUT);
  pinMode(out_H_IN,OUTPUT);
  
  for (int i=0; i<256; i++) {
    motor(motor_R,0,i);
    motor(motor_L,0,i);
    motor(motor_H,0,i);
    delay(30);
  }
  delay(1000);
  motor(motor_R,0,0);
  motor(motor_L,0,0);
  motor(motor_H,0,0);
}

void loop()
{

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
