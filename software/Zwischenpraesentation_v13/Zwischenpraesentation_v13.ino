//#include <digitalWriteFast.h>
#include <PID_v1.h>
#include <SPI.h>
#include <Mirf.h>
#include <nRF24L01.h>
#include <MirfHardwareSpiDriver.h>
#include <SimpleTimer.h>
#include <Wire.h>
#include "I2Cdev.h"
#include "MPU6050_9Axis_MotionApps41.h"

// NRF24 settings
#define RFADDR "Bloon"
#define RFCHANNEL 30
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

#define GyroMeasError PI * (40.0f / 180.0f) // gyroscope measurement error in rads/s (shown as 3 deg/s)
#define GyroMeasDrift PI * (0.0f / 180.0f) // gyroscope measurement drift in rad/s/s (shown as 0.0 deg/s/s)
#define beta sqrt(3.0f / 4.0f) * GyroMeasError // compute beta
#define zeta sqrt(3.0f / 4.0f) * GyroMeasDrift // compute zeta, the other free parameter in the Madgwick scheme usually set to a small or zero value
#define Kp 2.0f * 5.0f // these are the free parameters in the Mahony filter and fusion scheme, Kp for proportional feedback, Ki for integral
#define Ki 0.0f

SimpleTimer timer;

// Declare device MPU6050 class
MPU6050 mpu(0x69);

int16_t a1, a2, a3, g1, g2, g3, m1, m2, m3; // raw data arrays reading
uint16_t mcount = 0; // used to control display output rate
uint8_t MagRate; // read rate for magnetometer data

double pitch, yaw, roll;
float deltat = 0.0f; // integration interval for both filter schemes
uint16_t lastUpdate = 0; // used to calculate integration interval
uint16_t now = 0; // used to calculate integration interval

float ax, ay, az, gx, gy, gz, mx, my, mz; // variables to hold latest sensor data values
float q[4] = {1.0f, 0.0f, 0.0f, 0.0f}; // vector to hold quaternion
float eInt[3] = {0.0f, 0.0f, 0.0f}; // vector to hold integral error for Mahony method

byte sending[12] = {0};
byte receiving[12] = {0};

double sollH = 100;     //Sollhoehe für Hoehenregelung, vom IPS, in cm
double istH = 100;      //Isthoehe aus der Hoehenmessung vom Ultraschallsensor
double istH_IPS=0;      //Isthoehe, vom IPS, in cm
byte idummy = 0;        //Laufvariable für Hoehenmessung zur Mittelwertbildung in Ringbuffer
byte hoehe[5] = {100};  //Speicher für die aktuellsten 5 Messwerte aus der Hoehenmessung vom Ultraschallsensor
int reading = 0;        //Zwischenspeicher für Messwerte von Ultraschallsensor

double geschwindigkeitH = 0;  
byte IPS = 0;           //welche Seite gerade beim IPS sendet
int handler = 0;        //Handler für das Timing

//Richtungsregelung Variablen:
double distanz = 0;
double sollPhi = 0;       //Sollrichtung, vom IPS, in Grad
double phiIPS =0;
double phiFusion = 0;     //Richtung von IPS und IMU
double geschwindigkeit_r = 0; //Schub für die Richtungsmotoren

//double schub_const = 0; //Konstanter Schub für Abstandsregelung


//PID-Regler für Höhenregelung
PID hoehePID(&istH, &geschwindigkeitH, &sollH, 5, 0.6, 13, DIRECT);    //funktioniert ganz gut in Kombination mit Schranken für Motorleistung

//PID-Regler für Richtungsregelung
PID richtungsPID(&phiFusion, &geschwindigkeit_r, &sollPhi, 5, 0.6, 13, DIRECT);    //absolute Ausrichtung nach Nordpolwinkel funktioniert sehr gut!!

//PID-Regler für Abstand
//PID abstandsPID(&distanz, &schub_const, 0, 5, 0.6, 13, DIRECT);  //Noch nicht getestet

void NRF() {
  
  if(Mirf.dataReady()) {
    Mirf.getData(receiving);
 
    //NRF Receive Variablen
    int receive_distanz=0;
    int receive_sollPhi=0;
    int receive_phi = 0;
    
    receive_distanz = receiving[2];    // receive high byte
    receive_distanz = receive_distanz << 8;      // shift high byte to be high 8 bits
    receive_distanz |= receiving[3];   // receive low byte as lower 8 bits
    receive_sollPhi = receiving[4];    // receive high byte
    receive_sollPhi = receive_sollPhi << 8;      // shift high byte to be high 8 bits
    receive_sollPhi |= receiving[5];   // receive low byte as lower 8 bits
    sollH = receiving[6];    //Uebernehmen der neuen Sollhoehe
    receive_phi = receiving[8];        // receive high byte
    receive_phi = receive_phi << 8;            // shift high byte to be high 8 bits
    receive_phi |= receiving[9];       // receive low byte as lower 8 bits
      
    //Abwurf ausführen, wenn Wegpunkt mit Abwurf erreicht
    if(receiving[7] == 1) digitalWrite(out_Abwurf,HIGH);  //Pin für Abwurf auf High setzen um Paket abzuwerfen
    else if(receiving[7] == 50) sollH = 0;                //"deaktiviert" Hoehenregelung
    else if(receiving[7] == 0) digitalWrite(out_Abwurf,LOW);  //Pin für Abwurf auf Low setzen
     
    sollPhi = receive_sollPhi;
    phiIPS = receive_phi;
  }
  
  sending[2] = IPS;
  sending[3] = istH;
  sending[4] = sollH;
  sending[5] = receiving[6];
      
  Mirf.send(sending);
  while(Mirf.isSending()){
  } 
}


void MahonyQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz) {
  float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3]; // short name local variable for readability
  float norm;
  float hx, hy, bx, bz;
  float vx, vy, vz, wx, wy, wz;
  float ex, ey, ez;
  float pa, pb, pc;

  // Auxiliary variables to avoid repeated arithmetic
  float q1q1 = q1 * q1;
  float q1q2 = q1 * q2;
  float q1q3 = q1 * q3;
  float q1q4 = q1 * q4;
  float q2q2 = q2 * q2;
  float q2q3 = q2 * q3;
  float q2q4 = q2 * q4;
  float q3q3 = q3 * q3;
  float q3q4 = q3 * q4;
  float q4q4 = q4 * q4;

  // Normalise accelerometer measurement
  norm = sqrt(ax * ax + ay * ay + az * az);
  if (norm == 0.0f) return; // handle NaN
  norm = 1.0f / norm; // use reciprocal for division
  ax *= norm;
  ay *= norm;
  az *= norm;

  // Normalise magnetometer measurement
  norm = sqrt(mx * mx + my * my + mz * mz);
  if (norm == 0.0f) return; // handle NaN
  norm = 1.0f / norm; // use reciprocal for division
  mx *= norm;
  my *= norm;
  mz *= norm;

  // Reference direction of Earth's magnetic field
  hx = 2.0f * mx * (0.5f - q3q3 - q4q4) + 2.0f * my * (q2q3 - q1q4) + 2.0f * mz * (q2q4 + q1q3);
  hy = 2.0f * mx * (q2q3 + q1q4) + 2.0f * my * (0.5f - q2q2 - q4q4) + 2.0f * mz * (q3q4 - q1q2);
  bx = sqrt((hx * hx) + (hy * hy));
  bz = 2.0f * mx * (q2q4 - q1q3) + 2.0f * my * (q3q4 + q1q2) + 2.0f * mz * (0.5f - q2q2 - q3q3);

  // Estimated direction of gravity and magnetic field
  vx = 2.0f * (q2q4 - q1q3);
  vy = 2.0f * (q1q2 + q3q4);
  vz = q1q1 - q2q2 - q3q3 + q4q4;
  wx = 2.0f * bx * (0.5f - q3q3 - q4q4) + 2.0f * bz * (q2q4 - q1q3);
  wy = 2.0f * bx * (q2q3 - q1q4) + 2.0f * bz * (q1q2 + q3q4);
  wz = 2.0f * bx * (q1q3 + q2q4) + 2.0f * bz * (0.5f - q2q2 - q3q3);

  // Error is cross product between estimated direction and measured direction of gravity
  ex = (ay * vz - az * vy) + (my * wz - mz * wy);
  ey = (az * vx - ax * vz) + (mz * wx - mx * wz);
  ez = (ax * vy - ay * vx) + (mx * wy - my * wx);
  if (Ki > 0.0f) {
    eInt[0] += ex; // accumulate integral error
    eInt[1] += ey;
    eInt[2] += ez;
  }
  else {
    eInt[0] = 0.0f; // prevent integral wind up
    eInt[1] = 0.0f;
    eInt[2] = 0.0f;
  }

  // Apply feedback terms
  gx = gx + Kp * ex + Ki * eInt[0];
  gy = gy + Kp * ey + Ki * eInt[1];
  gz = gz + Kp * ez + Ki * eInt[2];

  // Integrate rate of change of quaternion
  pa = q2;
  pb = q3;
  pc = q4;
  q1 = q1 + (-q2 * gx - q3 * gy - q4 * gz) * (0.5f * deltat);
  q2 = pa + (q1 * gx + pb * gz - pc * gy) * (0.5f * deltat);
  q3 = pb + (q1 * gy - pa * gz + pc * gx) * (0.5f * deltat);
  q4 = pc + (q1 * gz + pa * gy - pb * gx) * (0.5f * deltat);

  // Normalise quaternion
  norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);
  norm = 1.0f / norm;
  q[0] = q1 * norm;
  q[1] = q2 * norm;
  q[2] = q3 * norm;
  q[3] = q4 * norm;
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
    
    get_Ultra();
  
    // Empfangen des neuen Höhenmesswerts
    if(2 <= Wire.available())   // if two bytes were received
    {
      reading = Wire.read();    // receive high byte (overwrites previous reading)
      reading = reading << 8;   // shift high byte to be high 8 bits
      reading |= Wire.read();   // receive low byte as lower 8 bits
      if((reading > 0) && (reading < 300)) {
        //Filter für Sprünge in der Höhenregelung
        //if(reading < (istH - 20)) sprungH = sollH - (istH - reading);
        //else if(reading > (istH + 20)) sprungH = sollH + (reading - istH);
        idummy++;
        if (idummy>4)  idummy=0;    // Abfrage für Buffer-Overlapping
        hoehe[idummy] = reading;

        //Mittelwertbildung der aktuellen 5 Hoehenwerte vom Ultraschallsensor
        int temp = 0;           //Mittelwertbildung bei Höhenmessung
        for(int l=0; l<5; l++) temp = temp + hoehe[l];
        istH = temp/5;
      }
    }
    
    // Ultraschallsensor eine neue Messung anordnen
    mess_Ultra();
    
    NRF();  //ruft die Kommunikationsfunktion mit dem NRF auf
    
    //neue Werte für den Höhenmotor übergeben
    if(geschwindigkeitH > 0) motor(motor_H, 0, geschwindigkeitH);
    else motor(motor_H, 1, 0-geschwindigkeitH);
 
    //Berechnungen für Richtungsregelung
    //phiFusion = 0.5*yaw + 0.5*phiIPS
    phiFusion = yaw;
    
    //Verhältnis von R:L = 11:10
    if(geschwindigkeit_r > 0) {
      motor(motor_R, 0, geschwindigkeit_r+10);
      if(geschwindigkeit_r < 10) motor(motor_L, 0, 10-geschwindigkeit_r);
      else motor(motor_L, 1, geschwindigkeit_r-10);
    }
    else {
      if((0-geschwindigkeit_r) < 10) motor(motor_R, 0, 10+geschwindigkeit_r);
      else motor(motor_R, 1, 0-geschwindigkeit_r-10);
      motor(motor_L, 0, 10-geschwindigkeit_r);
    }
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

void init_MPU(){
  // initialize MPU6050 device
  mpu.initialize();

  // Set up the accelerometer, gyro, and magnetometer for data output
  mpu.setRate(7); // set gyro rate to 8 kHz/(1 * rate) shows 1 kHz, accelerometer ODR is fixed at 1 KHz
  MagRate = 10; // set magnetometer read rate in Hz; 10 to 100 (max) Hz are reasonable values
    
  mpu.setDLPFMode(4); // set bandwidth of both gyro and accelerometer to ~20 Hz

  // Full-scale range of the gyro sensors:
  // 0 = +/- 250 degrees/sec, 1 = +/- 500 degrees/sec, 2 = +/- 1000 degrees/sec, 3 = +/- 2000 degrees/sec
  mpu.setFullScaleGyroRange(0); // set gyro range to 250 degrees/sec

  // Full-scale accelerometer range.
  // The full-scale range of the accelerometer: 0 = +/- 2g, 1 = +/- 4g, 2 = +/- 8g, 3 = +/- 16g
  mpu.setFullScaleAccelRange(0); // set accelerometer to 2 g range
  mpu.setIntDataReadyEnabled(true); // enable data ready interrupt
}

void init_PID() {
  //init PID-Regler für Höhenregelung
  hoehePID.SetOutputLimits(-80, 160);    //funktioniert ganz gut
  hoehePID.SetMode(AUTOMATIC);  //aktiviert PID-Regler
  hoehePID.SetSampleTime(200);  //Sample Time des PID für die Höhenregelung
  
  //init PID-Regler für Richtungsregelung
  richtungsPID.SetOutputLimits(-100, 100);
  richtungsPID.SetMode(AUTOMATIC); //aktiviert PID-Regler
  richtungsPID.SetSampleTime(200);  //Sample Time des PID für die Richtungsregelung
}

void mess_Ultra() {
  Wire.beginTransmission(114); // transmit to device #114 (0x70)
                               // the address specified in the datasheet is 224 (0xE0)
                               // but i2c adressing uses the high 7 bits so it's 112
  Wire.write(byte(0x00));      // sets register pointer to the command register (0x00)  
  Wire.write(byte(0x51));      // use 0x51 for centimeters
  Wire.endTransmission();      // stop transmitting
}

void get_Ultra() {
  // Sensor auf auslesen von bestimmten Sensorwert vorbereiten
  Wire.beginTransmission(114);// transmit to device #114
  Wire.write(byte(0x02));     // sets register pointer to echo #1 register (0x02)
  Wire.endTransmission();     // stop transmitting
  
  // Anfrage für Auslesen des Sensorwertes
  Wire.requestFrom(114, 2);   // request 2 bytes from slave device #114
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
  
  //Identität der Gondel festlegen
  sending[0] = 0;
  sending[1] = 30;
  
  //Ultraschallsensor erste Messung anordnen
  mess_Ultra();
  
  //MPU initialisieren
  init_MPU();
  
  //PID-Regler initialisieren
  init_PID();
  
  //init Timer für Timingverhalten
  timer.setInterval(periodendauer, Periode);  //Wiederholrate für Timing sämtlicher Programmteile
  
  //NRF initialisieren
  init_NRF();

  //ersten Messwert von Ultraschallsensor auslesen
  get_Ultra();
  
  //Ringspeicher für Höhenwerte anfangs mit erstem Messwert füllen
  if(2 <= Wire.available())   // if two bytes were received
  {
    reading = Wire.read();    // receive high byte (overwrites previous reading)
    reading = reading << 8;   // shift high byte to be high 8 bits
    reading |= Wire.read();   // receive low byte as lower 8 bits
    for(int l=0; l<5; l++) hoehe[l] = reading;
  }  
}

void loop() {
  
  timer.run();
  hoehePID.Compute();
  richtungsPID.Compute();
  
  if(mpu.getIntDataReadyStatus() == 1) { // wait for data ready status register to update all data registers
    mcount++;
    // read the raw sensor data
    mpu.getAcceleration ( &a1, &a2, &a3 );
    ax = a1*2.0f/32768.0f; // 2 g full range for accelerometer
    ay = a2*2.0f/32768.0f;
    az = a3*2.0f/32768.0f;

    mpu.getRotation ( &g1, &g2, &g3 );
    gx = g1*250.0f/32768.0f; // 250 deg/s full range for gyroscope
    gy = g2*250.0f/32768.0f;
    gz = g3*250.0f/32768.0f;
    
    if (mcount > 1000/MagRate) { // this is a poor man's way of setting the magnetometer read rate (see below)
      mpu.getMag ( &m1, &m2, &m3 );
      mx = m1*10.0f*1229.0f/4096.0f + 18.0f; // milliGauss (1229 microTesla per 2^12 bits, 10 mG per microTesla)
      my = m2*10.0f*1229.0f/4096.0f + 70.0f; // apply calibration offsets in mG that correspond to your environment and magnetometer
      mz = m3*10.0f*1229.0f/4096.0f + 270.0f;
      mcount = 0;
    }
  }
  now = micros();
  deltat = ((now - lastUpdate)/1000000.0f); // set integration time by time elapsed since last filter update
  lastUpdate = now;
  
  // Sensors x (y)-axis of the accelerometer is aligned with the y (x)-axis of the magnetometer;
  // the magnetometer z-axis (+ down) is opposite to z-axis (+ up) of accelerometer and gyro!
  // We have to make some allowance for this orientationmismatch in feeding the output to the quaternion filter.
  // For the MPU-9150, we have chosen a magnetic rotation that keeps the sensor forward along the x-axis just like
  // in the LSM9DS0 sensor. This rotation can be modified to allow any convenient orientation convention.
  // This is ok by aircraft orientation standards!
  // Pass gyro rate as rad/s
 // MadgwickQuaternionUpdate(ax, ay, az, gx*PI/180.0f, gy*PI/180.0f, gz*PI/180.0f, my, mx, mz);
  MahonyQuaternionUpdate(ax, ay, az, gx*PI/180.0f, gy*PI/180.0f, gz*PI/180.0f, my, mx, mz);
 
  yaw = atan2(2.0f * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]);
  pitch = -asin(2.0f * (q[1] * q[3] - q[0] * q[2]));
  roll = atan2(2.0f * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]);
  pitch *= 180.0f / PI;
  yaw *= 180.0f / PI - 2.6; // Declination at Munich, Germany is 2 degrees 34 minutes and 41 seconds
  roll *= 180.0f / PI;
  
  /*Serial.print("Yaw, Pitch, Roll: ");
  Serial.print(yaw, 2);
  Serial.print(", ");
  Serial.print(pitch, 2);
  Serial.print(", ");
  Serial.println(roll, 2);
  Serial.print("rate = "); Serial.print((float)1.0f/deltat, 2); Serial.println(" Hz");*/
}
