#include <math.h>



float Ta;//Abtastzeit
float y;
float F_z;
float e;
//---------------------------------------------------------------------//
void setup() {}
//---------------------------------------------------------------------//

//---------------------------------------------------------------------//
 int sgn(double x){
  if (x > 0.0L)
    return 1.0L;
  else if (x < 0.0L)  
    return -1.0L;
  else  
    return 0.0L;
  }
//---------------------------------------------------------------------//
//---------------------------------------------------------------------//
float HR(float z ,float z_soll ,float m ,float F_g ,float F_l ,float c_we){
    m=0.080;
    float z2nd;
    float z1st;

  
     e=(z_soll-z);
     
  F_z=PID_REGLER(2.7 , 0.1 , 0.6, 0.01, e);

  z2nd = (F_z + F_g - F_l - ( c_we * z1st * z1st * sgn(z1st)) )/m;

  z1st = z1st + z2nd ;
  
  z = z+ z1st;
  return z;
}
//---------------------------------------------------------------------//

//---------------------------------------------------------------------//
float PID_REGLER ( float Kp , float Ki ,float Kd ,float Ta ,float e ) {
  
float esum ;
float ealt ;
  esum = 0;
   ealt = 0;

  //P-Anteil mal Fehler
      
  //I_Anteil mal Fehler über die Zeit aufsummiert
  
  //D-Anteil mal Unterschied zweier aufeinanderfolgender Samples 
    
  // alle drei summieren
  
   esum = esum + e  ;
  
//   y = (Kp * e) + (Ki * Ta * esum) + ((Kd * (e – ealt))/Ta);
  
   ealt = e;
   
return y;

}

//---------------------------------------------------------------------//
//---------------------------------------------------------------------//

float RR(){
  
const float pi = 3.14159265359; 
float x_soll;
float y_soll;
float phi_soll;
float F_xr;
float F_xl;
float phi;
float phi1st;
float phi2nd;
const float h = 0.4;
const float J = 0.0014;
float abstand;
float maxSchub;


if(x_soll==0){
  if(y_soll>=0){phi_soll=(pi/2);}
  else{phi_soll=-(pi/2);}
}
else{
  if(x_soll>0){
  phi_soll = atan(y_soll/x_soll);
  }
  else{
  phi_soll = pi - atan(y_soll/(x_soll*sgn(x_soll)));
  }
}
abstand = sqrt((x_soll*x_soll)+(y_soll*y_soll));

if(abstand>0.5){
F_xr =  PID_REGLER(2.7 , 0.1 , 0.6, 0.01,phi_soll-phi) + maxSchub;
F_xl = -PID_REGLER(2.7 , 0.1 , 0.6, 0.01,phi_soll-phi) + maxSchub;
}
else{
  if(abstand>0.2){
    F_xr =  PID_REGLER(2.7 , 0.1 , 0.6, 0.01,phi_soll-phi) + 0.5*maxSchub;
    F_xl = -PID_REGLER(2.7 , 0.1 , 0.6, 0.01,phi_soll-phi) + 0.5*maxSchub;
  }
else{
      F_xr =  PID_REGLER(2.7 , 0.1 , 0.6, 0.01,phi_soll-phi) + 0.2*maxSchub;
      F_xl = -PID_REGLER(2.7 , 0.1 , 0.6, 0.01,phi_soll-phi) + 0.2*maxSchub;
}
}


phi2nd = (h*F_xr)+(h*F_xl)/J;
phi1st = phi1st + phi2nd ;
phi = phi+ phi1st;



}

//---------------------------------------------------------------------//


//---------------------------------------------------------------------//
void loop() {}
//---------------------------------------------------------------------//






