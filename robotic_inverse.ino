#include "math.h"     

unsigned long timep, time, etime;

//// MOTOR 1 ////
double enA=7;
const int in1=6;
const int in2=5;
const int chA=20;
const int chB=21;

int count=0; 
int angle=0;

boolean A,B;
byte state, statep;

//// MOTOR 2 ////
double enB=8;
const int in1_2=9;
const int in2_2=10;
const int chA_2=18;
const int chB_2=19;

int count2=0;
int angle2=0; 

boolean C,D;
byte state2, statep2;

//// MOTOR 3 ////
double enA_2=13;
const int in1_3=12;
const int in2_3=11;
const int chA_3=2;
const int chB_3=3;

int count3 = 0;
int angle3 = 0; 

boolean E,F;
byte state3, statep3;

///////////////////////////////////////////////////////

String point; // set x coordinate
double Kp=0.2;
double Kd=0.2;

String point2; //set y coordinate
float Kp2 = 0.2;
float Kd2 = 0.2;

String point3; //set z coordinate
float Kp3 = 0.2;
float Kd3 = 0.2;

///////////////////////////////////////////////////////

//// PID 1 ////
float curr_error=0;   
float pid_term=0;
static int last_error=0;

//// PID 2 ////
float curr_error2=0;   
float pid_term2=0;
static int last_error2=0;

//// PID 3 ////
float curr_error3=0;   
float pid_term3=0;
static int last_error3=0;

float Px=18; //
float Py=0;
float Pz=7.75;

float theta1; // inverse angle
float theta2;
float theta3;
float theta2_1;
float theta2_2;
float theta3_1;
float theta3_2;

//// Dimension ////
const float length0=7.75;
const int length1=8;
const int length2=10;

float length4;
float alpha1;
float alpha2;
float alpha3;
float gamma;

void setup()
{
  Serial.begin(115200);

  //// MOTOR 1 ////
  pinMode(chA,INPUT);
  pinMode(chB,INPUT);
  attachInterrupt(0,changeA,CHANGE);
  attachInterrupt(1,changeB,CHANGE);
  
  pinMode(enA,OUTPUT);
  pinMode(in1,OUTPUT);
  pinMode(in2,OUTPUT);
  digitalWrite(chA,HIGH); //turn on pull up resistor
  digitalWrite(chB,HIGH);

  //// MOTOR 2 ////
  pinMode(chA_2,INPUT);
  pinMode(chB_2,INPUT);
  attachInterrupt(4,changeC,CHANGE);
  attachInterrupt(5,changeD,CHANGE);
  
  pinMode(enB,OUTPUT);
  pinMode(in1_2,OUTPUT);
  pinMode(in2_2,OUTPUT);
  digitalWrite(chA_2,HIGH); //turn on pull up resistor
  digitalWrite(chB_2,HIGH);

  //// MOTOR 3 ////
  pinMode(chA_3,INPUT);
  pinMode(chB_3,INPUT);
  attachInterrupt(2,changeE,CHANGE);
  attachInterrupt(3,changeF,CHANGE);
  
  pinMode(enA_2,OUTPUT);
  pinMode(in1_3,OUTPUT);
  pinMode(in2_3,OUTPUT);
  digitalWrite(chA_3,HIGH); //turn on pull up resistor
  digitalWrite(chB_3,HIGH);
  
  timep = millis();
}

void loop() 
{
  setpoint();
  inv_kinematics();
  
  PID_calc1();
  AngleCorrection1();
  PID_calc2();
  AngleCorrection2();
  PID_calc3();
  AngleCorrection3();
  
  printAngle();
  analogWrite(enA,150);
  analogWrite(enB,150);
  analogWrite(enA_2,150);
}

void setpoint()
{
  while (Serial.available()>0)
  {
    point  = Serial.readStringUntil(',');
    point2 = Serial.readStringUntil(',');
    point3 = Serial.readStringUntil('\0');
  }
    Px=point.toInt();
    Py=point2.toInt();
    Pz=point3.toInt();
}

void inv_kinematics()
{
  length4=abs(sqrt(pow(Px,2)+pow(Pz-7.75,2)));
  alpha1=(acos((pow(length1,2)+pow(length4,2)-pow(length2,2))/(2*length1*length4)))*57.2958;
  alpha2=(acos((pow(length1,2)+pow(length2,2)-pow(length4,2))/(2*length1*length2)))*57.2958;
  alpha3=(acos((pow(length2,2)+pow(length4,2)-pow(length1,2))/(2*length2*length4)))*57.2958;
  gamma=(atan2(Pz,Px))*57.2958;
  theta1=(atan2(Py,Px))*57.2958; //rad-to-deg
  theta2_1=(gamma+alpha1);
  theta2_2=(gamma-alpha1);
  theta3_1=((22/7)+alpha2);
  theta3_2=((22/7)-alpha2);

//check for workspace theta2//
  if (((cos(theta2_1)>=-1)&&(cos(theta2_1)<=1))&&((cos(theta2_2)>=-1)&&(cos(theta2_2)<=1)))
  {
    theta2=theta2_1;
  }
  if (((cos(theta2_1)<-1)||(cos(theta2_1)>1)))
  {
    theta2=theta2_2;
  }
  if (((cos(theta2_2)<-1)||(cos(theta2_2)>1)))
  {
    theta2=theta2_1;
  }
//check for workspace theta3//
  if (((cos(theta3_1)>=-1)&&(cos(theta3_1)<=1))&&((cos(theta3_2)>=-1)&&(cos(theta3_2)<=1)))
  {
    theta3=theta3_1;
  }
  if (((cos(theta3_1)<-1)||(cos(theta3_1)>1)))
  {
    theta2=theta3_2;
  }
  if (((cos(theta2_2)<-1)||(cos(theta2_2)>1)))
  {
    theta3=theta3_1;
  }

//set limit to theta2 and theta3//
  if (theta2>180) 
  {
   theta2=180;
  }
  if (theta2<-180) 
  {
   theta2=-180; 
  }

  if (theta3>180) 
  {
   theta3=180;
  }
  if (theta3<-180) 
  {
   theta3=-180; 
  }
 }

void PID_calc1()
{
  angle = (count * 0.15);
  curr_error = abs(theta1)-abs(angle);
  pid_term = (Kp*curr_error) + (Kd*(curr_error - last_error));
  
  last_error = curr_error;
  
  return constrain (pid_term, 0, 255);
}

void AngleCorrection1()
{
   if (angle < theta1) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    
  } else if (angle > theta1) 
  {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  }
  else
  {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, HIGH);
  }
}

void PID_calc2()
{
  angle2 = (count2 * 0.15);
  curr_error2 = abs(theta2)-abs(angle2);
  pid_term2 = (Kp2*curr_error2) + (Kd2*(curr_error2 - last_error2));
  
  last_error2 = curr_error2;
  
  return constrain (pid_term2, 0, 255);
}

void AngleCorrection2()
{
   if (angle2 < theta2) {
    digitalWrite(in1_2, LOW);
    digitalWrite(in2_2, HIGH);
    
  } else if (angle2 > theta2) 
  {
    digitalWrite(in1_2, HIGH);
    digitalWrite(in2_2, LOW);
  }
  else
  {
    digitalWrite(in1_2, HIGH);
    digitalWrite(in2_2, HIGH);
  }
}

void PID_calc3()
{
  angle3 = (count3 * 0.15);
  curr_error3 = abs(theta3)-abs(angle3);
  pid_term3 = (Kp3*curr_error3) + (Kd3*(curr_error3 - last_error3));
  
  last_error3 = curr_error3;
  
  return constrain (pid_term3, 0, 255);
}

void AngleCorrection3()
{
   if (angle3 < theta3) {
    digitalWrite(in1_3, LOW);
    digitalWrite(in2_3, HIGH);
    
  } else if (angle3 > theta3) 
  {
    digitalWrite(in1_3, HIGH);
    digitalWrite(in2_3, LOW);
  }
  else
  {
    digitalWrite(in1_3, HIGH);
    digitalWrite(in2_3, HIGH);
  }
}

void printAngle()
{
  time = millis();
  etime = time - timep;
  
  if (etime > 1000)
  {
    
    Serial.println("Count: " + String(count));
    Serial.println("Angle: " + String(angle));
    Serial.println("Count2: " + String(count2));
    Serial.println("Angle2: " + String(angle2));
    Serial.println("Count3: " + String(count3));
    Serial.println("Angle3: " + String(angle3));
    
    Serial.print("\n");
  
    timep = time;  
  }
}

void changeA() 
{
  A = digitalRead(20);
  B = digitalRead(21);

  if ((A==HIGH)&&(B==HIGH)) state = 1;
  if ((A==HIGH)&&(B==LOW)) state = 2;
  if ((A==LOW)&&(B==LOW)) state = 3;
  if((A==LOW)&&(B==HIGH)) state = 4;
  
  switch (state)
  {
    case 1:
    {
      if (statep == 2) count++;
      if (statep == 4) count--;
      break;
    }
    case 2:
    {
      if (statep == 1) count--;
      if (statep == 3) count++;
      break;
    }
    case 3:
    {
      if (statep == 2) count --;
      if (statep == 4) count ++;
      break;
    }
    
    default:
    {
      if (statep == 1) count++;
      if (statep == 3) count--;
    }
  }
  
  statep = state;
}

void changeB()
{
  A = digitalRead(20);
  B = digitalRead(21);

  if ((A==HIGH)&&(B==HIGH)) state = 1;
  if ((A==HIGH)&&(B==LOW)) state = 2;
  if ((A==LOW)&&(B==LOW)) state = 3;
  if((A==LOW)&&(B==HIGH)) state = 4;
  switch (state)
  {
    case 1:
    {
      if (statep == 2) count++;
      if (statep == 4) count--;
      break;
    }
    case 2:
    {
      if (statep == 1) count--;
      if (statep == 3) count++;
      break;
    }
    case 3:
    {
      if (statep == 2) count --;
      if (statep == 4) count ++;
      break;
    }
    
    default:
    {
      if (statep == 1) count++;
      if (statep == 3) count--;
    }
  }
  
  statep = state;
}

void changeC()
{
  C = digitalRead(18);
  D = digitalRead(19);

  if ((C==HIGH)&&(D==HIGH)) state2 = 1;
  if ((C==HIGH)&&(D==LOW)) state2 = 2;
  if ((C==LOW)&&(D==LOW)) state2 = 3;
  if((C==LOW)&&(D==HIGH)) state2 = 4;
  
  switch (state2)
  {
    case 1:
    {
      if (statep2 == 2) count2++;
      if (statep2 == 4) count2--;
      break;
    }
    case 2:
    {
      if (statep2 == 1) count2--;
      if (statep2 == 3) count2++;
      break;
    }
    case 3:
    {
      if (statep2 == 2) count2 --;
      if (statep2 == 4) count2 ++;
      break;
    }
    default:
    {
      if (statep2 == 1) count2++;
      if (statep2 == 3) count2--;
    }
  }
  
  statep2 = state2;
}

void changeD()
{
  C = digitalRead(18);
  D = digitalRead(19);

  if ((C==HIGH)&&(D==HIGH)) state2 = 1;
  if ((C==HIGH)&&(D==LOW)) state2 = 2;
  if ((C==LOW)&&(D==LOW)) state2 = 3;
  if((C==LOW)&&(D==HIGH)) state2 = 4;
  
  switch (state2)
  {
    case 1:
    {
      if (statep2 == 2) count2++;
      if (statep2 == 4) count2--;
      break;
    }
    case 2:
    {
      if (statep2 == 1) count2--;
      if (statep2 == 3) count2++;
      break;
    }
    case 3:
    {
      if (statep2 == 2) count2 --;
      if (statep2 == 4) count2 ++;
      break;
    }
    default:
    {
      if (statep2 == 1) count2++;
      if (statep2 == 3) count2--;
    }
  }
  
  statep2 = state2;  
}

void changeE()
{
  E = digitalRead(2);
  F = digitalRead(3);

  if ((E==HIGH)&&(F==HIGH)) state3 = 1;
  if ((E==HIGH)&&(F==LOW)) state3 = 2;
  if ((E==LOW)&&(F==LOW)) state3 = 3;
  if((E==LOW)&&(F==HIGH)) state3 = 4;
  
  switch (state3)
  {
    case 1:
    {
      if (statep3 == 2) count3++;
      if (statep3 == 4) count3--;
      break;
    }
    case 2:
    {
      if (statep3 == 1) count3--;
      if (statep3 == 3) count3++;
      break;
    }
    case 3:
    {
      if (statep3 == 2) count3 --;
      if (statep3 == 4) count3 ++;
      break;
    }
    default:
    {
      if (statep3 == 1) count3++;
      if (statep3 == 3) count3--;
    }
  }
  
  statep3 = state3;
}

void changeF()
{
  E = digitalRead(2);
  F = digitalRead(3);

  if ((E==HIGH)&&(F==HIGH)) state3 = 1;
  if ((E==HIGH)&&(F==LOW)) state3 = 2;
  if ((E==LOW)&&(F==LOW)) state3 = 3;
  if((E==LOW)&&(F==HIGH)) state3 = 4;
  
  switch (state3)
  {
    case 1:
    {
      if (statep3 == 2) count3++;
      if (statep3 == 4) count3--;
      break;
    }
    case 2:
    {
      if (statep3 == 1) count3--;
      if (statep3 == 3) count3++;
      break;
    }
    case 3:
    {
      if (statep3 == 2) count3 --;
      if (statep3 == 4) count3 ++;
      break;
    }
    default:
    {
      if (statep3 == 1) count3++;
      if (statep3 == 3) count3--;
    }
  }
  
  statep3 = state3;
}
