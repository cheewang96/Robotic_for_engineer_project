#include <math.h>
#include <SoftwareSerial.h>
#include <Servo.h>
#define BLYNK_PRINT Serial
#include <BlynkSimpleSerialBLE.h>
char auth[] = "BfxiZqXhlXezs7APkSlZw1e8RTbXvY3-";

int x;
int d1; int d2; int d3;
float px; float py; float pz;
int servopin = 4; Servo servo1;

//////////////////////////////////// motor driver 1 ////////////////////////////////////
//motor 1//
const int EN1 = 13; const int a1 = 12; const int b1 = 11;
const int interruptA1 = 20; const int interruptB1 = 21;

//encoder 1//
boolean channelA1; boolean channelB1;
volatile double pulse1 = 0; volatile double angle1 = 0;
byte state1, prestate1;

//PID tuning based on trial & error
double Kp = 0.3; double Ki = 0.05; double Kd = 0.05;
float Error1 = 0; float lastError1 = 0; float changeError1 = 0; float totalError1 = 0;
float gain1 = 0; float pwmGain1 = 0; float pwm1 = 0;
////////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////// motor driver 2 ////////////////////////////////////
//motor 2//
const int EN2 = 10;  const int a2 = 9; const int b2 = 8;
const int interruptA2 = 19; const int interruptB2 = 18;

//encoder 2//
boolean channelA2; boolean channelB2;
volatile double pulse2 = 0; volatile double angle2 = 0;
byte state2,prestate2;

//PID tuning based on trial & error
double Kp2 = 0.3; double Ki2 = 0.05; double Kd2 = 0.05;
float Error2 = 0; float lastError2 = 0; float changeError2 = 0; float totalError2 = 0;
float gain2 = 0; float pwmGain2 = 0; float pwm2 = 0;
///////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////// motor driver 3 ///////////////////////////////////
//motor 3//
const int EN3 = 7;  const int a3 = 6; const int b3 = 5;
const int interruptA3 = 2; const int interruptB3 = 3;

//encoder 1//
boolean channelA3; boolean channelB3;
volatile double pulse3 = 0; volatile double angle3 = 0;
byte state3,prestate3;

//PID tuning based on trial & error
double Kp3 = 0.3; double Ki3 = 0.1; double Kd3 = 0.1;
float Error3 = 0; float lastError3 = 0; float changeError3 = 0; float totalError3 = 0;
float gain3 = 0; float pwmGain3 = 0; float pwm3 = 0;
///////////////////////////////////////////////////////////////////////////////////////

//////////////////// Read Serial ////////////////////

String INanglex;
String INangley;
String INanglez;
double anglex = 0;
double angley = 0;
double anglez = 0;

/////////////////////////////////////////////////////

void setup()
{ 
  Serial.begin(9600);
  
  ////////////// motor 1 //////////////
  pinMode(EN1, OUTPUT);
  pinMode(a1, OUTPUT);  //high => -
  pinMode(b1, OUTPUT);  //high => +
  ////////////////////////////////////
  
  ////////////// motor 2 //////////////
  pinMode(EN2, OUTPUT);
  pinMode(a2,OUTPUT);   //high => -
  pinMode(b2,OUTPUT);   //high => +
  /////////////////////////////////////

  ////////////// motor 3 //////////////
  pinMode(EN3, OUTPUT);
  pinMode(a3,OUTPUT);   //high => -
  pinMode(b3,OUTPUT);   //high => +
  ///////////////////////////////////// 

  //////////// servo /////////////
  servo1.attach (servopin);
  ///////////////////////////////

  ////////////////////////////// encoder 1 //////////////////////////////
  pinMode(interruptA1, INPUT); pinMode(interruptB1, INPUT);
  digitalWrite(interruptA1, HIGH); digitalWrite(interruptB1, HIGH);
  attachInterrupt(digitalPinToInterrupt(interruptA1),changeA1,CHANGE);
  attachInterrupt(digitalPinToInterrupt(interruptB1),changeB1,CHANGE);
  ///////////////////////////////////////////////////////////////////////

  ////////////////////////////// encoder 2 //////////////////////////////
  pinMode(interruptA2, INPUT); pinMode(interruptB2, INPUT);
  digitalWrite(interruptA2, HIGH); digitalWrite(interruptB2, HIGH);
  attachInterrupt(digitalPinToInterrupt(interruptA2),changeA2,CHANGE);
  attachInterrupt(digitalPinToInterrupt(interruptB2),changeB2,CHANGE);
  ///////////////////////////////////////////////////////////////////////
  
  ////////////////////////////// encoder 3 //////////////////////////////
  pinMode(interruptA3, INPUT); pinMode(interruptB3, INPUT);
  digitalWrite(interruptA3, HIGH); digitalWrite(interruptB3, HIGH);
  attachInterrupt(digitalPinToInterrupt(interruptA3),changeA3,CHANGE);
  attachInterrupt(digitalPinToInterrupt(interruptB3),changeB3,CHANGE);
  ///////////////////////////////////////////////////////////////////////
 }


void loop()
{
  
  digitalWrite(EN1, LOW); digitalWrite(EN2, LOW); digitalWrite(EN3, LOW);
 
  /*if (Serial.available() > 0)
  {
      while (Serial.available() > 0) 
      {
      set = Serial.readStringUntil(',');
      Serial.read();
      set2 = Serial.readStringUntil(',');
      Serial.read();
      set3 = Serial.readStringUntil('\0');
      }
      */
    anglex = 0; 
    angley = 0;
    anglez = 0;
    servo1.write(50);
  //}*/

  ////////////////////// motor controlling //////////////////////
  p_position();
  PID1(); PID2(); PID3();
  angleCorrectionX(); angleCorrectionY(); angleCorrectionZ();
  ///////////////////////////////////////////////////////////////

  ////////////////////// print //////////////////////
  Serial.print("[A1 A2 A3] = [");
  Serial.print(angle1); Serial.print(", ");
  Serial.print(angle2); Serial.print(", ");
  Serial.print(angle3); Serial.print("]\n");
  
  Serial.print("[Px Py Pz] = [");
  Serial.print(px); Serial.print(",");
  Serial.print(py); Serial.print(",");
  Serial.print(pz); Serial.print("]\n");

  Serial.print ("[Ax Ay Az] = [");
  Serial.print(anglex); Serial.print(",");
  Serial.print(angley); Serial.print(",");
  Serial.print(anglez); Serial.print("]\n");
  //////////////////////////////////////////////////
  
}


/////////////////////////////////////// Forward Kinematic position ///////////////////////////////////////
void p_position()
{
  d1 = angle1*0.0174533; d2 = angle2*0.0174533; d3 = angle3*0.0174533; //deg-to-rad
  px = 10*sin(d1)+8*cos(d1)*cos(d2); //update px py pz
  py = 8*sin(d1)*cos(d2)-10*cos(d1);
  pz = 8*sin(d2)+7.75;
  Serial.println(px); Serial.println(py); Serial.println(pz); 
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////


/////////////////////////// interrupt encoder 1 channel A //////////////////////////////
void changeA1()
{
  channelA1 = digitalRead(interruptA1); channelB1 = digitalRead(interruptB1);

  if ((channelA1 == LOW) && (channelB1 == LOW)) state1 = 1;
  if ((channelA1 == LOW) && (channelB1 == HIGH)) state1 = 2;
  if ((channelA1 == HIGH) && (channelB1 == HIGH)) state1 = 3;
  if ((channelA1 == HIGH) && (channelB1 == LOW)) state1 = 4;
  
  switch (state1)
  {
    case 1:{ if (prestate1 == 2) pulse1++; if (prestate1 == 4) pulse1--; break; }
    case 2:{ if (prestate1 == 1) pulse1--; if (prestate1 == 3) pulse1++; break; }
    case 3:{ if (prestate1 == 2) pulse1--; if (prestate1 == 4) pulse1++; break; }
    default:{ if (prestate1 == 1) pulse1++; if (prestate1 == 3) pulse1--; }
  }
  
  prestate1 = state1;
}
////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////// interrupt encoder 1 channel B /////////////////////////////
void changeB1()
{
  channelA1 = digitalRead(interruptA1); channelB1 = digitalRead(interruptB1);

  if ((channelA1 == LOW) && (channelB1 == LOW)) state1 = 1;
  if ((channelA1 == LOW) && (channelB1 == HIGH)) state1 = 2;
  if ((channelA1 == HIGH) && (channelB1 == HIGH)) state1 = 3;
  if ((channelA1 == HIGH) && (channelB1 == LOW)) state1 = 4;
  
  switch (state1)
  {
    case 1:{ if (prestate1 == 2) pulse1++; if (prestate1 == 4) pulse1--; break; }
    case 2:{ if (prestate1 == 1) pulse1--; if (prestate1 == 3) pulse1++; break; }
    case 3:{ if (prestate1 == 2) pulse1--; if (prestate1 == 4) pulse1++; break; }
    default:{ if (prestate1 == 1) pulse1++; if (prestate1 == 3) pulse1--; }
  }  
 prestate1 = state1;
}
///////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////// PID 1 /////////////////////////////////////////
void PID1()
{
  angle1 = 0.15 * pulse1; 
  /* for SPG30E-200K, gear ratio = 1:200, 3 pulse for 1 rear shaft revolution,   */
  /* 1 pulse rotate main shaft for 200 degree, 3 pulses = 3(200) = 600 degree of main shaft */
  /* 4 phase involved = 4 counts, 4(600) = 2400 counts per main shaft revolution   */
  /* degree = (360/2400)x pulse1 = (0.15)pulse1 */
  
  Error1 = anglex - angle1;             // => P of the PID
  totalError1 += Error1;                // => I of the PID
  changeError1 = lastError1 - Error1;   // => D of the PID

  gain1 = (Kp * Error1) + (Ki * totalError1) + (Kd * changeError1); // total gain
  pwmGain1 = constrain(gain1, -255, 255); // limit the value to -255 and 255
  pwm1 = abs(pwmGain1); // make it positive value

  lastError1 = Error1; //post-define the last error 

}
/////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////// correct x motor //////////////////////////////////////
void angleCorrectionX()
{
  float dangle = anglex - angle1;
  Serial.print ("dangle =");
  Serial.print (dangle); Serial.print ("\n");

  //angle x > angle 1 , thus ccw
  if (dangle >= 0.26)
  {
    digitalWrite(EN1,pwmGain1);
    digitalWrite(b1, LOW); digitalWrite(a1, HIGH);
    delay(10);
    digitalWrite(b1, LOW); digitalWrite(a1, LOW);
  }

  // angle x < angle 1, thus cw
  else if (dangle <= 0.26)
  {
    digitalWrite(EN1,pwmGain1);
    digitalWrite(a1, LOW); digitalWrite(b1, HIGH);
    delay(10);
    digitalWrite(b1, LOW); digitalWrite(a1, LOW);
  }

  //off motor
  else
  { 
    delay(2); digitalWrite(EN1, LOW);
    digitalWrite(a1, LOW); digitalWrite(b1, LOW);
  }
}
/////////////////////////////////////////////////////////////////////////////////////



///////////////////////////// interrupt encoder 2 channel A /////////////////////////////
void changeA2()
{
  channelA2 = digitalRead(interruptA2); channelB2 = digitalRead(interruptB2);
  
  if ((channelA2==LOW)&&(channelB2==LOW)) state2 = 1;
  if ((channelA2==LOW)&&(channelB2==HIGH)) state2 = 2;
  if ((channelA2==HIGH)&&(channelB2==HIGH)) state2 = 3;
  if ((channelA2==HIGH)&&(channelB2==LOW)) state2 = 4;
  
  switch (state2)
  {
   case 1:{ if (prestate2 == 2) pulse2++; if (prestate2 == 4) pulse2--; break; }
   case 2:{ if (prestate2 == 1) pulse2--; if (prestate2 == 3) pulse2++; break; }
   case 3:{ if (prestate2 == 2) pulse2--; if (prestate2 == 4) pulse2++; break; }
   default:{ if (prestate2 == 1) pulse2++; if (prestate2 == 3) pulse2--; }
  }
 prestate2 = state2;
}
/////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////// interrupt encoder 2 channel B /////////////////////////////
void changeB2()
{
  channelA2 = digitalRead(interruptA2); channelB2 = digitalRead(interruptB2);
  
  if ((channelA2==LOW)&&(channelB2==LOW)) state2 = 1;
  if ((channelA2==LOW)&&(channelB2==HIGH)) state2 = 2;
  if ((channelA2==HIGH)&&(channelB2==HIGH)) state2 = 3;
  if((channelA2==HIGH)&&(channelB2==LOW)) state2 = 4;
  
  switch (state2)
  { case 1: { if (prestate2 == 2) pulse2++; if (prestate2 == 4) pulse2--; break; }
    case 2: { if (prestate2 == 1) pulse2--; if (prestate2 == 3) pulse2++; break; }
    case 3: { if (prestate2 == 2) pulse2--; if (prestate2 == 4) pulse2++; break; }
    default: { if (prestate2 == 1) pulse2++; if (prestate2 == 3) pulse2--; }
  }
  prestate2 = state2;
}
/////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////// PID 2 /////////////////////////////////////////
void PID2()
{
  angle2 = 0.15 * pulse2; 
  /* for SPG30E-200K, gear ratio = 1:200, 3 pulse for 1 rear shaft revolution,   */
  /* 1 pulse rotate main shaft for 200 degree, 3 pulses = 3(200) = 600 degree of main shaft */
  /* 4 phase involved = 4 counts, 4(600) = 2400 counts per main shaft revolution   */
  /* degree = (360/2400)x pulse1 = (0.15)pulse1 */
                                
  Error2 = angley - angle2;             // => P of the PID
  totalError2 += Error2;                // => I of the PID
  changeError2 = lastError2 - Error2;   // => D of the PID

  gain2 = (Kp2 * Error2) + ( Ki2 * totalError2 ) + ( Kd2 * changeError2 ); // total gain
  pwmGain2 = constrain(gain2,-255,255); // limit the value to -255 and 255
  pwm2 = abs(pwmGain2); // make it positive value

  lastError2 = Error2; //  post-define last error
}
/////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////// correct y motor///////////////////////////////////////
void angleCorrectionY()
{
  float dangle2 = angley - angle2;

  // angle y > angle 2, thus ccw
  if (dangle2 >= 0.26)
  {
    digitalWrite(EN2,pwmGain2);
    digitalWrite(b2, LOW); digitalWrite(a2, HIGH);
    delay(10);
    digitalWrite(b2, LOW); digitalWrite(a2, LOW);
  }

  // angle y < angle 2, thus cw
  else if (dangle2 <= -0.26)
  {
    digitalWrite(EN2,pwmGain2);
    digitalWrite(a2, LOW); digitalWrite(b2, HIGH);
    delay(10);
    digitalWrite(b2, LOW); digitalWrite(a2, LOW);
  }

  // off motor 
  else
  { 
    delay(2); digitalWrite(EN2,LOW);
    digitalWrite(a2, LOW); digitalWrite(b2, LOW);
  }
}
//////////////////////////////////////////////////////////////////////////////////////

/////////////////////////// interrupt encoder 3 channel A ///////////////////////////

void changeA3()
{
  channelA3 = digitalRead(interruptA3); channelB3 = digitalRead(interruptB3);
  
  if ((channelA3==LOW)&&(channelB3==LOW)) state3 = 1;
  if ((channelA3==LOW)&&(channelB3==HIGH)) state3 = 2;
  if ((channelA3==HIGH)&&(channelB3==HIGH)) state3 = 3;
  if((channelA3==HIGH)&&(channelB3==LOW)) state3 = 4;
  switch (state3)
  { 
    case 1:{ if (prestate3 == 2) pulse3++; if (prestate3 == 4) pulse3--; break; }
    case 2:{ if (prestate3 == 1) pulse3--; if (prestate3 == 3) pulse3++; break; }
    case 3:{ if (prestate3 == 2) pulse3--; if (prestate3 == 4) pulse3++; break; }
    default:{ if (prestate3 == 1) pulse3++; if (prestate3 == 3) pulse3--; }
  }
 prestate3 = state3;
}
/////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////// interrupt encoder 3 channel B ///////////////////////////
void changeB3()
{
  channelA3 = digitalRead(interruptA3); channelB3 = digitalRead(interruptB3);
  
  if ((channelA3==LOW)&&(channelB3==LOW)) state3 = 1;
  if ((channelA3==LOW)&&(channelB3==HIGH)) state3 = 2;
  if ((channelA3==HIGH)&&(channelB3==HIGH)) state3 = 3;
  if((channelA3==HIGH)&&(channelB3==LOW)) state3 = 4;
  
  switch (state3)
  {
   case 1:{ if (prestate3 == 2) pulse3++; if (prestate3 == 4) pulse3--; break; } 
   case 2:{ if (prestate3 == 1) pulse3--; if (prestate3 == 3) pulse3++; break; }
   case 3:{ if (prestate3 == 2) pulse3--; if (prestate3 == 4) pulse3++; break; }
   default:{ if (prestate3 == 1) pulse3++; if (prestate3 == 3) pulse3--; }
  }
 prestate3 = state3;
}
/////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////// PID 3 /////////////////////////////////////////
void PID3() 
{
  angle3 = 0.15 * pulse3; 
  /* for SPG30E-200K, gear ratio = 1:200, 3 pulse for 1 rear shaft revolution,   */
  /* 1 pulse rotate main shaft for 200 degree, 3 pulses = 3(200) = 600 degree of main shaft */
  /* 4 phase involved = 4 counts, 4(600) = 2400 counts per main shaft revolution   */
  /* degree = (360/2400)x pulse1 = (0.15)pulse1 */
                                
  Error3 = anglez - angle3;             // => P of the PID
  totalError3 += Error3;                // => I of the PID
  changeError3 = lastError3 - Error3;   // => D of the PID

  gain3 = (Kp3 * Error3) + (Ki3 * totalError3) + (Kd3 * changeError3); // total gain
  pwmGain3 = constrain(gain3,-255,255); // limit the value to -255 and 255
  pwm3 = abs(pwmGain3); // make it positive value

  lastError3 = Error3; //post-define last error
}
/////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////// correct z motor///////////////////////////////////////
void angleCorrectionZ()
{
  float dangle3 = anglez - angle3;

  // angle z > angle 3, thus ccw
  if (dangle3 >= 0.26)
  {
    analogWrite(EN3,pwmGain3);
    digitalWrite(b3, LOW); digitalWrite(a3, HIGH);
    delay(10);
    digitalWrite(b3, LOW); digitalWrite(a3, LOW);
  }

  // angle z < angle 3, thus cw
  else if (dangle3 <= -0.26)
  {
    analogWrite(EN3,pwmGain3);
    digitalWrite(a3, LOW); digitalWrite(b3, HIGH);
    delay(10);
    digitalWrite(b3, LOW); digitalWrite(a3, LOW);
  }

  // off motor 
  else
  { 
    delay(2); analogWrite(EN3,LOW);
    digitalWrite(a3, LOW); digitalWrite(b3, LOW);
  }
}
///////////////////////////////////////////////////////////////////////////////////////////
