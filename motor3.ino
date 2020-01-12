#include <math.h>
#include <PID_v1.h>

/////////////////// motor driver 3 ///////////////////

const int EN3 = 7; //motor 3
const int a3 = 6;
const int b3 = 5;
const int interruptA3 = 2;
const int interruptB3 = 3;

boolean channelA3; //encoder 3
boolean channelB3;

volatile double pulse3 = 0;
volatile double angle3 = 0;

byte state3, prestate3;


double Kp3 = 0.3;// you can set these constants however you like depending on trial & error
double Ki3 = 0;
double Kd3 = 0;

float Error3 = 0;
float lastError3 = 0;
float changeError3 = 0;
float totalError3 = 0;

float gain3 = 0;
float pwmGain3 = 0;
float pwm3 = 0;

/////////////////////////////////////////////////////

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
  /*TCCR1B = TCCR1B & 0b11111000 | 1;  // set 31KHz PWM to prevent motor noise
  myPID.SetMode(AUTOMATIC);   //set PID in Auto mode
  myPID.SetSampleTime(1);  // refresh rate of PID controller
  myPID.SetOutputLimits(-125, 125); // this is the MAX PWM value to move motor, here change in value reflect change in speed of motor.
  */

  ///////////////////// motor 3 //////////////////////////
  pinMode(EN3, OUTPUT);
  pinMode(a3, OUTPUT);  //high => -
  pinMode(b3, OUTPUT);  //high => +
  ////////////////////////////////////////////////////////

  //////////////////////////// encoder 3 ///////////////////////////////////
  pinMode(interruptA3, INPUT_PULLUP);
  pinMode(interruptB3, INPUT_PULLUP);
  digitalWrite(interruptA3, HIGH);
  digitalWrite(interruptB3, HIGH);
  attachInterrupt(digitalPinToInterrupt(interruptA3), changeA3, CHANGE);
  attachInterrupt(digitalPinToInterrupt(interruptB3), changeB3, CHANGE);
  ///////////////////////////////////////////////////////////////////////////
}



void loop()
{

  digitalWrite(EN3, LOW);

  /*if (Serial.available() > 0)
  {


    INanglex = Serial.readString();
    

    Serial.print(INanglex);
    Serial.print('\n');

  }*/

  anglez = 150;
  PID3();
  angleCorrectionZ();
  
  Serial.print(angle3);
  Serial.print('\t');
  Serial.print(anglez);
  Serial.print('\t');
  Serial.print(gain3);
  Serial.print('\t');
  Serial.print(pwm3);
  Serial.print('\t');
  Serial.println(digitalRead(interruptA3));Serial.print('\t');
  Serial.println(digitalRead(interruptB3));Serial.print('\t');
  Serial.println(digitalRead(EN3));Serial.print('\t');
  Serial.println(digitalRead(a3));Serial.print('\t');
  Serial.println(digitalRead(b3));Serial.print('\t');

  Serial.println(pulse3);
}





//////////// interrupt encoder 3 channel A ///////////////////////////////

void changeA3()
{
  channelA3 = digitalRead(interruptA3);
  channelB3 = digitalRead(interruptB3);

  if ((channelA3 == LOW) && (channelB3 == LOW)) state3 = 1;
  if ((channelA3 == LOW) && (channelB3 == HIGH)) state3 = 2;
  if ((channelA3 == HIGH) && (channelB3 == HIGH)) state3 = 3;
  if ((channelA3 == HIGH) && (channelB3 == LOW)) state3 = 4;
  switch (state3)
  {
    case 1:
      {
        if (prestate3 == 2) pulse3++;
        if (prestate3 == 4) pulse3--;
        break;
      }
    case 2:
      {
        if (prestate3 == 1) pulse3--;
        if (prestate3 == 3) pulse3++;
        break;
      }
    case 3:
      {
        if (prestate3 == 2) pulse3--;
        if (prestate3 == 4) pulse3++;
        break;
      }
    default:
      {
        if (prestate3 == 1) pulse3++;
        if (prestate3 == 3) pulse3--;
      }
  }
  prestate3 = state3;
}
//////////////////////////////////////////////////////////////////////////

//////////// interrupt encoder 3 channel B ///////////////////////////////

void changeB3()
{
  channelA3 = digitalRead(interruptA3);
  channelB3 = digitalRead(interruptB3);

  if ((channelA3 == LOW) && (channelB3 == LOW)) state3 = 1;
  if ((channelA3 == LOW) && (channelB3 == HIGH)) state3 = 2;
  if ((channelA3 == HIGH) && (channelB3 == HIGH)) state3 = 3;
  if ((channelA3 == HIGH) && (channelB3 == LOW)) state3 = 4;
  switch (state3)
  {
    case 1:
      {
        if (prestate3 == 2) pulse3++;
        if (prestate3 == 4) pulse3--;
        break;
      }
    case 2:
      {
        if (prestate3 == 1) pulse3--;
        if (prestate3 == 3) pulse3++;
        break;
      }
    case 3:
      {
        if (prestate3 == 2) pulse3--;
        if (prestate3 == 4) pulse3++;
        break;
      }
    default:
      {
        if (prestate3 == 1) pulse3++;
        if (prestate3 == 3) pulse3--;
      }
  }
  prestate3 = state3;
}
//////////////////////////////////////////////////////////////////////////

//////////////////////////////////////// PID 3 /////////////////////////////////////////////
void PID3() //PID 3
{
  angle3 = 0.15*pulse3;        
  /* for SPG30E-200K, gear ratio = 1:200, 3 pulse for 1 rear shaft revolution,   */
  /* 1 pulse rotate main shaft for 200 degree, 3 pulses = 3(200) = 600 degree of main shaft */
  /* 4 phase involved = 4 counts, 4(600) = 2400 counts per main shaft revolution   */
  /* degree = (360/2400)x pulse3 = (0.15)pulse3 */ 

  Error3 = anglez - angle3;             // => P
  totalError3 += Error3;                // => I
  changeError3 = lastError3 - Error3;   // => D

  gain3 = (Kp3 * Error3) + (Ki3 * totalError3) + (Kd3 * changeError3); // total gain
  pwmGain3 = constrain(gain3, -255, 255);                 // limit the value to -255 and 255
  pwm3 = abs(pwmGain3);                                   // make it positive value
  lastError3 = Error3;                                    //post-define last error
}
/////////////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////// angleCorrectionZ ///////////////////////////////////////////
void angleCorrectionZ()      //correct Z motor
{
  float dangle3 = anglez - angle3;
  
  if (dangle3 >= 0.26)       // need ccw // anglez > angle3
  {
    digitalWrite(EN3,pwmGain3);
    digitalWrite(b3, LOW);
    digitalWrite(a3, HIGH);
    delay(20);
    digitalWrite(b3, LOW);
    digitalWrite(a3, LOW);
  }
  else if (dangle3 <= -0.26)          // need cw // anglez > angle3
  {
    digitalWrite(EN3, pwmGain3);
    digitalWrite(a3, LOW);
    digitalWrite(b3, HIGH);
    delay(20);
    digitalWrite(b3, LOW);
    digitalWrite(a3, LOW);
  }
  else              // off motor
  { 
    delay(2);
    digitalWrite(EN3, LOW);
    digitalWrite(a3, LOW);
    digitalWrite(b3, LOW);
  }
}
/////////////////////////////////////////////////////////////////////////////////////////////
