#include <math.h>
#include <PID_v1.h>

/////////////////// motor driver 1 ///////////////////

const int EN1 = 13; //motor 1
const int a1 = 12;
const int b1 = 11;
const int interruptA1 = 20;
const int interruptB1 = 21;

boolean channelA1; //encoder 1
boolean channelB1;

volatile double pulse1 = 0;
volatile double angle1 = 0;

byte state1, prestate1;


double Kp1 = 0.3;// you can set these constants however you like depending on trial & error
double Ki1 = 0;
double Kd1 = 0;

float Error1 = 0;
float lastError1 = 0;
float changeError1 = 0;
float totalError1 = 0;

float gain1 = 0;
float pwmGain1 = 0;
float pwm1 = 0;

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

  ///////////////////// motor 1 //////////////////////////
  pinMode(EN1, OUTPUT);
  pinMode(a1, OUTPUT);  //high => -
  pinMode(b1, OUTPUT);  //high => +
  ////////////////////////////////////////////////////////

  //////////////////////////// encoder 1 ///////////////////////////////////
  pinMode(interruptA1, INPUT_PULLUP);
  pinMode(interruptB1, INPUT_PULLUP);
  digitalWrite(interruptA1, HIGH);
  digitalWrite(interruptB1, HIGH);
  attachInterrupt(digitalPinToInterrupt(interruptA1), changeA1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(interruptB1), changeB1, CHANGE);
  ///////////////////////////////////////////////////////////////////////////
}



void loop()
{

  digitalWrite(EN1, LOW);

  /*if (Serial.available() > 0)
  {


    INanglex = Serial.readString();
    

    Serial.print(INanglex);
    Serial.print('\n');

  }*/

  anglex = -90;
  PID1();
  angleCorrectionX();
  
  Serial.print(angle1);
  Serial.print('\t');
  Serial.print(anglex);
  Serial.print('\t');
  Serial.print(gain1);
  Serial.print('\t');
  Serial.print(pwm1);
  Serial.print('\t');
  Serial.println(digitalRead(interruptA1));Serial.print('\t');
  Serial.println(digitalRead(interruptB1));Serial.print('\t');
  Serial.println(digitalRead(EN1));Serial.print('\t');
  Serial.println(digitalRead(a1));Serial.print('\t');
  Serial.println(digitalRead(b1));Serial.print('\t');

  Serial.println(pulse1);
}





//////////// interrupt encoder 1 channel A ///////////////////////////////

void changeA1()
{
  channelA1 = digitalRead(interruptA1);
  channelB1 = digitalRead(interruptB1);

  if ((channelA1 == LOW) && (channelB1 == LOW)) state1 = 1;
  if ((channelA1 == LOW) && (channelB1 == HIGH)) state1 = 2;
  if ((channelA1 == HIGH) && (channelB1 == HIGH)) state1 = 3;
  if ((channelA1 == HIGH) && (channelB1 == LOW)) state1 = 4;
  switch (state1)
  {
    case 1:
      {
        if (prestate1 == 2) pulse1++;
        if (prestate1 == 4) pulse1--;
        break;
      }
    case 2:
      {
        if (prestate1 == 1) pulse1--;
        if (prestate1 == 3) pulse1++;
        break;
      }
    case 3:
      {
        if (prestate1 == 2) pulse1--;
        if (prestate1 == 4) pulse1++;
        break;
      }
    default:
      {
        if (prestate1 == 1) pulse1++;
        if (prestate1 == 3) pulse1--;
      }
  }
  prestate1 = state1;
}
//////////////////////////////////////////////////////////////////////////

//////////// interrupt encoder 1 channel B ///////////////////////////////

void changeB1()
{
  channelA1 = digitalRead(interruptA1);
  channelB1 = digitalRead(interruptB1);

  if ((channelA1 == LOW) && (channelB1 == LOW)) state1 = 1;
  if ((channelA1 == LOW) && (channelB1 == HIGH)) state1 = 2;
  if ((channelA1 == HIGH) && (channelB1 == HIGH)) state1 = 3;
  if ((channelA1 == HIGH) && (channelB1 == LOW)) state1 = 4;
  switch (state1)
  {
    case 1:
      {
        if (prestate1 == 2) pulse1++;
        if (prestate1 == 4) pulse1--;
        break;
      }
    case 2:
      {
        if (prestate1 == 1) pulse1--;
        if (prestate1 == 3) pulse1++;
        break;
      }
    case 3:
      {
        if (prestate1 == 2) pulse1--;
        if (prestate1 == 4) pulse1++;
        break;
      }
    default:
      {
        if (prestate1 == 1) pulse1++;
        if (prestate1 == 3) pulse1--;
      }
  }
  prestate1 = state1;
}
//////////////////////////////////////////////////////////////////////////

//////////////////////////////////////// PID 1 /////////////////////////////////////////////
void PID1() //PID 1
{
  angle1 = 0.15*pulse1;        
  /* for SPG30E-200K, gear ratio = 1:200, 3 pulse for 1 rear shaft revolution,   */
  /* 1 pulse rotate main shaft for 200 degree, 3 pulses = 3(200) = 600 degree of main shaft */
  /* 4 phase involved = 4 counts, 4(600) = 2400 counts per main shaft revolution   */
  /* degree = (360/2400)x pulse1 = (0.15)pulse1 */ 

  Error1 = anglex - angle1;             // => P
  totalError1 += Error1;                // => I
  changeError1 = lastError1 - Error1;   // => D

  gain1 = (Kp1 * Error1) + (Ki1 * totalError1) + (Kd1 * changeError1); // total gain
  pwmGain1 = constrain(gain1, -255, 255);                 // limit the value to -255 and 255
  pwm1 = abs(pwmGain1);                                   // make it positive value
  lastError1 = Error1;                                    //post-define last error
}
/////////////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////// angleCorrectionX ///////////////////////////////////////////
void angleCorrectionX()      //correct X motor
{
  float dangle1 = anglex - angle1;
  
  if (dangle1 >= 0.26)       // need ccw // anglex > angle1
  {
    digitalWrite(EN1,pwmGain1);
    digitalWrite(b1, LOW);
    digitalWrite(a1, HIGH);
    delay(20);
    digitalWrite(b1, LOW);
    digitalWrite(a1, LOW);
  }
  else if (dangle1 <= -0.26)          // need cw // anglex > angle1
  {
    digitalWrite(EN1, pwmGain1);
    digitalWrite(a1, LOW);
    digitalWrite(b1, HIGH);
    delay(20);
    digitalWrite(b1, LOW);
    digitalWrite(a1, LOW);
  }
  else              // off motor
  { 
    delay(2);
    digitalWrite(EN1, LOW);
    digitalWrite(a1, LOW);
    digitalWrite(b1, LOW);
  }
}
/////////////////////////////////////////////////////////////////////////////////////////////
