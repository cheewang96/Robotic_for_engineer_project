#include <math.h>
#include <PID_v1.h>

/////////////////// motor driver 2 ///////////////////

const int EN2 = 10; //motor 1
const int a2 = 9;
const int b2 = 8;
const int interruptA2 = 19;
const int interruptB2 = 18;

boolean channelA2; //encoder 1
boolean channelB2;

volatile double pulse2 = 0;
volatile double angle2 = 0;

byte state2, prestate2;


double Kp2 = 0.3;// you can set these constants however you like depending on trial & error
double Ki2 = 0;
double Kd2 = 0;

float Error2 = 0;
float lastError2 = 0;
float changeError2 = 0;
float totalError2 = 0;

float gain2 = 0;
float pwmGain2 = 0;
float pwm2 = 0;

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

  ///////////////////// motor 2 //////////////////////////
  pinMode(EN2, OUTPUT);
  pinMode(a2, OUTPUT);  //high => -
  pinMode(b2, OUTPUT);  //high => +
  ////////////////////////////////////////////////////////

  //////////////////////////// encoder 2 ///////////////////////////////////
  pinMode(interruptA2, INPUT_PULLUP);
  pinMode(interruptB2, INPUT_PULLUP);
  digitalWrite(interruptA2, HIGH);
  digitalWrite(interruptB2, HIGH);
  attachInterrupt(digitalPinToInterrupt(interruptA2), changeA2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(interruptB2), changeB2, CHANGE);
  ///////////////////////////////////////////////////////////////////////////
}



void loop()
{

  digitalWrite(EN2, LOW);

  /*if (Serial.available() > 0)
  {


    INanglex = Serial.readString();
    

    Serial.print(INanglex);
    Serial.print('\n');

  }*/

  angley = -80;
  PID2();
  angleCorrectionY();
  
  Serial.print(angle2);
  Serial.print('\t');
  Serial.print(angley);
  Serial.print('\t');
  Serial.print(gain2);
  Serial.print('\t');
  Serial.print(pwm2);
  Serial.print('\t');
  Serial.println(digitalRead(interruptA2));Serial.print('\t');
  Serial.println(digitalRead(interruptB2));Serial.print('\t');
  Serial.println(digitalRead(EN2));Serial.print('\t');
  Serial.println(digitalRead(a2));Serial.print('\t');
  Serial.println(digitalRead(b2));Serial.print('\t');

  Serial.println(pulse2);
}





//////////// interrupt encoder 2 channel A ///////////////////////////////

void changeA2()
{
  channelA2 = digitalRead(interruptA2);
  channelB2 = digitalRead(interruptB2);

  if ((channelA2 == LOW) && (channelB2 == LOW)) state2 = 1;
  if ((channelA2 == LOW) && (channelB2 == HIGH)) state2 = 2;
  if ((channelA2 == HIGH) && (channelB2 == HIGH)) state2 = 3;
  if ((channelA2 == HIGH) && (channelB2 == LOW)) state2 = 4;
  switch (state2)
  {
    case 1:
      {
        if (prestate2 == 2) pulse2++;
        if (prestate2 == 4) pulse2--;
        break;
      }
    case 2:
      {
        if (prestate2 == 1) pulse2--;
        if (prestate2 == 3) pulse2++;
        break;
      }
    case 3:
      {
        if (prestate2 == 2) pulse2--;
        if (prestate2 == 4) pulse2++;
        break;
      }
    default:
      {
        if (prestate2 == 1) pulse2++;
        if (prestate2 == 3) pulse2--;
      }
  }
  prestate2 = state2;
}
//////////////////////////////////////////////////////////////////////////

//////////// interrupt encoder 2 channel B ///////////////////////////////

void changeB2()
{
  channelA2 = digitalRead(interruptA2);
  channelB2 = digitalRead(interruptB2);

  if ((channelA2 == LOW) && (channelB2 == LOW)) state2 = 1;
  if ((channelA2 == LOW) && (channelB2 == HIGH)) state2 = 2;
  if ((channelA2 == HIGH) && (channelB2 == HIGH)) state2 = 3;
  if ((channelA2 == HIGH) && (channelB2 == LOW)) state2 = 4;
  switch (state2)
  {
    case 1:
      {
        if (prestate2 == 2) pulse2++;
        if (prestate2 == 4) pulse2--;
        break;
      }
    case 2:
      {
        if (prestate2 == 1) pulse2--;
        if (prestate2 == 3) pulse2++;
        break;
      }
    case 3:
      {
        if (prestate2 == 2) pulse2--;
        if (prestate2 == 4) pulse2++;
        break;
      }
    default:
      {
        if (prestate2 == 1) pulse2++;
        if (prestate2 == 3) pulse2--;
      }
  }
  prestate2 = state2;
}
//////////////////////////////////////////////////////////////////////////

//////////////////////////////////////// PID 2 /////////////////////////////////////////////
void PID2() //PID 2
{
  angle2 = 0.15*pulse2;        
  /* for SPG30E-200K, gear ratio = 1:200, 3 pulse for 1 rear shaft revolution,   */
  /* 1 pulse rotate main shaft for 200 degree, 3 pulses = 3(200) = 600 degree of main shaft */
  /* 4 phase involved = 4 counts, 4(600) = 2400 counts per main shaft revolution   */
  /* degree = (360/2400)x pulse1 = (0.15)pulse2 */ 

  Error2 = angley - angle2;             // => P
  totalError2 += Error2;                // => I
  changeError2 = lastError2 - Error2;   // => D

  gain2 = (Kp2 * Error2) + (Ki2 * totalError2) + (Kd2 * changeError2); // total gain
  pwmGain2 = constrain(gain2, -255, 255);                 // limit the value to -255 and 255
  pwm2 = abs(pwmGain2);                                   // make it positive value
  lastError2 = Error2;                                    //post-define last error
}
/////////////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////// angleCorrectionY ///////////////////////////////////////////
void angleCorrectionY()      //correct Y motor
{
  float dangle2 = angley - angle2;
  
  if (dangle2 >= 0.26)       // need ccw // angley> angle2
  {
    digitalWrite(EN2,pwmGain2);
    digitalWrite(b2, LOW);
    digitalWrite(a2, HIGH);
    delay(20);
    digitalWrite(b2, LOW);
    digitalWrite(a2, LOW);
  }
  else if (dangle2 <= -0.26)          // need cw // angley < angle2
  {
    digitalWrite(EN2, pwmGain2);
    digitalWrite(a2, LOW);
    digitalWrite(b2, HIGH);
    delay(20);
    digitalWrite(b2, LOW);
    digitalWrite(a2, LOW);
  }
  else              // off motor
  { 
    delay(2);
    digitalWrite(EN2, LOW);
    digitalWrite(a2, LOW);
    digitalWrite(b2, LOW);
  }
}
/////////////////////////////////////////////////////////////////////////////////////////////
