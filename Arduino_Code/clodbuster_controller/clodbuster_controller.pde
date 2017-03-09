/* Controller for "Clod Buster" R/C car:
 * This code should drive a rear brush motor and 
 * a single front servo for steering
 * 
 * Requires 2 pins for motor (forward and backwards), and
 * 2 pins for steering (prop. left and right)
 *
 *
 *
 */


#include "stdio.h"
#define WHEELCENTER 48 // best guess at the "maintain value"  .76 - .96 - 1.22 in V, 38 - 49 - 62 in pwm (hopefully...)
// *We'll have to continuously estimate this if it turns out to be unstable...
#define RCTOGGLE_PIN 8
#define forwardPin 10
#define backwardPin 11
#define turnPin 9
#define lightbluePin 2
#define purplePin 1
#define blackPin 0
#define rangerPin 5

int forwardVal = 0;
int backwardVal = 0;
int turnVal = WHEELCENTER;  
int lbVal; // light blue wire
int bVal;  // black wire
int pVal;  // purple wire
int goalT = 50;  // desired wheel position on range 0:100, with 50 being straight ahead
int pgain = 5; // proportion gain (this is a dividend, not a multiplier)
int dgain = 40;  // derivative gain - should be greater than 1 or else it will cancel the P term
int rcState = LOW;
int pwmIncr = 3;   // amount to increment up pwm value on keypress

boolean printData = false;
boolean runServoController = true;
boolean runIRController = true;

int rangerVal;

char buf[64];

void writeVals()
{
  analogWrite(forwardPin, forwardVal);
  analogWrite(backwardPin, backwardVal);

  sprintf(buf,"f,b,t = [%d %d %d]",forwardVal, backwardVal, goalT);
  Serial.println(buf);
}

void turnControl()
{
  static int oldPos;
  int vel = abs(lbVal - oldPos); // discrete velocity estimate
  oldPos = lbVal;

  int goalPos = bVal + ((long int)(goalT)*(long int)(pVal) - (long int)(goalT)*(long int)(bVal)) / 100;
  int error = -1*(goalPos - lbVal); // need to reverse error due to how we're tricking control circuitry

  //turnVal = constrain(turnVal + error/pgain, 20, 100); // should be all the range we need

  /* Compute a PWM value in PD-fashion - with an error and two gains
   * -the proportional gain attempts to minimize the error
   * -the derivative gain is attempts to minimize the velocity, as estimated with discrete samples
   */
  turnVal = constrain(WHEELCENTER + error/pgain - constrain(vel/dgain,0,100), 20, 100); // should be all the range we need

  //    sprintf(buf,"curpos = %d, goalpos = %d, error = %d, vel = %d, turnVal = %d", lbVal, goalPos, error, vel, turnVal);
  //    Serial.println(buf);
  //    delay(30);

  //    if (printData)
  analogWrite(turnPin, turnVal); // turnVal
}

void printSensorData()
{
  sprintf(buf,"%d %d %d %d",lbVal, bVal, pVal, rangerVal);
  Serial.println(buf); 
}

void setup()
{    
  Serial.begin(9600); 
  pinMode(RCTOGGLE_PIN,OUTPUT);
  writeVals();
}

void loop()
{
  // read the encoder position:
  lbVal = analogRead(lightbluePin);
  bVal = analogRead(blackPin);
  pVal = analogRead(purplePin);

  // read the IR sensor
  rangerVal = analogRead(rangerPin);
  int rangerDist = 500-rangerVal; //TODO: should code in the actual function sometime

  if (Serial.available() > 0) {
    int inByte = Serial.read();
    /*
    Eventually, this should be set up to only set pin values when a key is held
     down, and always reset otherwise.  I don't know if it can handle multiple 
     simultaneous keypresses though, so for now i'll treat them all as increments
     */
    switch (inByte) {
    case '+':   
      pwmIncr++; 
      sprintf(buf,"Setting pwmIncr to %d", pwmIncr);
      Serial.println(buf);
      break;
    case '-':    
      pwmIncr--; 
      sprintf(buf,"Setting pwmIncr to %d", pwmIncr);
      Serial.println(buf);
      break;
    case 'w':
      backwardVal = 0;
      forwardVal += pwmIncr;
      writeVals();
      break;
    case 's':
      forwardVal = 0;
      backwardVal += pwmIncr;
      writeVals();
      break;
    case 'a':
      //goalT--;
      goalT -= pwmIncr;
      writeVals();
      break;
    case 'd':
      //goalT++;
      goalT += pwmIncr;
      writeVals();
      break;
    case 'r':
      if (rcState == LOW)
        rcState = HIGH;
      else
        rcState = LOW;
      sprintf(buf,"rcState = %d", rcState);
      Serial.println(buf);
      digitalWrite(RCTOGGLE_PIN, rcState);
      break;
    case 't':
      runServoController = !runServoController;
      sprintf(buf,"runServoController = %d", runServoController);
      Serial.println(buf);
      break;
    case 'g':
      runIRController = !runIRController;
      sprintf(buf,"runIRController = %d", runIRController);
      Serial.println(buf);
      break;
    case 'p':
      printData = !printData;
      break;
    default:
      // whatever
      Serial.println("Resetting all values");
      forwardVal = 0;
      backwardVal = 0;
      goalT = 50;
      writeVals();
      break;
    } 
  }
  else {
    // whatever
  }



  if (runServoController)
    turnControl();

  if (runIRController) {
    // Stop if we're about to hit something
    if (rangerDist < 150) {
      forwardVal = 0;
      backwardVal = 0;
      writeVals();
      /* CLODBUG algorithm: servo along a wall, keeping the distance in a fixed range.
       * -If break in wall detected, turn into it
       */
    } else {
      forwardVal = 60;
      writeVals();
    }
  }


  if (printData)
    printSensorData();
}


//TODO :
/* need some sort of controller on the PWM for steering.  The desired position will be maintained as an int on some scale,
 and the code will increase or decrease the voltage (via pulse width) according to the error between this position and the 
 value returned by the light blue wire (perhaps adjusted for range between black and purple)
 
 * read in blue and calibrate against black and purple (don't forget to aref this against board's high)
 ** TURNRES is the resolution for the turning position comparison.  Values greater than 1023 are meaningless 
 because the analogRead() resolution is 0-1023
 curPos = map(lightBlueVal, blackVal, purpleVal, 0, 1023);  // this maps the current encoder position to somewhere in a fixed range
 
 // can then compare curPos to goalPos, and map the output back to a pwm
 
 blerg.
 * need to know where VCC is on scale of 0-1023 with aref as rc board's high 
 ** from here we can do something like this:
 outPWM = map(
 (this is all about calculating the expected value of the encoder for a desired position, and then computing the PWM signal
 necessary to produce this voltage.  To move we'd set the reference line (lightBlue out) to the negative of this value, which
 would trick the computer into moving the wheel... but this is all wrong.  First, it assumes we know the function relating pwm
 and voltage, and second it assumes that we get a position by setting the encoder to what we want that position to be  (or its 
 netative).  This logic is busted because the RC controller is broadcasting a home value all the time which is enabled even
 if the feedback line is cut.  Thus, to set a position we need to trick it into moving the right direction (with some gain)
 and then holding by setting the encoder line to the value it would have if it was exactly where the controller wanted it 
 to be.  Perhaps the trick to finding this value is to do an analogRead() on lightBlue WITHOUT aref-ing it, with the hopes that
 it would be less than 5V.  Mapping this number from the range of 1024 to 256 would give us the exact voltage (assuming linear
 pwm function...) to write to the lightBlue out.  This should work as long as we take our reading with the encoder line closed.
 
 * if this doesn't work (analytically finding the wheel-maintain value), we can try finding it with the controller.  Given an
 encoder value and a reference, we can just calculate a P term and either:
 1) hope that it decays naturally
 2) force it to with a V term
 3) force it to by decaying it everytime the error switches signs
 
 ** check the voltage across lb->black with rc off at various wheel positions:
 Does it vary directly with position?  Does it represent the error between pos and some internal offset?
 
 ** need to find the "maintain" signal.  See if this forces the wheel to stay in place regardless of where it is.  
 If so, then all i have to do is deviate in either direction long enough to move to a goal, and collapse back to
 the maintain value to hold position.  
 */




