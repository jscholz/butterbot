#include "stdio.h"
#include "string.h"

/* These are hardcoded values for "horizontal" pulled from sensor data.
 * Really, we should be estimating these values online based on either:
 *  - the values that keep drift minimized (requires vision or gps)
 *  - the values during a user-marked window of time where the helicopter is horizontal
 */
#define XGOAL 527  // goal analog read value for roll 
#define YGOAL 508  // goal analog read value for pitch 
#define ZGOAL 596  // goal analog read value for yaw
#define GYROSTILL 500   // fix me
#define MB_MAXSIZE 8   // max buffer size for accumulating multi-byte commands

// Motor Params
int forwardPin = 10;   // tail motor up
int backwardPin = 11;  // tail motor down
int rotor1Pin = 6;        // main motor values (up/down)
int rotor2Pin = 9;        // main motor values (up/down)
int forwardVal;
int backwardVal;
int rotor1Val;
int rotor2Val;
int mainVal = 0;  // main thrust multiplier
int lrMid = 50;
int lrProp = lrMid;     // left vs. right proportion: adjusted by (lrProp - lrMid) in each direction

// Sensor Params
int accelPinZ = 0; // Yaw
int accelPinY = 1; // Roll
int accelPinX = 2; // Pitch
int gyroPin1 = 3;
int gyroPin2 = 4;
int accelZ; // Yaw
int accelY; // Roll
int accelX; // Pitch
int gyroVal1;
int gyroVal2;

// Control Params
int pwmIncr = 10;   // amount to increment up pwm value on keypress
boolean runPitchController = false;
boolean runYawController = false;
boolean printData = false;
int pgain = 1;   // proportion gain (this is a divisor, not a multiplier)
int dgain = 1;  // derivative gain (also a divisor)

// IO Params
char buf[64];
boolean verbose = false;
boolean multiRead = false;
char multibuf[MB_MAXSIZE];

void setup()
{    
  Serial.begin(9600);
  //Serial.begin(115200);
}


void writeVals()
{
  analogWrite(forwardPin, forwardVal);
  analogWrite(backwardPin, backwardVal);
  rotor1Val = mainVal + lrProp - lrMid;
  rotor2Val = mainVal + lrMid - lrProp;
  analogWrite(rotor1Pin, rotor1Val);
  analogWrite(rotor2Pin, rotor2Val);
  
  if (verbose) {
    sprintf(buf,"main,lr = [%d %d]; r1,r2,f,b = [%d %d %d %d]",mainVal,lrProp,rotor1Val,rotor2Val,forwardVal,backwardVal);
    Serial.println(buf);
  }
}

void readSensorData()
{
  gyroVal1 = analogRead(gyroPin1);
  gyroVal2 = analogRead(gyroPin2);

  accelX = analogRead(accelPinX);
  accelY = analogRead(accelPinY);
  accelZ = analogRead(accelPinZ);
}

void printSensorData()
{
  sprintf(buf,"[%d %d %d %d %d]",accelX, accelY, accelZ, gyroVal1, gyroVal2); // arducom version
  //sprintf(buf,"%d %d %d %d %d",accelX, accelY, accelZ, gyroVal1, gyroVal2); // gnuplot version
  Serial.println(buf); 
}

void multiByteHandler()
{
  /**
   * Handle the commands passed in as multi-byte strings.
   *
   * Checks for the presence of predefined escape characters, and acts accordingly.
   * Currently set up to handle 1 command at a time, indicated by a 1-char identifier.
   * e.g. a command to set the main thrust value from a laptop might be 'm120!m' 
   */
   
   /**
    * Codes:
    * !: up
    * @: down (currently same as up, i.e. we're in spring-loaded mode)
    * #: left
    * $: right (currently same as left, so laptop handles axis logic for steering)
    * %: forward
    * ^: backward
    */
   int len = strlen(multibuf);
   Serial.println(multibuf[len]);
   switch (multibuf[len]) {
     case '!': // thrust
       multibuf[len-1] = '\0'; // trim command char
       mainVal = atoi(multibuf);
     sprintf(buf,"mainVal=",mainVal);
     Serial.println(buf);
       rotor1Val = mainVal + lrProp - lrMid;
       rotor2Val = mainVal + lrMid - lrProp;
       writeVals();
       break;
     case '@': // thrust
       multibuf[len-1] = '\0'; // trim command char
       mainVal = atoi(multibuf);
     sprintf(buf,"mainVal=",mainVal);
     Serial.println(buf);
       rotor1Val = mainVal + lrProp - lrMid;
       rotor2Val = mainVal + lrMid - lrProp;
       writeVals();
       break;
     case '#': // yaw
       multibuf[len-1] = '\0'; // trim command char
       lrProp = atoi(multibuf);
     sprintf(buf,"lrProp=",lrProp);
     Serial.println(buf);
       rotor1Val = mainVal + lrProp - lrMid;
       rotor2Val = mainVal + lrMid - lrProp;
       writeVals();
       break;
     case '$': // yaw
       multibuf[len-1] = '\0'; // trim command char
       lrProp = atoi(multibuf);
     sprintf(buf,"lrProp=",lrProp);
     Serial.println(buf);
       rotor1Val = mainVal + lrProp - lrMid;
       rotor2Val = mainVal + lrMid - lrProp;
       writeVals();
       break;
     case '%': // forward
       multibuf[len-1] = '\0'; // trim command char
       forwardVal = atoi(multibuf);
     sprintf(buf,"forwardVal=",forwardVal);
     Serial.println(buf);
       backwardVal = 0;
       writeVals();
       break;
     case '^': // backward
       multibuf[len-1] = '\0'; // trim command char
       backwardVal = atoi(multibuf);
     sprintf(buf,"backwardVal=",backwardVal);
     Serial.println(buf);
       forwardVal = 0;
       writeVals();
       break;       
     default:
       break;
   }
}

void pitchControl(int offset)
{
  /**
   * A controller for the tail motor that tries to keep the helicopter pitch at <offset> degrees
   * 
   * Implemented as a PD controller that reads the accelerometer (and eventually some gyros) 
   * and estimates the e
   * Compute a PWM value in PD-fashion - with an error and two gains
   * -the proportional gain attempts to minimize the error
   * -the derivative gain is attempts to minimize the velocity, as estimated with discrete samples
   *
   * @param an offset for the error calculations - pass in a nonzero int to drive
   * forwards or backwards
   */

  /**
   * Proportional terms
   * Can do this directly on the axis values, or can run on the actual pose of the helicopter
   * Pose can be reasonably calculated in 2D since we don't have controll of roll anyway
   *  e.g. pitch = 
   *  
   * analog values DECREASE for increasing values on their axis
   */
  int xError = (XGOAL - offset - accelX);  // G vector along X indicates rotation about Y
  int yError = (YGOAL - offset - accelY);  // G vector along Y indicates rotation about X
  int zError = (ZGOAL - offset - accelZ);  // no one cares about Z unless we experience high vertical accelerations

  /* Currently just using a discrete estimate of rotational velocity since we don't know
   * how the gyros will be mounted.  
   * This should definitely be replaced with actual gyro readings when we get the hardware worked out
   *
   * Don't get confused: xVelocity is NOT velocity about the x-axis.  It's the rate of change in the 
   * acceleration along the x-axis, which indicates rotation about the y-axis (or actual jerk
   * in the x-direction...)
   */
  static int oldX;
  static int oldY;
  static int oldZ;
  int xVel = abs(accelX - oldX);  
  int yVel = abs(accelY - oldY);  
  int zVel = abs(accelZ - oldZ);  
  oldX = accelX;
  oldY = accelY;
  oldZ = accelZ;

  /**
   * Compute a PD control, using the gains as divisors
   * 
   */
  int tailMotorVal = (xError * pgain) - (xVel * dgain); // switched to accelerometer's x-axis since we're mounting sideways

  if (verbose) {
    sprintf(buf, "[XY]error = [%3.d,%3.d] # [XY]vel = [%3.d,%3.d] # tailMotorVal = %3.d", xError, yError, xVel, yVel, tailMotorVal);
    Serial.println(buf);
  }

  /**
   * Writes to the appropriate pitch motor pin 
   * constrain values to fall within pwm range {0:255}
   */
  if (tailMotorVal >= 0){
    analogWrite(backwardPin, 0); 
    analogWrite(forwardPin, constrain(tailMotorVal, 0 , 255)); 
  } 
  else {
    analogWrite(forwardPin, 0); 
    analogWrite(backwardPin, constrain(-tailMotorVal, 0 , 255)); 
  }
}

void yawControl(int offset)
{
  /**
   * A controller for the tail motor that tries to keep the helicopter pitch at <offset> degrees
   * 
   * Implemented as a PD controller that reads the accelerometer (and eventually some gyros) 
   * and estimates the e
   * Compute a PWM value in PD-fashion - with an error and two gains
   * -the proportional gain attempts to minimize the error
   * -the derivative gain is attempts to minimize the velocity, as estimated with discrete samples
   *
   * @param an offset for the error calculations - pass in a nonzero int to drive
   * forwards or backwards
   */

  /**
   * Proportional terms
   * Can do this directly on the axis values, or can run on the actual pose of the helicopter
   * Pose can be reasonably calculated in 2D since we don't have controll of roll anyway
   *  e.g. pitch = 
   *  
   * analog values DECREASE for increasing values on their axis
   */
  int yError = GYROSTILL - gyroVal1;  // gyro value compared to still reading

  /* Currently just using a discrete estimate of rotational velocity since we don't know
   * how the gyros will be mounted.  
   * This should definitely be replaced with actual gyro readings when we get the hardware worked out
   *
   * Don't get confused: xVelocity is NOT velocity about the x-axis.  It's the rate of change in the 
   * acceleration along the x-axis, which indicates rotation about the y-axis (or actual jerk
   * in the x-direction...)
   */
  //  static int oldX;
  //  static int oldY;
  //  static int oldZ;
  //  int xVel = abs(accelX - oldX);  
  //  int yVel = abs(accelY - oldY);  
  //  int zVel = abs(accelZ - oldZ);  
  //  oldX = accelX;
  //  oldY = accelY;
  //  oldZ = accelZ;

  /**
   * Compute a PD control, using the gains as divisors
   * 
   */
  //  int tailMotorVal = (xError * pgain) - (xVel * dgain); // switched to accelerometer's x-axis since we're mounting sideways

  //  sprintf(buf, "[XY]error = [%3.d,%3.d] # [XY]vel = [%3.d,%3.d] # tailMotorVal = %3.d", xError, yError, xVel, yVel, tailMotorVal);
  //  Serial.println(buf);

  /**
   * Writes to the appropriate pitch motor pin 
   * constrain values to fall within pwm range {0:255}
   */
  //  if (tailMotorVal >= 0){
  //    analogWrite(backwardPin, 0); 
  //    analogWrite(forwardPin, constrain(tailMotorVal, 0 , 255)); 
  //  } 
  //  else {
  //    analogWrite(forwardPin, 0); 
  //    analogWrite(backwardPin, constrain(-tailMotorVal, 0 , 255)); 
  //  }
}

void loop()
{
  // read the sensor:
  readSensorData();

  // print if requested:
  if (printData)
    printSensorData();

  if (runPitchController)
    pitchControl(0); // Eventually this should be passed the forward/backward vals as the offset

  if (runYawController)
    yawControl(0); // Eventually this should be passed the forward/backward vals as the offset

  if (Serial.available() > 0) {
    int inByte = Serial.read(); 
    if (multiRead) {
      if (inByte == 'm') {
        multiRead = false;
        sprintf(buf,"multiRead = %d", multiRead);
        Serial.println(buf);
      } 
      else {
        // Append to multibyte buffer
        int last = strlen(multibuf);
        if (last > MB_MAXSIZE-1) 
          Serial.println("multibuf is full (and I don't feel like rotating it)");
        else {
          sprintf(multibuf,"%s%c", multibuf, inByte);
          //Serial.println(multibuf);
        }
        // Call multibyte handler
        multiByteHandler();
      }
    } 
    else {
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
      case 'v':
        verbose = !verbose;
        sprintf(buf,"verbose = %d", verbose);
        Serial.println(buf);
        break;      
      case 'e':
        mainVal += pwmIncr;
        rotor1Val = mainVal + lrProp - lrMid;
        rotor2Val = mainVal + lrMid - lrProp;
        writeVals();
        break;
      case 'q':
        mainVal -= pwmIncr;
        rotor1Val = mainVal + lrProp - lrMid;
        rotor2Val = mainVal + lrMid - lrProp;
        writeVals();
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
        lrProp--;
        rotor1Val = mainVal + lrProp - lrMid;
        rotor2Val = mainVal + lrMid - lrProp;
        writeVals();
        break;
      case 'd':
        lrProp++;
        rotor1Val = mainVal + lrProp - lrMid;
        rotor2Val = mainVal + lrMid - lrProp;
        writeVals();
        break;
      case 'l':
        printData = !printData;
        break;
      case '.':
        pgain++;
        sprintf(buf,"Setting pgain to %d", pgain);
        Serial.println(buf);
        break;
      case ',':
        pgain--;
        sprintf(buf,"Setting pgain to %d", pgain);
        Serial.println(buf);
        break;
      case '>':
        dgain++;
        sprintf(buf,"Setting dgain to %d", dgain);
        Serial.println(buf);
        break;
      case '<':
        dgain--;
        sprintf(buf,"Setting dgain to %d", dgain);
        Serial.println(buf);
        break;
      case 'p':
        runPitchController = !runPitchController;
        sprintf(buf,"runPitchController = %d", runPitchController);
        Serial.println(buf);
        break;
      case 'y':
        runYawController = !runYawController;
        sprintf(buf,"runYawController = %d", runYawController);
        Serial.println(buf);
        break;
      case 'm':
        /**
         * Multibyte command: used for passing custom commands or escape sequences
         * Hitting 'm' will toggle this mode, which if true will cause all serial 
         * input to be logged to a single buffer (max size = MB_MAXSIZE)
         * During each loop if the boolean multiRead is true, the function handleMulti()
         * will be called.
         * This approach provides an elegant way to read compliated commands without 
         * interrupting the primary control loop or resorting to interrupts
         */
        multiRead = true;
        sprintf(buf,"multiRead = %d", multiRead);
        Serial.println(buf);
        multibuf[0] = '\0'; // reset multibuf if entering multiRead mode
        break;
      default:
        // whatever
        Serial.println("Resetting all values");
        forwardVal = 0;
        backwardVal = 0;
        mainVal = 0;
        lrProp = lrMid;
        writeVals();
        break;
      } 
    }
  }
  else {
    // Set all voltages to zero
  }
}











