/* Manual PWM without delay()
 
 Turns on and off a digital pin to generate a PWM signal, without 
 using the delay() function.  This means that other code
 can run at the same time without being interrupted by the PWM code.
 
 Instead of using delay() calls, this code uses millis/micros calls
 to check the time since the Arduino began running the current program.
 This allows the pwm call to be non-blocking, but at the risk of a 
 missed deadline for switching states.  
 
 Currently implemented to provide high resolution by using micros() 
 rather than millis().  
 
 created 2009
 by Jon Scholz
 */

// {
#define PWMPERIOD 20000  // period (20 ms)

/* Pulse Width, in microseconds 
 *   -Must make this a long if a period longer than 32ms is desired!
 */
unsigned int pw = 0;

/* Container for time variable
 *   -Must make this a long if a period longer than 32ms is desired!
 */
unsigned int previousMicros;

/* Current state of the PWM pin
 */
int pwmState = LOW;
// }

unsigned int pstep = 100;  // pulse width step in microseconds

char buf[32];

/*
frequency is 1/PWMPERIOD Hz
 
 */

void setup() {
  Serial.begin(9600); 
  // {
  pinMode(6, OUTPUT);
  previousMicros = micros();
  // }
}

void loop() {

  if (Serial.available() > 0) {
    int inByte = Serial.read();

    switch (inByte) {
    case '+':   
      pw += pstep; 
      sprintf(buf,"Setting pulse width to %d us", pw);
      Serial.println(buf);
      break;
    case '-':    
      pw -= pstep; 
      sprintf(buf,"Setting pulse width to %d us", pw);
      Serial.println(buf);
      break;
    case '1':
      pstep -= 50;
      sprintf(buf,"Setting pulse step to %d", pstep);
      Serial.println(buf);
      break;
    case '2':    
      pstep += 50;
      sprintf(buf,"Setting pulse step to %d", pstep);
      Serial.println(buf);
      break;
    default:
      // whatever
      break;
    } 
  }

  pulseOut(6, pw);

}

void pulseOut(int pinNumber, int pulseWidth) {
  /* Uses a millis() call to check clock so that we can run
   * a non-blocking PWM (delay calls hold up everything 
   * except interrupt-driven code)
   */
   static int interval = 0;

  if (micros() - previousMicros > interval) {
    previousMicros = micros();   
    if (pwmState == LOW) {
      pwmState = HIGH;
      interval = pulseWidth;
    } else {
      pwmState = LOW;
      interval = PWMPERIOD - pulseWidth;
    }
    digitalWrite(pinNumber, pwmState);
  }
}

