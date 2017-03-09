#include <Servo.h> 

char buf[32];
int sensorPin = 4; // select the input pin for the ranger
int sensorValue;   // variable to store the value coming from the sensor

Servo myservo;     // create servo object to control a servo 
int servopin = 6;  
int pos = 0;       // servo position, in degrees
int waitDur = 15;  // how long to wait for servo to move (ms)
int npts = 15;

// LED stuff...
int m = -2;
int b = 500;
int ctr = 0;
int pollfreq = 10;
int ledState = 0;
int ledPin = 12;      // select the pin for the LED

void setup() {
  // initialize serial port
  Serial.begin(9600);

  myservo.attach(servopin);  // attaches the servo on pin 9 to the servo object 

  // declare the ledPin as an OUTPUT:
  pinMode(ledPin, OUTPUT);
  pinMode(servopin, OUTPUT); // redundant? I dunno how servo lib works
}

void proxBlinker()
{
  if (ctr > (int)(m*sensorValue)+b) {
    ctr = 0;
    if (ledState == 0) {
      digitalWrite(ledPin, HIGH);
      ledState = 1;
    } 
    else {
      digitalWrite(ledPin, LOW);
      ledState = 0;
    }
  } 
  else {
    ctr += pollfreq;
  }
}

void tryPWM()
{
  int pw=900; // pulse width in us
  int period=20000; // pwm period in us
  int i=0;
  while (1)  {
    digitalWrite(servopin, HIGH);
    delayMicroseconds(pw);
    digitalWrite(servopin, LOW);
    delayMicroseconds(period-pw);
    ++i;
    if (i>700) {
      pw+=100;
      sprintf(buf,"pw = %d",pw);
      Serial.println(buf);
      i=0;
      if (pw>2100)
        pw = 900;
    }
  }
}

void loop() {

  /* manual tinkering with pwm signal to control full-rotation servo */
  tryPWM();
  
  /* Manual control of servo position, blinkey otherwise */
//  if (Serial.available() > 0) {
//    int inByte = Serial.read();
//    switch (inByte) {
//    case '+':   
//      pos++;
//      myservo.write(pos);                  // tell servo to go to position in variable 'pos' 
//      delay(waitDur);                           // waits 15ms for the servo to reach the position 
//      sensorValue = analogRead(sensorPin); // read the value from the sensor:
//      sprintf(buf,"%d %d",pos, sensorValue);
//      Serial.println(buf);      
//      break;
//    case '-':    
//      pos--;
//
//      myservo.write(pos);                  // tell servo to go to position in variable 'pos' 
//      delay(waitDur);                      // waits 15ms for the servo to reach the position 
//      sensorValue = analogRead(sensorPin); // read the value from the sensor:
//      sprintf(buf,"%d %d",pos, sensorValue);
//      Serial.println(buf);  
//      break;
//    default:
//      break;
//    }
//  }
//  sensorValue = analogRead(sensorPin); // read the value from the sensor:
//  delay(waitDur);                           // waits 15ms for the servo to reach the position 
//  proxBlinker();

  // 5 sweeps like radar-style
  //  if (Serial.available() > 0) {
  //    int inByte = Serial.read();
  //    switch (inByte) {
  //    case '+':   
  //      for (int i=0; i<5; ++i){
  //        for (int t=30; t<180; ++t) {
  //          myservo.write(t);                  // tell servo to go to position in variable 'pos' 
  //          delay(waitDur);                      // waits 15ms for the servo to reach the position 
  //
  //          int ptsum = 0;
  //          for (int x=0; x<npts; ++x){
  //            ptsum += analogRead(sensorPin); // read the value from the sensor:
  //            delay(1);
  //          }
  //
  //          sprintf(buf,"%d %d",t, (int)ptsum/npts);
  //          Serial.println(buf);  
  //        }
  //        for (int t=180; t>30; --t) {
  //          myservo.write(t);                  // tell servo to go to position in variable 'pos' 
  //          delay(waitDur);                      // waits 15ms for the servo to reach the position 
  //
  ////          int ptsum = 0;
  ////          for (int x=0; x<npts; ++x){
  ////            ptsum += analogRead(sensorPin); // read the value from the sensor:
  ////            delay(1);
  ////          }
  ////
  ////          sprintf(buf,"%d %d",t, (int)ptsum/npts);
  ////          Serial.println(buf);  
  //        }
  //      }
  //    default:
  //      break;
  //    }
  //  }

}


















