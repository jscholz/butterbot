// Sweep
// by BARRAGAN <http://barraganstudio.com> 
#include <Servo.h> 

Servo myservo;  // create servo object to control a servo 
int servopin = 6;
char buf[32];

int pos = 0;    // variable to store the servo position 

void setup() 
{ 
  Serial.begin(9600); 
  myservo.attach(servopin);  // attaches the servo on pin 9 to the servo object 
} 

/* To just loop over the positions 0:180:0 */
//void loop() 
//{ 
//  for(pos = 0; pos < 180; pos += 1)  // goes from 0 degrees to 180 degrees 
//  {                                  // in steps of 1 degree 
//    sprintf(buf,"setting pos %d",pos);
//    Serial.println(buf);
//    myservo.write(pos);              // tell servo to go to position in variable 'pos' 
//    delay(15);                       // waits 15ms for the servo to reach the position 
//  } 
//
//  for(pos = 180; pos>=1; pos-=1)     // goes from 180 degrees to 0 degrees 
//  {                                
//    sprintf(buf,"setting pos %d",pos);
//    Serial.println(buf);
//    myservo.write(pos);              // tell servo to go to position in variable 'pos' 
//    delay(15);                       // waits 15ms for the servo to reach the position 
//  } 
//} 

/* To set positions manually */
void loop() { 
  if (Serial.available() > 0) {
    int inByte = Serial.read();
    switch (inByte) {
    case '+':   
      pos++;
      sprintf(buf,"Setting position to %d degrees", pos);
      Serial.println(buf);
      break;
    case '-':    
      pos--;
      sprintf(buf,"Setting position to %d degrees", pos);
      Serial.println(buf);
      break;
    default:
      break;
    }
  }

  myservo.write(pos);              // tell servo to go to position in variable 'pos' 
  delay(15);                       // waits 15ms for the servo to reach the position 
} 



