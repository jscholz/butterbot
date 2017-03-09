#include "stdio.h"

int ledPin = 13;      // LED connected to digital pin 9
int ain1 = 4;      // 
int ain2 = 5;      // 
int pwmA = 6;      // motor voltage
//int val = 0;         // variable to store the read value

int pwmV = 0;
int a1V = 1;
int a2V = 0;

char buf[32];

void setup()
{    
  pinMode(ledPin, OUTPUT);   // sets the pin as output
  //pinMode(pwmA, OUTPUT);
  Serial.begin(9600); 
}

void loop()
{
  // read the sensor:
  if (Serial.available() > 0) {
    int inByte = Serial.read();
    // do something different depending on the character received.  
    // The switch statement expects single number values for each case;
    // in this exmaple, though, you're using single quotes to tell
    // the controller to get the ASCII value for the character.  For 
    // example 'a' = 97, 'b' = 98, and so forth:
    switch (inByte) {
    case '+':   
      pwmV++; 
      sprintf(buf,"Setting pwm voltage to %d", pwmV);
      Serial.println(buf);
      analogWrite(pwmA, pwmV);
      break;
    case '-':    
      pwmV--; 
      sprintf(buf,"Setting pwm voltage to %d", pwmV);
      Serial.println(buf);
      analogWrite(pwmA, pwmV);
      break;
    case '1':
      if (a1V == 0) 
        a1V = 255;
      else
        a1V = 0;
      sprintf(buf,"Setting a1 voltage to %d", a1V);
      Serial.println(buf);
      analogWrite(ain1, a1V);
      break;
    case '2':    
      if (a2V == 0) 
        a2V = 255;
      else
        a2V = 0;
      sprintf(buf,"Setting a2 voltage to %d", a2V);
      Serial.println(buf);
      analogWrite(ain2, a2V);
      break;
    default:
      // whatever
      break;
    } 
  }
}



