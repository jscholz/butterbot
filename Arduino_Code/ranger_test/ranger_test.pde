#include <stdio.h>

int sensorPin = 4;    // select the input pin for the potentiometer
int ledPin = 13;      // select the pin for the LED
int led2Pin = 12;      // select the pin for the LED
int sensorValue = 0;  // variable to store the value coming from the sensor

int m = -2;
int b = 500;
int ctr = 0;
int ledState;
int pollfreq = 100;
char buf[32];

void setup() {
  // initialize serial port
  Serial.begin(9600);

  // declare the ledPin as an OUTPUT:
  pinMode(ledPin, OUTPUT);  
  pinMode(led2Pin, OUTPUT);  
}

void loop() {
  // read the value from the sensor:
  sensorValue = analogRead(sensorPin);    
  delay(pollfreq);

  sprintf(buf,"%d",sensorValue);
  Serial.println(buf);

//  analogWrite(led2Pin,sensorValue);

  if (ctr > (int)(m*sensorValue)+b) {
    ctr = 0;
    if (ledState == 0) {
      digitalWrite(ledPin, HIGH);
      digitalWrite(led2Pin, HIGH);
      ledState = 1;
    } 
    else {
      digitalWrite(ledPin, LOW);
      digitalWrite(led2Pin, LOW);
      ledState = 0;
    }
  } 
  else {
    ctr += pollfreq;
  }
}




