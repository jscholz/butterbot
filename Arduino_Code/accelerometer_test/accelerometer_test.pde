#include <stdio.h>

int xPin = 0;
int yPin = 1;
int zPin = 2;
int gyroPin = 4;
int xVal;
int yVal;
int zVal;
int gyroVal;

int pollfreq = 50;
char buf[32];

void setup() {
  // initialize serial port
  Serial.begin(9600);
}

void loop() {
  // read the value from the sensor:
  xVal = analogRead(xPin);    
  delay(pollfreq);
  yVal = analogRead(yPin);    
  delay(pollfreq);
  zVal = analogRead(zPin);    
  delay(pollfreq);
  gyroVal = analogRead(zPin);    
  delay(pollfreq);

  sprintf(buf,"%d %d %d %d",xVal,yVal,zVal,gyroVal);
  Serial.println(buf);
}






