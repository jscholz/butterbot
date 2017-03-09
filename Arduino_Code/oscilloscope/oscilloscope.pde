#include <stdio.h>

#define PIN0 = 0;
#define PIN1 = 1;
#define PIN2 = 2;
#define PIN3 = 3;
#define PIN4 = 4;
#define PIN5 = 5;
int val0;
int val1;
int val2;
int val3;
int val4;
int val5;
char buf[32];

void setup() {
  // initialize serial port
  Serial.begin(9600);
  analogReference(EXTERNAL); // Expects ground for the system on the aref pin 
  /*                           * don't forget to pass through 5K resistor- in case
                               * analogReference settings are wrong we don't want to 
                               * damage the microcontroller
  */
}

void loop() {
  // read the value from the sensor:
//  val0 = analogRead(PIN0);
  val1 = analogRead(PIN1);
  val2 = analogRead(PIN2);
  val3 = analogRead(PIN3);
//  val4 = analogRead(PIN4);
//  val5 = analogRead(PIN5);

  //delay(10);

  sprintf(buf,"%d %d %d",val2, val3, val4); // 2:4
  //sprintf(buf,"%d %d %d %d",val2, val3, val4, val5); // 2:5
  //sprintf(buf,"%d %d %d %d %d %d",val0, val1, val2, val3, val4, val5); // all 6

  Serial.println(buf);
}
