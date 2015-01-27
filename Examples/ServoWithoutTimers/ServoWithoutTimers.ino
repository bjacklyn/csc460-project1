#include "scheduler.h"
 
int potpin = 0;  // analog pin used to connect the potentiometer
int servopin = 9;
int val;    // variable to read the value from the analog pin 
 
void setup() 
{ 
pinMode(servopin, OUTPUT);
Serial.begin(9600);
}
 
void loop() 
{ 
  val = analogRead(potpin);            // reads the value of the potentiometer (value between 0 and 1023) 
  val = map(val, 0, 1023, 0, 10);     // scale it to use it with the servo (value between 0 and 180) 
  val = val * 180 + 600;
  digitalWrite(servopin, HIGH);
  delayMicroseconds(val);        // waits for the servo to get there 
  digitalWrite(servopin, LOW);
  Serial.println(val);
  delay(20);
} 
