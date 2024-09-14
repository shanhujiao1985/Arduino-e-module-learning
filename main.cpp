#include <Arduino.h>
#include <Servo.h>
#include <HCSR04.h>

Servo myservo;  // create Servo object to control a servo

UltraSonicDistanceSensor distanceSensor(9, 10);  // Trigger pin, Echo pin

int val;    // variable to read the value from the analog pin
int angle=0;
int shift=1;


void printAngle(int angle, int distance){
    Serial.print(angle);
    Serial.print(",");
    Serial.println(distance);
}

void setup() {
  Serial.begin(115200);
  myservo.attach(11);  // attaches the servo on pin 9 to the Servo object
}

void loop() {

  if(angle>=180){
    shift =-1;
            // waits for the servo to get there
  }

  if(angle<=0)
  {
    shift =1;

  }
  angle+=shift;
  myservo.write(angle);   // sets the servo position according to the scaled value
  int distance = distanceSensor.measureDistanceCm();
  printAngle(angle,distance);
  delay(100);         
}