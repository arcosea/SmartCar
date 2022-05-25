// Include library
#include <Servo.h>
#include "SR04.h"

// define servo pin and ultra sound 
#define Pin_Servo 10
#define Pin_Trig 13
#define Pin_Echo 12

// initialize initial position. Recall these are degrees
#define centerPos 90     // in degrees
#define maxDistance 200 // in cm
int minPos = 0;
int maxPos = 180;
int sec = 3000;
int currentPos;
long distance; // in cm

//create object servo
Servo myServo;
SR04 sensor = SR04(Pin_Echo, Pin_Trig);

void setup() {
  // connect servo to uno
  myServo.attach(Pin_Servo);
  Serial.begin(9600);
  delay(sec);
  Serial.println("Distance and Servo Position");
  
}

void loop() {
  // Set pos to initial.
  myServo.write(centerPos);
  for(currentPos = centerPos; currentPos <= 180; currentPos=currentPos+15){
  
    myServo.write(currentPos);
    distance = sensor.Distance();
    Serial.println();
    Serial.print(distance); Serial.print("cm     "); Serial.print(currentPos);
    delay(sec);
  }
  for(currentPos = centerPos; currentPos >= 0; currentPos=currentPos-15)
  {
    myServo.write(currentPos);
    distance = sensor.Distance();
    Serial.println();
    Serial.print(distance); Serial.print("cm     "); Serial.print(currentPos);
    delay(sec);
  }
  
  

}
