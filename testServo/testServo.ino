// Include library
#include <Servo.h>

//create object servo
Servo myServo;

// servo pin
#define Pin_Servo 10

// initialize initial position. Recall these are degrees
#define centerPos 90
int minPos = 0;
int maxPos = 180;
int sec = 1000;
int currentPos;

void setup() {
  // connect servo to uno
  myServo.attach(Pin_Servo);
  
}

void loop() {
  // Set pos to initial.
  myServo.write(centerPos);
  for(currentPos = centerPos; currentPos <= 180; currentPos=currentPos+15)
  {
    myServo.write(currentPos);
    delay(sec);
  }
  for(currentPos = centerPos; currentPos >= 0; currentPos=currentPos-15)
  {
    myServo.write(currentPos);
    delay(sec);
  }
  

}
