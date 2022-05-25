
#include "IRremote.h"
#include "func.h"
// Tell IDE we want to use code from IR.h 
#include "IRsetup.h"

IRrecv irrecv(RECV_PIN);     
decode_results results; 
/*
#define PIN_Motor_PWMA 5    // Pin 5
#define PIN_Motor_PWMB 6    // Pin 6
#define PIN_Motor_BIN_1 7   // Pin 7
#define PIN_Motor_AIN_1 8   // Pin 8
*/
#define speed_Max 255       // 0-255 Max speed/rotation of motor
#define direction_Forward true 
#define direction_Back false 




void setup() {
  // put your setup code here, to run once:

  Serial.begin(9600);
  Serial.println("IR Receiver Button Decode");
  irrecv.enableIRIn();
  
  pinMode(PIN_Motor_PWMA, OUTPUT);
  pinMode(PIN_Motor_PWMB, OUTPUT);
  pinMode(PIN_Motor_AIN_1, OUTPUT);
  pinMode(PIN_Motor_BIN_1, OUTPUT);
  
  // motorPin();
}

void loop() {
  
  int tmpValue;
  if (irrecv.decode(&results)) // have we received an IR signal?
  {
    for (int i = 0; i < 23; i++)
    {
      // Compare received result to array and looks for a match
      if ((keyValue[i] == results.value) && (i<KEY_NUM))
      {
        Serial.println(keyBuf[i]); // Prints that buttons name in monitor
        tmpValue = results.value;


        
      }
      else if(REPEAT==i)
      {
        results.value = tmpValue;
      }
    }
    irrecv.resume(); // receive the next value
  }

  if(irrecv.decode(&results))
  {
    
      // Compare received result to array and looks for a match
      if (results.value  == aRECV_upper) 
      {
          
             digitalWrite(PIN_Motor_AIN_1, LOW);     // LOW is forward direction for motor A
             analogWrite(PIN_Motor_PWMA, 180);       // HIGH is forward direction for motor B
             digitalWrite(PIN_Motor_BIN_1, HIGH);
             analogWrite(PIN_Motor_PWMB, 180);
    
             delay(2000);
             analogWrite(PIN_Motor_PWMA, 0);
             analogWrite(PIN_Motor_PWMB, 0);
         
      }
      if( results.value == aRECV_ok) {
        analogWrite(PIN_Motor_PWMA, 0);
        analogWrite(PIN_Motor_PWMB, 0);
      }
      if(results.value == aRECV_lower){
              digitalWrite(PIN_Motor_AIN_1, HIGH);     // LOW is forward direction for motor A
             analogWrite(PIN_Motor_PWMA, 180);       // HIGH is forward direction for motor B
             digitalWrite(PIN_Motor_BIN_1, LOW);
             analogWrite(PIN_Motor_PWMB, 180);
    
             delay(2000);
             analogWrite(PIN_Motor_PWMA, 0);
             analogWrite(PIN_Motor_PWMB, 0);
      }
  }
  
  //motorTest();

}
