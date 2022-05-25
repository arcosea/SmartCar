
/**********************************
 *        Define Variables        *
 **********************************/
// Note: PWM is to control its speed and IN is to control direction.
        // Right Motors are A and Left are B
#define PIN_Motor_PWMA 5    // Pin 5
#define PIN_Motor_PWMB 6    // Pin 6
#define PIN_Motor_BIN_1 7   // Pin 7
#define PIN_Motor_AIN_1 8   // Pin 8

#define speed_Max 255       // 0-255 Max speed/rotation of motor
#define direction_Forward true 
#define direction_Back false 
int speed_Rest =0;
int vel = 100;

#define dirAmotorA LOW
#define dirAmotorB HIGH


void setup() {
  pinMode(PIN_Motor_PWMA, OUTPUT);
  pinMode(PIN_Motor_PWMB, OUTPUT);
  pinMode(PIN_Motor_AIN_1, OUTPUT);
  pinMode(PIN_Motor_BIN_1, OUTPUT);
  
}

void loop() {
  // put your main code here, to run repeatedly:
    for(int i = 0; i < 3; i++){
    digitalWrite(PIN_Motor_AIN_1, dirAmotorA);     // LOW is forward direction for motor A
    analogWrite(PIN_Motor_PWMA, vel);       // HIGH is forward direction for motor B
    digitalWrite(PIN_Motor_BIN_1, dirAmotorB);
    analogWrite(PIN_Motor_PWMB, vel);
    delay(1000);
    }
    
    analogWrite(PIN_Motor_PWMA, speed_Rest);
    analogWrite(PIN_Motor_PWMB, speed_Rest);
    delay(5000);
 
  
  
  

}
