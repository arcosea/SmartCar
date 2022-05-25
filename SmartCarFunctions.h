/*
 * This File contains all the functions present in the SmartCar File
 */

 /*****************************************
 *       NECESSARY FILES/LIBRARIES        *
 ******************************************/
  #include "SmartCarSetup.h"          // File with all global variables
  #include "IRremote.h"               // IR Remote Library
  #include <Servo.h>                  // Servo Motor Library     
  #include "SR04.h"                   // Ultra Zip File 

 /*****************************************
 *       NECESSARY OBJECTS                *
 ******************************************/
  // FOR INFRARED (IR) SENSOR
  IRrecv irrecv(RECV_PIN);        // Object based on Uno Pin
  decode_results results;         // Result of IR signal

  // FOR SERVO MOTOR        
  Servo myServo;                  // Servo object and initialization
  
  // FOR ULTRASONIC 
  SR04 ultra = SR04(pin_Echo, pin_Trig);

  // FOR LINETRACKING
  float read_L;                   // Analog value (0-1023) read for L (left) sensor
  float read_M;                   // Analog value (0-1023) read for M (middle) sensor
  float read_R;                   // Analog value (0-1023) read for R (right) sensor

/**************************************************************************************************************************/
  /*****************************************
 *              SETUP FUNCTIONS            *
 ******************************************/
  // SET MOTORS AS OUTPUTS
  void motorSetup() {
    pinMode(motorA_PWM, OUTPUT);          // Set PWM/Speed as output for motor A
    pinMode(motorB_PWM, OUTPUT);          // Set PWM/Speed as output for motor B
    pinMode(motorB_DIR, OUTPUT);          // Set direction as output for motor B
    pinMode(motorA_DIR, OUTPUT);          // Set direction as output for motor A
    analogWrite(motorA_PWM, restSpeed);   // Sets Speed to 0
    analogWrite(motorB_PWM, restSpeed);   // Sets Speed to 0
  }

/**************************************************************************************************************************/ 
  // INITIALIZE IR & SERVO
  void IRservoSetup(){
    irrecv.enableIRIn();              // Enables IR to recieve info
    myServo.attach(pin_Servo);        // Attaches servo to pin 10 
    myServo.write(centerDeg);         // Positions Servo to center
    Serial.begin(9600);
  }

/**************************************************************************************************************************/
  // INITIALIZES LINETRACKING SENSORS AS INPUT
  void opticalSensorSetup() {
    // For White/Black Line Tracking
    pinMode(pin_L, INPUT);
    pinMode(pin_M, INPUT);
    pinMode(pin_R, INPUT);

    // For general Line Tracking
    pinMode(LT_L, INPUT);
    pinMode(LT_M, INPUT);
    pinMode(LT_R, INPUT);
  }

 /******************************************
 *            FUNCTIONS                    *
 ******************************************/
//________________________________________________________________________________________________________________________//
//||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||//
//------------------------------------------------------------------------------------------------------------------------//

 //******************************************//
 //*            MOTOR FUNCTIONS             *//
 //******************************************//
 
 // STOP ALL MOTORS
  void stopCar() {
    analogWrite(motorA_PWM, restSpeed);         // Sets Speed to 0
    analogWrite(motorB_PWM, restSpeed);         // Sets Speed to 0
    myServo.write(centerDeg);                   // Sets Servo to Center
          
 }

/**************************************************************************************************************************/
 // CHECK TO SEE IF IR SIGNAL WAS RECIEVED TO STOP CAR   
 void stopSignal(){
   if (irrecv.decode(&results)){
        if(results.value == aRECV_ok)                         //DOES NOT REALLY WORK
                stopCar();
        }
    irrecv.resume();
 } 

/**************************************************************************************************************************/
 // MOVE CAR FORWARD FOR 3 SECONDS AND STOPS
 void moveForward(int speed) {
    digitalWrite(motorA_DIR, forward_A );     // Set motor A move FORWARD
    analogWrite(motorA_PWM, speed);    // Set motor A move at FORWARD Speed    
    digitalWrite(motorB_DIR, forward_B );     // Set motor B move FORWARD  
    analogWrite(motorB_PWM, speed);    // Set motor B move at FORWARD Speed 
    //delay(secMove);                           // Set motors on for 1.5 sec
    //stopCar();                                // Set stopCar Function 
 }

/**************************************************************************************************************************/
 // MOVE CAR BACKWARD FOR 3 SECONDS AND STOPS
 void moveBackward() {
    digitalWrite(motorA_DIR, backward_A );    // Set motor A move BACKWARD
    analogWrite(motorA_PWM, backwardSpeed);   // Set motor A move at BACKWARD Speed    
    digitalWrite(motorB_DIR, backward_B );    // Set motor B move BACKWARD  
    analogWrite(motorB_PWM, backwardSpeed);   // Set motor B move at BACKWARDSpeed 
    // delay(secMove);                           // Set motors on for 1.5 sec
    // stopCar();                                // Set stopCar Function 
 }
 
/**************************************************************************************************************************/
 // TURNS CAR TO THE RIGHT ABOUT 45 DEG AND STOPS. 
 void turnRight() {
    digitalWrite(motorA_DIR, backward_A);     // Set motor A move BACKWARD
    analogWrite(motorA_PWM, turnSpeed);       // Set motor A move at turn Speed    
    digitalWrite(motorB_DIR, forward_B);      // Set motor B move FORWARD  
    analogWrite(motorB_PWM, turnSpeed);       // Set motor B move at TURN Speed 
    delay(secTurn);                           // Set motors on for .5sec
    stopCar();                                // Set stopCar Function  
 }
 
/**************************************************************************************************************************/
 // SPINS CAR TO THE RIGHT FOR AS LONG AS BUTTON IS HELD 
 void spinRight() {
    digitalWrite(motorA_DIR, backward_A);     // Set motor A move BACKWARD
    analogWrite(motorA_PWM, turnSpeed);       // Set motor A move at turn Speed    
    digitalWrite(motorB_DIR, forward_B);      // Set motor B move FORWARD  
    analogWrite(motorB_PWM, turnSpeed);       // Set motor B move at TURN Speed 
 }
 
/**************************************************************************************************************************/
 // TURNS CAR TO THE LEFT ABOUT 45 DEG AND STOPS
 void turnLeft() {
    digitalWrite(motorA_DIR, forward_A );     // Set motor A move FORWARD
    analogWrite(motorA_PWM, turnSpeed);       // Set motor A move at turn Speed    
    digitalWrite(motorB_DIR, backward_B );    // Set motor B move BACKWARD  
    analogWrite(motorB_PWM, turnSpeed);       // Set motor B move at TURN Speed 
    delay(secTurn);                           // Set motors on for .5 sec
    stopCar();                                // Set stopCar Function 
 }
 
/**************************************************************************************************************************/
 // SPINS CAR TO THE LEFT FOR AS LONG AS BUTTON IS HELD 
 void spinLeft() {
    digitalWrite(motorA_DIR, forward_A );     // Set motor A move FORWARD
    analogWrite(motorA_PWM, turnSpeed);       // Set motor A move at turn Speed    
    digitalWrite(motorB_DIR, backward_B );    // Set motor B move BACKWARD  
    analogWrite(motorB_PWM, turnSpeed);       // Set motor B move at TURN Speed 
 }

//________________________________________________________________________________________________________________________//
//||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||//
//------------------------------------------------------------------------------------------------------------------------//
 //******************************************//
 //*  OBSTACLE (SERVO/ULTRASONIC) FUNCTIONS *//
 //******************************************//
 
 // QUICKLY DETERMINES DISTANCE OF OBJECTS IN FRONT
  void quickDistance(){
    int distanceMeasured = ultra.Distance();        // Distance Measured
    if(distanceMeasured < distanceLimit) {          // If less than limit, move backward
        moveBackward();
        delay(secBack);
        turnRight();
        turnRight();
    }
    // END FUNCTION
  }
 
/**************************************************************************************************************************/
 // DISTANCE OF OBSTACLE AND PUT IN ARRAY
 void obstacleDistance(){
    int currentDeg;                             // Deg servo is currently on
    int arrayIndex = 0;                         // Index in array containing 
    float distanceMeasured;                     // Distance Measured of 
    int distanceArray[7];                       // Array of distances
    int maxDistance = distanceArray[0];         // Contains largest Distance
    int maxIndex;                               // Contains Index of largest Distance

    // Loops servo from rightmost side( 0 deg) to left(180). Calculates distance and stores in array
    for(currentDeg = minDeg; currentDeg <= maxDeg; currentDeg = currentDeg + changeDeg){
        myServo.write(currentDeg);              
        distanceMeasured = ultra.Distance();
        distanceArray[arrayIndex] = distanceMeasured;
        delay(shiftTime);
        arrayIndex++;
    }
 
    // Determine which direction is most open/obstacle-FREE
    for(arrayIndex = 0; arrayIndex < 7; arrayIndex++){
      if(distanceArray[arrayIndex] > maxDistance) {
          maxDistance = distanceArray[arrayIndex];
          maxIndex = arrayIndex;
      }
    }
 
    if(maxDistance < distanceLimit) {
        moveBackward();
        delay(secBack);
        turnRight();
        turnRight();
    }
    
    // Turn car based on which direction is most free/open
    if(maxIndex < 2){
      turnRight();
      turnRight();
      quickDistance();
    }

    if(maxIndex >= 2 && maxIndex <= 3){
      turnRight();
      quickDistance();
    }

    if(maxIndex > 4 && maxIndex < 6){
      turnLeft();
      quickDistance();
    }

    if(maxIndex >= 6){
      turnLeft();
      turnLeft();
      quickDistance();
    }
    // Put servo to center
    myServo.write(centerDeg);  
    // End of function    
  }

/**************************************************************************************************************************/
  // MOVES CAR UNTIL OBSTACLE IS NEARBY AND CHECKS CONDITIONS
  void obstacleAvoidance(){ 
      int totalReps = 100;                            // Number of times loop will repeat
      int checkDistance;                              // Distance of obstacle measured
      for(int rep = 0; rep < totalReps; rep++){
        checkDistance = ultra.Distance();             // Checks distance of obstacle
        if(checkDistance > distanceLimit){            // If distance measured is greater than limit, movecar
            moveForward(slowSpeed);
            checkDistance = ultra.Distance();
            delay(secSlow);
        }                                             // If measured is less, stop car and run obstacleDistance 
        if(checkDistance <= distanceLimit){
            stopCar();
            obstacleDistance();
            delay(shiftTime);
        }  
        stopSignal();
      }
      // END FUNCTION
      
  }

/**************************************************************************************************************************/
 // SERVO SCAN JUST ROTATES THE SERVO - WILL BE USED WITH MOTION SENSOR
  void servoScan(){
    int totalScans = 100;                           // Number of times it will loop 
    int currentDeg;                                 // Degree servo is currently on.
    int minSec= 60;                                 // Number of seconds in a minute
    
    for(int scan = 0; scan< totalScans; scan++)     
    {
      // Loops from 0 deg to 180 deg
      for(currentDeg = minDeg; currentDeg <= maxDeg; currentDeg += (maxDeg/minSec)){
        myServo.write(currentDeg);
        delay(secTime);                             // Delay one Sec  
      } 
      stopSignal();
      
      // Loops from 180 to 0;
      for(currentDeg = maxDeg; currentDeg >= minDeg; currentDeg -= (maxDeg/minSec))
      {
        myServo.write(currentDeg);
        delay(secTime);                           // Delay one Sec 
      }
      stopSignal();
      // End loop
    }
    myServo.write(centerDeg);                       // Returns servo to center once done
    // END FUNCTION
  }

//________________________________________________________________________________________________________________________//
//||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||//
//------------------------------------------------------------------------------------------------------------------------//
  //*******************************************//
  //*           LINE TRACKER FUNCTIONS        *//
  //*******************************************//
  
  // LINE TRACKING FUNCTION THAT IS VERY SIMPLE
  void basicLineTracking(){
    /* NOTE: if statement is valid if it is HIGH */
    int i = 1;
    while(i  > 0) {                       // Loop that runs forever
      stopSignal;
      if(pin_M){                          // If MIDDLE sensor reads LOW then set to HIGH & moveForward
        moveForward(trackSpeed);
      }

      else if(pin_R) {                    // If RIGHT sensor reads LOW then set to HIGH & spinRight while it is reading LOW 
        spinRight();
        while(pin_R);
      }

      else if(pin_L) {                    // If LEFT sensor reads LOW then set to HIGH & spinLeft while it is reading LOW
        spinLeft();
        while(pin_L);
      }
    }
    // END FUNCTION
  }

/**************************************************************************************************************************/
  // FUNCTION THAT READ LMR OPTICAL SENSOR VALUES
  void readLMR_Values(){
    read_L = analogRead(LT_L);
    read_M = analogRead(LT_M);
    read_R = analogRead(LT_R);
    // END FUNCTION
  }

/**************************************************************************************************************************/
  // GENERAL LINE TRACKING
  void lineTracking(){
    int forever = 1;  
    
    while (forever > 0){                // Loop that runs forever
      // Read analog values for each sensor
      readLMR_Values();
    
      // If all the sensors read value of 950 (minSpaceVal) or more, then car is not in ground so stop car
      if(read_L >= minSpaceVal && read_M >= minSpaceVal && read_R >= minSpaceVal){
        stopCar();
        readLMR_Values();
      }

      // If middle line is within range value of black line then move car forward
      else if(read_M >= minLineVal && read_M <= maxLineVal ){
        moveForward(trackSpeed);
        readLMR_Values();
      }

      // If L sensor is above black line, turn car left until M is
      else if(read_L >= minLineVal && read_L <= maxLineVal){ 
          while(read_L >= minLineVal && read_L <= maxLineVal) {
              spinLeft();
              readLMR_Values();
              if(read_L < surfVal) {
                stopCar();
              }
          }    
     }
    
      // If R sensor is above black line, turn car left until M is
      else if(read_R >= minLineVal && read_R <= maxLineVal){ 
          while(read_R >= minLineVal && read_R <= maxLineVal) {
              spinRight();
              readLMR_Values();
              if(read_R < surfVal) {
                stopCar();
              }
          }    
      }
      stopSignal();          // Does not really work
    }
    
    // END FUNCTION
  }

//________________________________________________________________________________________________________________________//
//||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||//
//------------------------------------------------------------------------------------------------------------------------//

  //*******************************************//
  //*               FOLLOWER                  *//
  //*******************************************//
  // DETERMINES WHETHER OBJECT TO FOLLOW IS WITHIN DISTANCE OR NOT AND OPERATES FUNCTION 
  void searchStatus(){
    int currentDeg;                             // Deg servo is currently on
    int arrayIndex = 0;                         // Index in array containing 
    float distanceMeasured;                     // Distance Measured of 
    int distanceArray[5];                       // Array of distances
    int minDistance = distanceArray[0];         // Contains largest Distance
    int minIndex;                               // Contains Index of smallest Distance

    // Loops servo from rightmost side (0 deg) to left (180) by 45. Calculates distance and stores in array
    for(currentDeg = minDeg; currentDeg <= maxDeg; currentDeg = currentDeg + shiftDeg){
        myServo.write(currentDeg);              
        distanceMeasured = ultra.Distance();
        distanceArray[arrayIndex] = distanceMeasured;
        delay(servoTime);
        arrayIndex++;
    }

    // Determine which direction is CLOSEST FREE
    for(arrayIndex = 0; arrayIndex < 5; arrayIndex++){
      if(distanceArray[arrayIndex] <= minDistance) {
          minDistance = distanceArray[arrayIndex];
          minIndex = arrayIndex;
      }
    }

    // Set servo to center
    myServo.write(centerDeg);

    // If object is within 20cm distance & is at 0 deg turn RIGHT TWICE and move forward
    if(minDistance <= objDistanceLimit && minIndex == 0){
        turnRight();
        turnRight();
        moveForward(followSpeed);
    }

    // If object is within 20cm distance & is at 45 deg turn RIGHT ONCE and move forward
    else if(minDistance <= objDistanceLimit && minIndex == 1){
        turnRight();
        moveForward(followSpeed);
    }
    // If object is within 20cm distance & is at 90 deg move forward
    else if(minDistance <= objDistanceLimit && minIndex == 2){
        moveForward(followSpeed);
    }

    // If object is within 20cm distance & is at 135 deg TURN LEFT ONCE and move forward
    else if(minDistance <= objDistanceLimit && minIndex == 3){
        turnLeft();
        moveForward(followSpeed);
    }

    // If object is within 20cm distance & is at 180 deg TURN LEFT TWICE and move forward
    else if(minDistance <= objDistanceLimit && minIndex == 4){
        turnLeft();
        turnLeft();
        moveForward(followSpeed);
    }
    // If object is not within 20cm distance, do NOT MOVE
    else{
      stopCar();
    }   
  }

/**************************************************************************************************************************/
  // FUNCTION THAT DOES OBJECT FOLLOWING USING SEARCHSTATUS METHOD
  void follow(){
    stopCar();                                      // Initial condition of car - Not moving
    int forever = 0;                                // Variable to make infinite loop
    int measureDist = ultra.Distance();             // Initial measured value of object
    int searchCount = 0;                            // Initial count of searchStatus method
              
    while (forever < 1){                            // Infinite loop
       // If object is within distance, Follow it
       if(measureDist <= objDistanceLimit){
            moveForward(followSpeed);
            delay(followTime);
            stopCar();
            measureDist = ultra.Distance();
       }
       // If it is not within distance, search for it w searchStatus.
       // If it is still not there, end loop
       else if (measureDist != objDistanceLimit){
             stopCar();
             searchStatus();
             searchCount++;
             delay(followTime);
             measureDist = ultra.Distance();

             if(searchCount > 2){
                forever = 1;
             } 
             
             if(measureDist <= objDistanceLimit){
                moveForward(followSpeed);
                delay(followTime);
                stopCar();
                searchCount = 0;
             }   
       }
       else{
          stopCar();
          measureDist = ultra.Distance();
       }
       stopSignal();  
    }
    stopCar();    
  }
  
