/* Name: Erick Arcos
 * Last Updated: 06/ 05/2021
 * Project: Remote Control Car With infrared controller. Obstacle Avoidance,
 *          Follower, and Line Tracking
 */

/*******************************************
 *        LIBRARIES & OBJECTS              *
 ******************************************/
// INCLUDE LIBRARIES
  #include "SmartCarSetup.h"      // File with variable declarations
  #include "SmartCarFunctions.h"  // File with all the functions
 

/*******************************************
 *            SETUP LOOP                   *
 *******************************************/
void setup() {
  motorSetup();                   // Sets motors as output
  IRservoSetup();                 // Sets up IR as recieve & attaches servo & sets to center
  opticalSensorSetup();           // Sets Optical source sensors as inputs
}

/*******************************************
 *            INFINITE LOOP                *
 *******************************************/
void loop() {
  // Check to see if IR signal is Recieved
  if (irrecv.decode(&results)) {
    preMillis = millis();                                // Gives number of ms since arduino powered up
    irrecv.resume();
    switch(results.value){
      
        case aRECV_upper:                                // If Upper clicked, move forward
        case bRECV_upper: moveForward(fastSpeed); break;
        
        case aRECV_lower:                                // If lower clicked, move back
        case bRECV_lower: moveBackward(); break;
        
        case aRECV_left:                                 // If left clicked, spin left
        case bRECV_left: spinLeft(); break;
        
        case aRECV_right:                                // If right clicked, spin right
        case bRECV_right: spinRight(); break;
        
        case aRECV_ok:                                   // If ok clicked, stop car
        case bRECV_ok: stopCar(); break;

        case aRECV_1:                                    // If 1 held, turn car Left
        case bRECV_1: turnLeft(); break;

        case aRECV_2:                                    // If 2 clicked, do servoScan
        case bRECV_2: servoScan(); break;

        case aRECV_3:                                    // If 3 held, turn car Right
        case bRECV_3: turnRight(); break;


        case aRECV_4:                                    // If 4 clicked, operate obstacleAvoidance
        case bRECV_4: obstacleAvoidance(); break;

        case aRECV_5:                                    // If 5 clicked, do obstacleDistance
        case bRECV_5: obstacleDistance(); break;

        case aRECV_6:                                    // If 6 clicked, do follow
        case bRECV_6: follow(); break;

        case aRECV_7:                                    // If 7 clicked, do basicLineTracking
        case bRECV_7: basicLineTracking(); break;

        case aRECV_8:                                    // If 8 clicked, do lineTracking
        case bRECV_8: lineTracking(); break;
       
    }
  }
                                              
  else{                                                  // If nothing is pressed/held for 1/2 sec then stopCar
      if(millis() - preMillis > 500){
      stopCar();
      preMillis = millis();
    }
  }
   // END PROGRAM
   
}
