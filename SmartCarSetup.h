/* All Constant and Global Variables will be found on this document
 * 
 */

#ifndef _SmartCarSetup_H
#define _SmartCarSetup_H


/*******************************************
 *          INFRARED (IR) CONTROL          *
 ******************************************/
#define RECV_PIN 9    // Connected to Pin 9
/*A:4294967295*/
#define aRECV_upper 16736925
#define aRECV_lower 16754775
#define aRECV_left 16720605
#define aRECV_right 16761405
#define aRECV_ok 16712445
#define aRECV_1 16738455
#define aRECV_2 16750695
#define aRECV_3 16756815
#define aRECV_4 16724175
#define aRECV_5 16718055
#define aRECV_6 16743045
#define aRECV_7 16716015
#define aRECV_8 16726215
#define aRECV_9 16734885
#define KEY_NUM 21
#define REPEAT 22
#define aRECV_ast 16728765
#define aRECV_0 16730805
#define aRECV_hashTag 16732845
/*B:*/
#define bRECV_upper 5316027
#define bRECV_lower 2747854299
#define bRECV_left 1386468383
#define bRECV_right 553536955
#define bRECV_ok 3622325019
#define bRECV_1 3238126971
#define bRECV_2 2538093563
#define bRECV_3 4039382595
#define bRECV_4 2534850111
#define bRECV_5 1033561079
#define bRECV_6 1635910171
#define bRECV_7 2351064443
#define bRECV_8 1217346747
#define bRECV_9 71952287
#define bRECV_ast 851901943
#define bRECV_0 465573243
#define bRECV_hashTag 1053031451

unsigned long keyRec[]={bRECV_upper, bRECV_lower, bRECV_left, bRECV_right, bRECV_ok, bRECV_1, 
                          bRECV_2, bRECV_3, bRECV_4, bRECV_5, bRECV_6, bRECV_7, bRECV_8, bRECV_9,
                          bRECV_ast, bRECV_0, bRECV_hashTag};

unsigned long keyValue[]={aRECV_upper, aRECV_lower, aRECV_left, aRECV_right, aRECV_ok, aRECV_1, 
                          aRECV_2, aRECV_3, aRECV_4, aRECV_5, aRECV_6, aRECV_7, aRECV_8, aRECV_9,
                          aRECV_ast, aRECV_0, aRECV_hashTag};

char keyBuf[][10]={"UPPER","LOWER","LEFT","RIGHT","OK","1","2","3","4","5","6","7","8","9","*","0","#"};


/*******************************************
 *            FOR MOTORS                   *
 ******************************************/
#define motorA_PWM 5       // Pin 5  PWM Controls Speed 0 - 255 for RIGHT side Motors. Use analog
#define motorB_PWM 6       // Pin 6  PWM Controls Speed 0 - 255 for LEFT side Motors. Use analog
#define motorB_DIR 7       // Pin 7  IN Controls direction: HIGH is FORWARD. LOW is BACKWARD. Use digital
#define motorA_DIR 8       // Pin 8  IN Controls Direction: LOW is FORWARD. HIGH is BACKWARD. Use digital

#define forward_A LOW      // Motors A go FORWARD on LOW
#define forward_B HIGH     // Motors B go FORWARD on HIGH
#define backward_A HIGH    // Motors A go BACKWARD on HIGH
#define backward_B LOW     // Motors B go BACKWARD on LOW

#define maxSpeed 255       // Max speed/PWM of motors
#define restSpeed 0        // PWM for rest/stationary motors
#define fastSpeed 200      // Speed/PWM of motors when going FORWARD
#define slowSpeed 100      // Speed/PWM of motors when doing obstacleAvoidance
#define backwardSpeed 120  // Speed/PWM of motors when going BACKWARD
#define turnSpeed 75       // Speed/PWM of motors when TURNING
#define currentSpeed;      // Current speed of motors

#define secMove 1500       // Duration (1.5sec) that motors will run when going FORWARD
#define secBack 500        // Duration(1/2 sec) that motors will run going BACKWARD
#define secSlow 300        // Duration(1/2 sec) that motors will run doing obstacleAvoidance
#define secStop 2000       // Duraction (2sec) that motors will STOP for
#define secTurn 400        // Duration (~1/2 sec) that motors will run when TURNING. Turns about 45deg
long preMillis;            // Used to create a delay 


/*******************************************
 *            FOR SERVO MOTOR              *
 ******************************************/
#define pin_Servo 10       // Pin (digital) Servo is connected it
#define centerDeg 90       // Servo rest pos is facing center 90 degreees
#define minDeg 0           // Servo at 0 degrees is facing right
#define maxDeg 180         // Servo can turn up to 180 degrees from minDeg
#define changeDeg 30       // Servo will turn to sides by 90 degrees
#define shiftTime 500      // Servo will transition every half sec
#define secTime 1000       // Servo will transition every sec


/*******************************************
 *            FOR ULTRASONIC             *
 ******************************************/
#define pin_Trig 13        // UltraSonic Trig Pin
#define pin_Echo 12        // Ultrasonic Echo Pin
#define distanceLimit 30   // Obstacle distance limit is 30 cm


/*******************************************
 *            FOR  LINE-TRACKER            *
 ******************************************/
 /* Note: if light is reflected into sensor it will invert the value (High) to LOW. If Black line right under sensor
          if light NOT reflecting back, value will be a HIGH. If NOT reflecting light (LOW) it means sensor is positioned
          right OVER it

   Summary: If optical source sensor read LOW (it is over Black line and reflects NO light)
            If optical source sensor reads HIGH (it is White and DOES reflect light)
            digitalRead uses LOW/HIGH
            analogRead uses PWM Values typically from 0-255 or 0-1023
 */
 
 // For white/Black tracking (VERYYY SIMPLE)
#define pin_L !digitalRead(A2)         // Pin (Analog) L (Left) reads inverted input
#define pin_M !digitalRead(A1)         // Pin (Analog) M (Middle) reads inverted input
#define pin_R !digitalRead(A0)         // Pin (Analog) R (Right) reads inverted input

#define trackSpeed 80                  // Speed of Car when following Line
 
// For General LineTracking
#define LT_L A2                        // Pin (Analog) L (Left optical sensor) is connected to
#define LT_M A1                        // Pin (Analog) M (Middle optical sensor) is connected to
#define LT_R A0                        // Pin (Analog) R (Left optical sensor) is connected to

#define minLineVal 250                 // Minimum Analog value read for a Line (Surface w/ minimal reflection)
#define maxLineVal 850                 // Maximum Analog value read for a Line (Surface w/ minimal reflection)
#define minSpaceVal 950                // Minimum Analog value read by sensor(s) when in free Space (not in Ground)
#define surfVal 150                    // Maximum Analog value read by sensor(s) that are NOT right above Line


/*******************************************
 *            FOR  FOLLOWER            *
 ******************************************/
/* Will use global variables from Ultrasonic and Servo */ 
#define followSpeed 100                // Speed of wheels when moving forward
#define objDistanceLimit 20            // Object to follow must be within 20cm
#define shiftDeg 45                    // Servo will change by 45 deg each scan
#define servoTime 500                  // Servo will change deg every half sec
#define followTime 1000                // Follow for one second




#endif
