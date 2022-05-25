#ifndef _IRsetup_H
#define _IRsetup_H

#define RECV_PIN 9

/*A:4294967295*/
#define aRECV_upper 16736925
#define aRECV_lower 16754775
#define aRECV_Left 16720605
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
#define bRECV_Left 1386468383
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


unsigned long keyRec[]={bRECV_upper, bRECV_lower, bRECV_Left, bRECV_right, bRECV_ok, bRECV_1, 
                          bRECV_2, bRECV_3, bRECV_4, bRECV_5, bRECV_6, bRECV_7, bRECV_8, bRECV_9,
                          bRECV_ast, bRECV_0, bRECV_hashTag};



  // array of all definitions
unsigned long keyValue[]={aRECV_upper, aRECV_lower, aRECV_Left, aRECV_right, aRECV_ok, aRECV_1, 
                          aRECV_2, aRECV_3, aRECV_4, aRECV_5, aRECV_6, aRECV_7, aRECV_8, aRECV_9,
                          aRECV_ast, aRECV_0, aRECV_hashTag};

char keyBuf[][10]={"UPPER","LOWER","LEFT","RIGHT","OK","1","2","3","4","5","6","7","8","9","*","0","#"};

// Motor stuff
#define PIN_Motor_PWMA 5    // Pin 5
#define PIN_Motor_PWMB 6    // Pin 6
#define PIN_Motor_BIN_1 7   // Pin 7
#define PIN_Motor_AIN_1 8   // Pin 8

#endif
