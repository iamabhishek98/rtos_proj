#ifndef LED_CONTROL_H_
#define LED_CONTROL_H_

#include "common.h"

// LED pin definitions for FRDM board
#define GREEN_LED_1 7
#define GREEN_LED_2 0 
#define GREEN_LED_3 3
#define GREEN_LED_4 4
#define GREEN_LED_5 5
#define GREEN_LED_6 6
#define GREEN_LED_7 10
#define GREEN_LED_8 11
#define RED_LED 6

// global variable that is used in the main program as well
extern volatile int isMotorMoving;

void initLED(void);
void ledGreenOn(void);
void ledGreenOff(void);
void ledGreenRunning(int delay);
void ledRedOn(void);
void ledRedOff(void);

#endif /* LED_CONTROL_H_ */
