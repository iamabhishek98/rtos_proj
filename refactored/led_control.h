#ifndef LED_CONTROL_H_
#define LED_CONTROL_H_

#include "common.h"

#define GREEN_LED_1 7
#define GREEN_LED_2 0 
#define GREEN_LED_3 3
#define GREEN_LED_4 4
#define GREEN_LED_5 5
#define GREEN_LED_6 6
#define GREEN_LED_7 10
#define GREEN_LED_8 11
#define RED_LED 6

extern volatile int isMotorMoving;

void initLED(void);
void led_green_on(void);
void led_green_off(void);
void led_green_running(int delay);
void led_red_on(void);
void led_red_off(void);

#endif /* LED_CONTROL_H_ */
