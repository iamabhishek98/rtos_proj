#ifndef MOTOR_CONTROL_H_
#define MOTOR_CONTROL_H_

/*
	Module that helps with motor control for 
	simple vehicle movements. 
	
	Open Loop control (no feedback).
	
	Motor control works by having difference of voltage
	across PWM and GPIO pin. (PWM +V, GPIO 0) results in 
	forward motion. (PWM +(1-V), GPIO 1) results in backward
	motion of the same speed since (1-V) - 1 = -V.
	
	Lower speeds may stall due to low battery capacity.
*/

#include "common.h"

// PWM pins on FRDM board

#define MOTOR_PWM_1 1  // PTB1  TPM1_CH1 
#define MOTOR_PWM_2 23 // PTE23 TPM2_CH1
#define MOTOR_PWM_3 22 // PTE22 TPM2_CH0
#define MOTOR_PWM_4 29 // PTE29 TPM0_CH2

// GPIO Pins on FRDM board

#define MOTOR_DIR_1 2 // PTB2
#define MOTOR_DIR_2 2 // PTC2
#define MOTOR_DIR_3 3 // PTB3
#define MOTOR_DIR_4 1 // PTC1


void initPWM_motor(void);
void initGPIOMotor(void);
void setFrequencyMotor(int PWM_module, int PWM_channel, float duty_cycle);
void moveMotor(int motor_num, float speed);
void moveForward(float speed, int duration);
void moveBackward(float speed, int duration);
void moveLeft (float speed, int duration);
void moveRight (float speed, int duration);
void curlRight(float speed, int duration, int intensity);
void curlLeft (float speed, int duration, int intensity);

#endif /* MOTOR_CONTROL_H_ */
