#ifndef MOTOR_CONTROL_H_
#define MOTOR_CONTROL_H_

#include "common.h"

#define MOTOR_PWM_1 1 // PTB1 TPM1_CH1 PTE30 TPM0_CH3 PTE21 TPM1_CH1 PTE20 TPM1_CH0
#define MOTOR_PWM_2 23 // PTE23 TPM2_CH1
#define MOTOR_PWM_3 22 // PTE22 TPM2_CH0
#define MOTOR_PWM_4 29 // PTE29 TPM0_CH2

#define MOTOR_DIR_1 2 // PTB2
#define MOTOR_DIR_2 2 // PTC2
#define MOTOR_DIR_3 3 // PTB3
#define MOTOR_DIR_4 1 // PTC1


void initPWM_motor(void);
void initMotor(void);
void setFrequencyMotor(int PWM_module, int PWM_channel, float duty_cycle);
void move_motor(int motor_num, float speed);
void move_forward(float speed, int duration);
void move_backward(float speed, int duration);
void move_left (float speed, int duration);
void move_right (float speed, int duration);
void curl_right(float speed, int duration, int intensity);
void curl_left (float speed, int duration, int intensity);

#endif /* MOTOR_CONTROL_H_ */
