#include "motor_control.h"

void initPWM_motor(void) 
{
 // enable clock for PORT E
 SIM_SCGC5 |= SIM_SCGC5_PORTB_MASK | SIM_SCGC5_PORTE_MASK;
 
 // configure MUX functionality for PWM
 PORTB->PCR[MOTOR_PWM_1] &= ~PORT_PCR_MUX_MASK;
 PORTB->PCR[MOTOR_PWM_1] |= PORT_PCR_MUX(3);
 PORTE->PCR[MOTOR_PWM_2] &= ~PORT_PCR_MUX_MASK;
 PORTE->PCR[MOTOR_PWM_2] |= PORT_PCR_MUX(3);
 PORTE->PCR[MOTOR_PWM_3] &= ~PORT_PCR_MUX_MASK;
 PORTE->PCR[MOTOR_PWM_3] |= PORT_PCR_MUX(3);
 PORTE->PCR[MOTOR_PWM_4] &= ~PORT_PCR_MUX_MASK;
 PORTE->PCR[MOTOR_PWM_4] |= PORT_PCR_MUX(3);

 // enable clock to TPM modules 0 and 2 
 SIM_SCGC6 |= SIM_SCGC6_TPM0_MASK | SIM_SCGC6_TPM1_MASK | SIM_SCGC6_TPM2_MASK;
 
 // clear all prior settings for the Timer-PWM Module 
 SIM->SOPT2 &= ~SIM_SOPT2_TPMSRC_MASK;
 //  select the clock source for the TPM counter clock
 SIM->SOPT2 |= SIM_SOPT2_TPMSRC(1); /***NOT SURE***/
 
 // clears the previous configuration for the clock mode and prescaler for TPM0, TPM1 and TPM2
 TPM0->SC &= ~((TPM_SC_CMOD_MASK) | (TPM_SC_PS_MASK));
 TPM1->SC &= ~((TPM_SC_CMOD_MASK) | (TPM_SC_PS_MASK));
 TPM2->SC &= ~((TPM_SC_CMOD_MASK) | (TPM_SC_PS_MASK));
 
 // selects the 128 prescaler factor which decreases the clock frequency 
 // and  selects the LPTPM counter clock modes such that the LPTPM counter increments on every LPTPM counter clock
 TPM0->SC |= (TPM_SC_CMOD(1) | TPM_SC_PS(7));
 TPM1->SC |= (TPM_SC_CMOD(1) | TPM_SC_PS(7));
 TPM2->SC |= (TPM_SC_CMOD(1) | TPM_SC_PS(7));
 
 // selects the CPWM mode for the LPTPM counter in order to operate in the count-up mode
 TPM0->SC &= ~(TPM_SC_CPWMS_MASK);
 TPM1->SC &= ~(TPM_SC_CPWMS_MASK);
 TPM2->SC &= ~(TPM_SC_CPWMS_MASK);
 
 // clears the previous configurations for the Channel mode, Edge and Level selection
 // enable high true pulses. This means that a high pulse is present when the counter is counting up
 // The second part of the statement makes the PWM Edge aligned which means the leading edges of signals from all PWM Channels are aligned
 TPM0_C2SC &= ~((TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) | (TPM_CnSC_MSB_MASK) | (TPM_CnSC_MSB_MASK));
 TPM0_C2SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1));
 TPM2_C0SC &= ~((TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) | (TPM_CnSC_MSB_MASK) | (TPM_CnSC_MSB_MASK));
 TPM2_C0SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1));
 TPM2_C1SC &= ~((TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) | (TPM_CnSC_MSB_MASK) | (TPM_CnSC_MSB_MASK));
 TPM2_C1SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1));
 TPM1_C1SC &= ~((TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) | (TPM_CnSC_MSB_MASK) | (TPM_CnSC_MSB_MASK));
 TPM1_C1SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1));
}

void initMotor(void) 
{
	/* enable clock for port B & C */ 
	SIM->SCGC5 |= SIM_SCGC5_PORTB_MASK | SIM_SCGC5_PORTC_MASK; 
	
	/* Select GPIO and enable pull-up resistors*/ 
	PORTB->PCR[MOTOR_DIR_1] |= PORT_PCR_MUX(1) | PORT_PCR_PS_MASK | PORT_PCR_PE_MASK;
	PORTC->PCR[MOTOR_DIR_2] |= PORT_PCR_MUX(1) | PORT_PCR_PS_MASK | PORT_PCR_PE_MASK;
	PORTB->PCR[MOTOR_DIR_3] |= PORT_PCR_MUX(1) | PORT_PCR_PS_MASK | PORT_PCR_PE_MASK;
	PORTC->PCR[MOTOR_DIR_4] |= PORT_PCR_MUX(1) | PORT_PCR_PS_MASK | PORT_PCR_PE_MASK;
	
	/* Set port C & D switch bit to outputs */	
	PTB->PDDR |= MASK(MOTOR_DIR_1);
	PTC->PDDR |= MASK(MOTOR_DIR_2);
	PTB->PDDR |= MASK(MOTOR_DIR_3);
	PTC->PDDR |= MASK(MOTOR_DIR_4);
}

void setFrequencyMotor(int PWM_module, int PWM_channel, float duty_cycle)
{
	int frequency = 2000;
	int mod = (TIMER_CLCK_FREQ / frequency) - 1;
	
	if (PWM_module == 0 && PWM_channel == 2) {
		TPM0->MOD = mod;
		TPM0_C2V = (mod+1)*duty_cycle;
	} else if (PWM_module == 2 && PWM_channel == 0) {
		TPM2->MOD = mod;
		TPM2_C0V = (mod+1)*duty_cycle;
	} else if (PWM_module == 2 && PWM_channel == 1) {
		TPM2->MOD = mod;
		TPM2_C1V = (mod+1)*duty_cycle;
	} else if (PWM_module == 1 && PWM_channel == 1) {
		TPM1->MOD = mod;
		TPM1_C1V = (mod+1)*duty_cycle;
	}
}

void move_motor(int motor_num, float speed)
{
	
	int dir = 0;
	// DIR = 1 reverse, 0 forward.
	if (speed < 0) {
		dir = 1;
		speed = 1 + speed;
	}
	
	int pwm_module = -1, channel_num = -1;
	if (motor_num == 1) {
		pwm_module = 1;
		channel_num = 1;
		if (dir) PTB->PDOR |= MASK(MOTOR_DIR_1);
		else PTB->PDOR &= ~MASK(MOTOR_DIR_1);
	}
	else if (motor_num == 2) {
		pwm_module = 2;
		channel_num = 1;
		if (dir) PTC->PDOR |= MASK(MOTOR_DIR_2);
		else PTC->PDOR &= ~MASK(MOTOR_DIR_2);
	}
	else if (motor_num == 3) {
		pwm_module = 2;
		channel_num = 0;
		if (dir) PTB->PDOR |= MASK(MOTOR_DIR_3);
		else PTB->PDOR &= ~MASK(MOTOR_DIR_3);
	}
	else if (motor_num == 4) {
		pwm_module = 0;
		channel_num = 2;
		if (dir) PTC->PDOR |= MASK(MOTOR_DIR_4);
		else PTC->PDOR &= ~MASK(MOTOR_DIR_4);
	}

	setFrequencyMotor(pwm_module, channel_num, speed);	
	
}

/*-------------------------------------------------------------------------------------------------------------------*/

// Movement Functions for Motor :

void move_forward(float speed, int duration) 
{
	move_motor(1, speed);
	move_motor(2, speed);
	move_motor(3, speed);
	move_motor(4, speed);
	osDelay(duration);
	move_motor(1, 0);
	move_motor(2, 0);
	move_motor(3, 0);
	move_motor(4, 0);
}

void move_backward(float speed, int duration) 
{
	move_motor(1, -1*speed);
	move_motor(2, -1*speed);
	move_motor(3, -1*speed);
	move_motor(4, -1*speed);
	osDelay(duration);
	move_motor(1, 0);
	move_motor(2, 0);
	move_motor(3, 0);
	move_motor(4, 0);
}

void move_left(float speed, int duration) 
{
	move_motor(1, -1*speed);
	move_motor(2, speed);
	move_motor(3, -1*speed);
	move_motor(4, speed);
	osDelay(duration);
	move_motor(1, 0);
	move_motor(2, 0);
	move_motor(3, 0);
	move_motor(4, 0);
}

void move_right(float speed, int duration) 
{
	move_motor(1, speed);
	move_motor(2, -1*speed);
	move_motor(3, speed);
	move_motor(4, -1*speed);
	osDelay(duration);
	move_motor(1, 0);
	move_motor(2, 0);
	move_motor(3, 0);
	move_motor(4, 0);
}

void curl_right(float speed, int duration, int intensity) 
{
	move_motor(1, speed);
	move_motor(2, speed/intensity);
	move_motor(3, speed);
	move_motor(4, speed/intensity);
	osDelay(duration);
	move_motor(1, 0);
	move_motor(2, 0);
	move_motor(3, 0);
	move_motor(4, 0);
}

void curl_left(float speed, int duration, int intensity) 
{
	move_motor(1, speed/intensity);
	move_motor(2, speed);
	move_motor(3, speed/intensity);
	move_motor(4, speed);
	osDelay(duration);
	move_motor(1, 0);
	move_motor(2, 0);
	move_motor(3, 0);
	move_motor(4, 0);
}