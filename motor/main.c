/*----------------------------------------------------------------------------
 * CMSIS-RTOS 'main' function template
 *---------------------------------------------------------------------------*/
 
#include "RTE_Components.h"
#include  CMSIS_device_header
#include "cmsis_os2.h"
#include<MKL25Z4.h>

#define MOTOR_CONTROL_1 6 // PTE30 TPM0_CH3
#define MOTOR_CONTROL_2 29 // PTE29 TPM0_CH2
#define MOTOR_CONTROL_3 23 // PTE23 TPM2_CH1
#define MOTOR_CONTROL_4 22 // PTE22 TPM2_CH0

#define PTB0_Pin 0
#define PTB1_Pin 1
#define MOTOR_POWER_1 1 // PTC1
#define MOTOR_POWER_2 2 // PTC2
#define MOTOR_POWER_3 3  // PTB3
#define MOTOR_POWER_4 2 // PTB2

#define TIMER_CLCK_FREQ 375000 
#define MASK(x) (1 << (x)) 
#define osDel 2000

void initGPIO(void) {
	/* enable clock for port B & C */ 
	SIM->SCGC5 |= SIM_SCGC5_PORTD_MASK; 
	
	/* Select GPIO and enable pull-up resistors*/ 
	//PORTC->PCR[MOTOR_POWER_1] |= PORT_PCR_MUX(1) | PORT_PCR_PS_MASK | PORT_PCR_PE_MASK;
	/*PORTC->PCR[MOTOR_POWER_2] |= PORT_PCR_MUX(1) | PORT_PCR_PS_MASK | PORT_PCR_PE_MASK;
	PORTB->PCR[MOTOR_POWER_3] |= PORT_PCR_MUX(1) | PORT_PCR_PS_MASK | PORT_PCR_PE_MASK;
	PORTB->PCR[MOTOR_POWER_4] |= PORT_PCR_MUX(1) | PORT_PCR_PS_MASK | PORT_PCR_PE_MASK;*/
	
	PORTD->PCR[MOTOR_CONTROL_1] |= PORT_PCR_MUX(1) | PORT_PCR_PS_MASK | PORT_PCR_PE_MASK;
	/*PORTE->PCR[MOTOR_CONTROL_2] |= PORT_PCR_MUX(1) | PORT_PCR_PS_MASK | PORT_PCR_PE_MASK;
	*/
	
	/* Set port C & D switch bit to outputs */	
	//PTC->PDDR |= MASK(MOTOR_POWER_1);
	/*PTC->PDDR |= MASK(MOTOR_POWER_2);
	PTB->PDDR |= MASK(MOTOR_POWER_3);	
	PTB->PDDR |= MASK(MOTOR_POWER_4);	*/
	
	PTD->PDDR |= MASK(MOTOR_CONTROL_1);
	/*PTE->PDDR |= MASK(MOTOR_CONTROL_2);
	*/
	// set everything to 0
	//PTC->PDOR |= MASK(MOTOR_POWER_1);		
	//PTC->PDOR |= MASK(MOTOR_POWER_2);			
	PTD->PDOR |= MASK(MOTOR_CONTROL_1);		
	//PTE->PDOR |= MASK(MOTOR_CONTROL_2);		
		
}
void initPWM_temp(void) {
	SIM_SCGC5 |= SIM_SCGC5_PORTB_MASK;
 
 PORTB->PCR[PTB0_Pin] &= ~PORT_PCR_MUX_MASK;
 PORTB->PCR[PTB0_Pin] |= PORT_PCR_MUX(3);
 
 PORTB->PCR[PTB1_Pin] &= ~PORT_PCR_MUX_MASK;
 PORTB->PCR[PTB1_Pin] |= PORT_PCR_MUX(3);
 
 SIM_SCGC6 |= SIM_SCGC6_TPM1_MASK;
 
 SIM->SOPT2 &= ~SIM_SOPT2_TPMSRC_MASK;
 SIM->SOPT2 |= SIM_SOPT2_TPMSRC(1);
	
 TPM1->SC &= ~((TPM_SC_CMOD_MASK) | (TPM_SC_PS_MASK));
 TPM1->SC |= (TPM_SC_CMOD(1) | TPM_SC_PS(7));
 TPM1->SC &= ~(TPM_SC_CPWMS_MASK);
 
 TPM1_C0SC &= ~((TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) | (TPM_CnSC_MSB_MASK) | (TPM_CnSC_MSB_MASK));
 TPM1_C0SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1));
}

void initPWM(void) {
 // enable clock for Port E
 SIM_SCGC5 |= SIM_SCGC5_PORTE_MASK;

 // configure MUX for PWM
 PORTE->PCR[MOTOR_CONTROL_1] &= ~PORT_PCR_MUX_MASK;
 PORTE->PCR[MOTOR_CONTROL_1] |= PORT_PCR_MUX(3);
 PORTE->PCR[MOTOR_CONTROL_2] &= ~PORT_PCR_MUX_MASK;
 PORTE->PCR[MOTOR_CONTROL_2] |= PORT_PCR_MUX(3);
 PORTE->PCR[MOTOR_CONTROL_3] &= ~PORT_PCR_MUX_MASK;
 PORTE->PCR[MOTOR_CONTROL_3] |= PORT_PCR_MUX(3);
 PORTE->PCR[MOTOR_CONTROL_4] &= ~PORT_PCR_MUX_MASK;
 PORTE->PCR[MOTOR_CONTROL_4] |= PORT_PCR_MUX(3);
	
 // enable clock to TPM modules 0 and 2 
 SIM_SCGC6 |= SIM_SCGC6_TPM0_MASK | SIM_SCGC6_TPM2_MASK;
 
 // clear all prior settings for the Timer-PWM Module 
 SIM->SOPT2 &= ~SIM_SOPT2_TPMSRC_MASK;
 //  select the clock source for the TPM counter clock
 SIM->SOPT2 |= SIM_SOPT2_TPMSRC(1); /***NOT SURE***/

 // clears the previous configuration for the clock mode and prescaler for TPM0 and TMP2
 TPM0->SC &= ~((TPM_SC_CMOD_MASK) | (TPM_SC_PS_MASK));
 TPM2->SC &= ~((TPM_SC_CMOD_MASK) | (TPM_SC_PS_MASK));
 // selects the 128 prescaler factor which decreases the clock frequency 
 // and  selects the LPTPM counter clock modes such that the LPTPM counter increments on every LPTPM counter clock
 TPM0->SC |= (TPM_SC_CMOD(1) | TPM_SC_PS(7));
 TPM2->SC |= (TPM_SC_CMOD(1) | TPM_SC_PS(7));
 // selects the CPWM mode for the LPTPM counter in order to operate in the count-up mode
 TPM0->SC &= ~(TPM_SC_CPWMS_MASK);
 TPM2->SC &= ~(TPM_SC_CPWMS_MASK); 
 
 // clears the previous configurations for the Channel mode, Edge and Level selection for TPM0 and TPM2
 // enable high true pulses. This means that a high pulse is present when the counter is counting up
 // The second part of the statement makes the PWM Edge aligned which means the leading edges of signals from all PWM Channels are aligned
 TPM0_C0SC &= ~((TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) | (TPM_CnSC_MSB_MASK) | (TPM_CnSC_MSB_MASK));
 TPM2_C0SC &= ~((TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) | (TPM_CnSC_MSB_MASK) | (TPM_CnSC_MSB_MASK));
 TPM0_C0SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1));
 TPM2_C0SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1));

 /* for TPM1 module
 SIM_SCGC6 |= SIM_SCGC6_TPM1_MASK
 
 // clear all prior settings for the Timer-PWM Module 
 SIM->SOPT2 &= ~SIM_SOPT2_TPMSRC_MASK;
 //  select the clock source for the TPM counter clock
 SIM->SOPT2 |= SIM_SOPT2_TPMSRC(1);

 // clears the previous configuration for the clock mode and prescaler for TPM1
 TPM1->SC &= ~((TPM_SC_CMOD_MASK) | (TPM_SC_PS_MASK));
 // selects the 128 prescaler factor which decreases the clock frequency 
 // and  selects the LPTPM counter clock modes such that the LPTPM counter increments on every LPTPM counter clock
 TPM1->SC |= (TPM_SC_CMOD(1) | TPM_SC_PS(7));
 // selects the CPWM mode for the LPTPM counter in order to operate in the count-up mode
 TPM1->SC &= ~(TPM_SC_CPWMS_MASK);
 
 // clears the previous configurations for the Channel mode, Edge and Level selection
 // enable high true pulses. This means that a high pulse is present when the counter is counting up
 // The second part of the statement makes the PWM Edge aligned which means the leading edges of signals from all PWM Channels are aligned
 TPM1_C0SC &= ~((TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) | (TPM_CnSC_MSB_MASK) | (TPM_CnSC_MSB_MASK));
 TPM1_C0SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1));
 */
}

void setFrequency(int PWM_module, float duty_cycle){
	int frequency = 2000;
	int mod = (TIMER_CLCK_FREQ / frequency) - 1;
	if (PWM_module == 0) {
		TPM0->MOD = mod;
		TPM0_C0V = (mod+1)/2;
	} else if (PWM_module == 1) {
		TPM1->MOD = mod;
		TPM1_C0V = (mod+1)*duty_cycle;
	} else if (PWM_module == 2) {
		TPM2->MOD = mod;
		TPM2_C0V = (mod+1)/2;
	}
}

void motor_thread (void *argument) {
	
	// ...
  for (;;) {
		/*//PTC->PDOR |= MASK(MOTOR_POWER_1);		
		PTD->PDOR &= ~MASK(MOTOR_CONTROL_1);		
		osDelay(osDel);
		//PTC->PDOR &= ~MASK(MOTOR_POWER_1);
		PTD->PDOR |= MASK(MOTOR_CONTROL_1);		
		osDelay(osDel);
		PTC->PDOR |= MASK(MOTOR_POWER_2);		
		osDelay(osDel);
		PTC->PDOR &= ~MASK(MOTOR_POWER_2);
		osDelay(osDel);
		PTB->PDOR |= MASK(MOTOR_POWER_3);		
		osDelay(osDel);
		PTB->PDOR &= ~MASK(MOTOR_POWER_3);
		osDelay(osDel);
		PTB->PDOR |= MASK(MOTOR_POWER_4);		
		osDelay(osDel);
		PTB->PDOR &= ~MASK(MOTOR_POWER_4);
		osDelay(osDel);
		*/
		setFrequency(1, 0.2);
		osDelay(osDel);
		setFrequency(1, 0.9);
		osDelay(osDel);
	}
}

int main(void){
		SystemCoreClockUpdate();
	  initGPIO();  
	  initPWM_temp();
		osKernelInitialize();
	
		osThreadNew(motor_thread, NULL, NULL);    // Create application main thread
		osKernelStart();
		for(;;) {}
}
