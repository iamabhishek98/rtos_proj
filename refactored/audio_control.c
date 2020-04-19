#include "audio_control.h"

// Initialize PWM module of buzzer
void initPWMBuzzer(void)
{
 SIM_SCGC5 |= SIM_SCGC5_PORTB_MASK;
 
 PORTB->PCR[BUZZER] &= ~PORT_PCR_MUX_MASK;
 PORTB->PCR[BUZZER] |= PORT_PCR_MUX(3);
 
 SIM_SCGC6 |= SIM_SCGC6_TPM1_MASK;
 
 SIM->SOPT2 &= ~SIM_SOPT2_TPMSRC_MASK;
 SIM->SOPT2 |= SIM_SOPT2_TPMSRC(1);
	
 TPM1->SC &= ~((TPM_SC_CMOD_MASK) | (TPM_SC_PS_MASK));
 TPM1->SC |= (TPM_SC_CMOD(1) | TPM_SC_PS(7));
 TPM1->SC &= ~(TPM_SC_CPWMS_MASK);
 
 TPM1_C0SC &= ~((TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) | (TPM_CnSC_MSB_MASK) | (TPM_CnSC_MSB_MASK));
 TPM1_C0SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1));
}

// Set freq for the buzzer
void setFrequencyBuzzer(int frequency)
{
	int mod = (TIMER_CLCK_FREQ / frequency) - 1;
	TPM1->MOD = mod;
	TPM1_C0V = (mod+1)/2;
}
