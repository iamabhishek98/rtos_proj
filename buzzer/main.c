/*----------------------------------------------------------------------------
 * CMSIS-RTOS 'main' function template
 *---------------------------------------------------------------------------*/
 
#include "RTE_Components.h"
#include  CMSIS_device_header
#include "cmsis_os2.h"
#include<MKL25Z4.h>
#define PTB0_Pin 0
#define PTB1_Pin 1
#define TIMER_CLCK_FREQ 375000 
float songspeed = 2; //Change to 2 for a slower version of the song, the bigger the number the slower the song
//*****************************************
#define del 5
#define NOTE_B0  31 * del
#define NOTE_C1  33 * del
#define NOTE_CS1 35 * del
#define NOTE_D1  37 * del
#define NOTE_DS1 39 * del
#define NOTE_E1  41 * del
#define NOTE_F1  44 * del
#define NOTE_FS1 46 * del
#define NOTE_G1  49 * del
#define NOTE_GS1 52 * del
#define NOTE_A1  55 * del
#define NOTE_AS1 58 * del
#define NOTE_B1  62 * del
#define NOTE_C2  65 * del
#define NOTE_CS2 69 * del
#define NOTE_D2  73 * del
#define NOTE_DS2 78 * del
#define NOTE_E2  82 * del
#define NOTE_F2  87 * del
#define NOTE_FS2 93 * del
#define NOTE_G2  98 * del
#define NOTE_GS2 104 * del
#define NOTE_A2  110 * del
#define NOTE_AS2 117 * del
#define NOTE_B2  123 * del
#define NOTE_C3  131 * del
#define NOTE_CS3 139 * del
#define NOTE_D3  147 * del
#define NOTE_DS3 156 * del
#define NOTE_E3  165 * del
#define NOTE_F3  175 * del
#define NOTE_FS3 185 * del
#define NOTE_G3  196 * del
#define NOTE_GS3 208 * del
#define NOTE_A3  220 * del
#define NOTE_AS3 233 * del
#define NOTE_B3  247 * del
#define NOTE_C4  262 * del
#define NOTE_CS4 277 * del
#define NOTE_D4  294 * del
#define NOTE_DS4 311 * del
#define NOTE_E4  330 * del
#define NOTE_F4  349 * del
#define NOTE_FS4 370 * del
#define NOTE_G4  392 * del
#define NOTE_GS4 415 * del
#define NOTE_A4  440 * del
#define NOTE_AS4 466 * del
#define NOTE_B4  494 * del
#define NOTE_C5  523 * del
#define NOTE_CS5 554 * del
#define NOTE_D5  587 * del
#define NOTE_DS5 622 * del
#define NOTE_E5  659 * del
#define NOTE_F5  698 * del
#define NOTE_FS5 740 * del
#define NOTE_G5  784 * del
#define NOTE_GS5 831 * del
#define NOTE_A5  880 * del
#define NOTE_AS5 932 * del
#define NOTE_B5  988 * del
#define NOTE_C6  1047 * del
#define NOTE_CS6 1109 * del
#define NOTE_D6  1175 * del
#define NOTE_DS6 1245 * del
#define NOTE_E6  1319 * del
#define NOTE_F6  1397 * del
#define NOTE_FS6 1480 * del
#define NOTE_G6  1568 * del
#define NOTE_GS6 1661 * del
#define NOTE_A6  1760 * del
#define NOTE_AS6 1865 * del
#define NOTE_B6  1976 * del
#define NOTE_C7  2093 * del
#define NOTE_CS7 2217 * del
#define NOTE_D7  2349 * del
#define NOTE_DS7 2489 * del
#define NOTE_E7  2637 * del
#define NOTE_F7  2794 * del
#define NOTE_FS7 2960 * del
#define NOTE_G7  3136 * del
#define NOTE_GS7 3322 * del
#define NOTE_A7  3520 * del
#define NOTE_AS7 3729 * del
#define NOTE_B7  3951 * del
#define NOTE_C8  4186 * del
#define NOTE_CS8 4435 * del
#define NOTE_D8  4699 * del
#define NOTE_DS8 4978 * del

//*****************************************
int notes[] = {       //Note of the song, 0 is a rest/pulse
   NOTE_E4, NOTE_G4, NOTE_A4, NOTE_A4, 0, 
   NOTE_A4, NOTE_B4, NOTE_C5, NOTE_C5, 0, 
   NOTE_C5, NOTE_D5, NOTE_B4, NOTE_B4, 0,
   NOTE_A4, NOTE_G4, NOTE_A4, 0,
   
   NOTE_E4, NOTE_G4, NOTE_A4, NOTE_A4, 0, 
   NOTE_A4, NOTE_B4, NOTE_C5, NOTE_C5, 0, 
   NOTE_C5, NOTE_D5, NOTE_B4, NOTE_B4, 0,
   NOTE_A4, NOTE_G4, NOTE_A4, 0,
   
   NOTE_E4, NOTE_G4, NOTE_A4, NOTE_A4, 0, 
   NOTE_A4, NOTE_C5, NOTE_D5, NOTE_D5, 0, 
   NOTE_D5, NOTE_E5, NOTE_F5, NOTE_F5, 0,
   NOTE_E5, NOTE_D5, NOTE_E5, NOTE_A4, 0,
   
   NOTE_A4, NOTE_B4, NOTE_C5, NOTE_C5, 0, 
   NOTE_D5, NOTE_E5, NOTE_A4, 0, 
   NOTE_A4, NOTE_C5, NOTE_B4, NOTE_B4, 0,
   NOTE_C5, NOTE_A4, NOTE_B4, 0,

   NOTE_A4, NOTE_A4, 
   //Repeat of first part
   NOTE_A4, NOTE_B4, NOTE_C5, NOTE_C5, 0, 
   NOTE_C5, NOTE_D5, NOTE_B4, NOTE_B4, 0,
   NOTE_A4, NOTE_G4, NOTE_A4, 0,

   NOTE_E4, NOTE_G4, NOTE_A4, NOTE_A4, 0, 
   NOTE_A4, NOTE_B4, NOTE_C5, NOTE_C5, 0, 
   NOTE_C5, NOTE_D5, NOTE_B4, NOTE_B4, 0,
   NOTE_A4, NOTE_G4, NOTE_A4, 0,
   
   NOTE_E4, NOTE_G4, NOTE_A4, NOTE_A4, 0, 
   NOTE_A4, NOTE_C5, NOTE_D5, NOTE_D5, 0, 
   NOTE_D5, NOTE_E5, NOTE_F5, NOTE_F5, 0,
   NOTE_E5, NOTE_D5, NOTE_E5, NOTE_A4, 0,
   
   NOTE_A4, NOTE_B4, NOTE_C5, NOTE_C5, 0, 
   NOTE_D5, NOTE_E5, NOTE_A4, 0, 
   NOTE_A4, NOTE_C5, NOTE_B4, NOTE_B4, 0,
   NOTE_C5, NOTE_A4, NOTE_B4, 0,
   //End of Repeat

   NOTE_E5, 0, 0, NOTE_F5, 0, 0,
   NOTE_E5, NOTE_E5, 0, NOTE_G5, 0, NOTE_E5, NOTE_D5, 0, 0,
   NOTE_D5, 0, 0, NOTE_C5, 0, 0,
   NOTE_B4, NOTE_C5, 0, NOTE_B4, 0, NOTE_A4,

   NOTE_E5, 0, 0, NOTE_F5, 0, 0,
   NOTE_E5, NOTE_E5, 0, NOTE_G5, 0, NOTE_E5, NOTE_D5, 0, 0,
   NOTE_D5, 0, 0, NOTE_C5, 0, 0,
   NOTE_B4, NOTE_C5, 0, NOTE_B4, 0, NOTE_A4
};

int melody[] = {
  NOTE_E7, NOTE_E7, 0, NOTE_E7,
  0, NOTE_C7, NOTE_E7, 0,
  NOTE_G7, 0, 0,  0,
  NOTE_G6, 0, 0, 0,
 
  NOTE_C7, 0, 0, NOTE_G6,
  0, 0, NOTE_E6, 0,
  0, NOTE_A6, 0, NOTE_B6,
  0, NOTE_AS6, NOTE_A6, 0,
 
  NOTE_G6, NOTE_E7, NOTE_G7,
  NOTE_A7, 0, NOTE_F7, NOTE_G7,
  0, NOTE_E7, 0, NOTE_C7,
  NOTE_D7, NOTE_B6, 0, 0,
 
  NOTE_C7, 0, 0, NOTE_G6,
  0, 0, NOTE_E6, 0,
  0, NOTE_A6, 0, NOTE_B6,
  0, NOTE_AS6, NOTE_A6, 0,
 
  NOTE_G6, NOTE_E7, NOTE_G7,
  NOTE_A7, 0, NOTE_F7, NOTE_G7,
  0, NOTE_E7, 0, NOTE_C7,
  NOTE_D7, NOTE_B6, 0, 0
};

int underworld_melody[] = {
  NOTE_C4, NOTE_C5, NOTE_A3, NOTE_A4,
  NOTE_AS3, NOTE_AS4, 0,
  0,
  NOTE_C4, NOTE_C5, NOTE_A3, NOTE_A4,
  NOTE_AS3, NOTE_AS4, 0,
  0,
  NOTE_F3, NOTE_F4, NOTE_D3, NOTE_D4,
  NOTE_DS3, NOTE_DS4, 0,
  0,
  NOTE_F3, NOTE_F4, NOTE_D3, NOTE_D4,
  NOTE_DS3, NOTE_DS4, 0,
  0, NOTE_DS4, NOTE_CS4, NOTE_D4,
  NOTE_CS4, NOTE_DS4,
  NOTE_DS4, NOTE_GS3,
  NOTE_G3, NOTE_CS4,
  NOTE_C4, NOTE_FS4, NOTE_F4, NOTE_E3, NOTE_AS4, NOTE_A4,
  NOTE_GS4, NOTE_DS4, NOTE_B3,
  NOTE_AS3, NOTE_A3, NOTE_GS3,
  0, 0, 0
};
//*****************************************
int duration[] = {         //duration of each note (in ms) Quarter Note is set to 250 ms
  125, 125, 250, 125, 125, 
  125, 125, 250, 125, 125,
  125, 125, 250, 125, 125,
  125, 125, 375, 125, 
  
  125, 125, 250, 125, 125, 
  125, 125, 250, 125, 125,
  125, 125, 250, 125, 125,
  125, 125, 375, 125, 
  
  125, 125, 250, 125, 125, 
  125, 125, 250, 125, 125,
  125, 125, 250, 125, 125,
  125, 125, 125, 250, 125,

  125, 125, 250, 125, 125, 
  250, 125, 250, 125, 
  125, 125, 250, 125, 125,
  125, 125, 375, 375,

  250, 125,
  //Rpeat of First Part
  125, 125, 250, 125, 125,
  125, 125, 250, 125, 125,
  125, 125, 375, 125, 
  
  125, 125, 250, 125, 125, 
  125, 125, 250, 125, 125,
  125, 125, 250, 125, 125,
  125, 125, 375, 125, 
  
  125, 125, 250, 125, 125, 
  125, 125, 250, 125, 125,
  125, 125, 250, 125, 125,
  125, 125, 125, 250, 125,

  125, 125, 250, 125, 125, 
  250, 125, 250, 125, 
  125, 125, 250, 125, 125,
  125, 125, 375, 375,
  //End of Repeat
  
  250, 125, 375, 250, 125, 375,
  125, 125, 125, 125, 125, 125, 125, 125, 375,
  250, 125, 375, 250, 125, 375,
  125, 125, 125, 125, 125, 500,

  250, 125, 375, 250, 125, 375,
  125, 125, 125, 125, 125, 125, 125, 125, 375,
  250, 125, 375, 250, 125, 375,
  125, 125, 125, 125, 125, 500
};

int tempo[] = {
  83, 83, 83, 83,
  83, 83, 83, 83,
  83, 83, 83, 83,
  83, 83, 83, 83,
 
  83, 83, 83, 83,
  83, 83, 83, 83,
  83, 83, 83, 83,
  83, 83, 83, 83,
 
  111, 111, 111,
  83, 83, 83, 83,
  83, 83, 83, 83,
  83, 83, 83, 83,
 
  83, 83, 83, 83,
  83, 83, 83, 83,
  83, 83, 83, 83,
  83, 83, 83, 83,
 
  111, 111, 111,
  83, 83, 83, 83,
  83, 83, 83, 83,
  83, 83, 83, 83,
};

int tempo1[] = {
	83, 83, 83, 83,
  83, 83, 166,
  333,
  83, 83, 83, 83,
  83, 83, 166,
  333,
  83, 83, 83, 83,
  83, 83, 166,
  333,
  83, 83, 83, 83,
  83, 83, 166,
  166, 55, 55, 55,
  166, 166,
  166, 166,
  166, 166,
  55, 55, 55, 55, 55, 55,
  100, 100, 100,
  100, 100, 100,
  333, 333, 333
};

/* initPWM() */
void initPWM(void) {
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

void setFrequency(int frequency){
	int mod = (TIMER_CLCK_FREQ / frequency) - 1;
	TPM1->MOD = mod;
	TPM1_C0V = (mod+1)/2;
}

void buzzer_thread (void *argument) {
	
	// ...
	int count = 0;
  for (;;) {
		/*for (int i=0;i<203;i++){              //203 is the total number of music notes in the song
				int wait = duration[i] * songspeed;
				setFrequency(notes[i]);          //tone(pin,frequency,duration)
				osDelay(wait);
		} */

		for (int i=0;i<30;i++){              //203 is the total number of music notes in the song
				int wait = tempo1[i] * songspeed;
				setFrequency(underworld_melody[i]);          //tone(pin,frequency,duration)
				osDelay(wait);
		}
count++; 		
	}
}

int main(void){
		SystemCoreClockUpdate();
	  initPWM();
		osKernelInitialize();
	
		osThreadNew(buzzer_thread, NULL, NULL);    // Create application main thread
		osKernelStart();
		for(;;) {}
}
