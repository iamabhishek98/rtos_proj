/*----------------------------------------------------------------------------
 * CG22271 Project by Abhishek, Sukrut and Hashir
 *
 * Code for the robot
 *---------------------------------------------------------------------------*/
 
/*-------------------------------------------------------------------------------------------------------------------
 * Importing of header files and componenets
 *-------------------------------------------------------------------------------------------------------------------*/
 

#include "RTE_Components.h"
#include  CMSIS_device_header
#include "cmsis_os2.h"
#include "MKL25Z4.h"

// Masks and delay values
#define MASK(x) (1 << (x)) 
#define del_global 0x40000
#define osDel 2000

//thread ids for led
osThreadId_t redLED_Id, greenLED_Id, motor_Id, brain_Id, audio_Id ;

// Message Queue ids :
osMessageQueueId_t redMsg, greenMsg, motorMsg, audioMsg;

volatile int isMotorMoving = 0;
volatile int isFinished = 0;
volatile int musicCount = 0;

#define MSG_COUNT 5

/*-------------------------------------------------------------------------------------------------------------------*/

// Def. LED's 
#define GREEN_LED_1 7
#define GREEN_LED_2 0 
#define GREEN_LED_3 3
#define GREEN_LED_4 4
#define GREEN_LED_5 5
#define GREEN_LED_6 6
#define GREEN_LED_7 10
#define GREEN_LED_8 11
#define RED_LED 6

/*-------------------------------------------------------------------------------------------------------------------*/

// Def. UART Module
#define FREQ 50
#define BAUD_RATE 9600
#define UART_TX_PORTE22 22
#define UART_RX_PORTE23 23
#define UART_INT_PRIO 128
#define Q_SIZE (32)
 
// volatile int check = 0;




/*-------------------------------------------------------------------------------------------------------------------*/

// Def for motor:

#define TIMER_CLCK_FREQ 375000 

#define MOTOR_PWM_1 1 // PTB1 TPM1_CH1 PTE30 TPM0_CH3 PTE21 TPM1_CH1 PTE20 TPM1_CH0
#define MOTOR_PWM_2 23 // PTE23 TPM2_CH1
#define MOTOR_PWM_3 22 // PTE22 TPM2_CH0
#define MOTOR_PWM_4 29 // PTE29 TPM0_CH2

#define MOTOR_DIR_1 2 // PTB2
#define MOTOR_DIR_2 2 // PTC2
#define MOTOR_DIR_3 3 // PTB3
#define MOTOR_DIR_4 1 // PTC1


/*-------------------------------------------------------------------------------------------------------------------*/

// Def. Song :

#define BUZZER 0
#define TIMER_CLCK_FREQ 375000 
float songspeed = 2; //Change to 2 for a slower version of the song, the bigger the number the slower the song

//*****************************************
#define del_song 5
#define NOTE_B0  31 * del_song
#define NOTE_C1  33 * del_song
#define NOTE_CS1 35 * del_song
#define NOTE_D1  37 * del_song
#define NOTE_DS1 39 * del_song
#define NOTE_E1  41 * del_song
#define NOTE_F1  44 * del_song
#define NOTE_FS1 46 * del_song
#define NOTE_G1  49 * del_song
#define NOTE_GS1 52 * del_song
#define NOTE_A1  55 * del_song
#define NOTE_AS1 58 * del_song
#define NOTE_B1  62 * del_song
#define NOTE_C2  65 * del_song
#define NOTE_CS2 69 * del_song
#define NOTE_D2  73 * del_song
#define NOTE_DS2 78 * del_song
#define NOTE_E2  82 * del_song
#define NOTE_F2  87 * del_song
#define NOTE_FS2 93 * del_song
#define NOTE_G2  98 * del_song
#define NOTE_GS2 104 * del_song
#define NOTE_A2  110 * del_song
#define NOTE_AS2 117 * del_song
#define NOTE_B2  123 * del_song
#define NOTE_C3  131 * del_song
#define NOTE_CS3 139 * del_song
#define NOTE_D3  147 * del_song
#define NOTE_DS3 156 * del_song
#define NOTE_E3  165 * del_song
#define NOTE_F3  175 * del_song
#define NOTE_FS3 185 * del_song
#define NOTE_G3  196 * del_song
#define NOTE_GS3 208 * del_song
#define NOTE_A3  220 * del_song
#define NOTE_AS3 233 * del_song
#define NOTE_B3  247 * del_song
#define NOTE_C4  262 * del_song
#define NOTE_CS4 277 * del_song
#define NOTE_D4  294 * del_song
#define NOTE_DS4 311 * del_song
#define NOTE_E4  330 * del_song
#define NOTE_F4  349 * del_song
#define NOTE_FS4 370 * del_song
#define NOTE_G4  392 * del_song
#define NOTE_GS4 415 * del_song
#define NOTE_A4  440 * del_song
#define NOTE_AS4 466 * del_song
#define NOTE_B4  494 * del_song
#define NOTE_C5  523 * del_song
#define NOTE_CS5 554 * del_song
#define NOTE_D5  587 * del_song
#define NOTE_DS5 622 * del_song
#define NOTE_E5  659 * del_song
#define NOTE_F5  698 * del_song
#define NOTE_FS5 740 * del_song
#define NOTE_G5  784 * del_song
#define NOTE_GS5 831 * del_song
#define NOTE_A5  880 * del_song
#define NOTE_AS5 932 * del_song
#define NOTE_B5  988 * del_song
#define NOTE_C6  1047 * del_song
#define NOTE_CS6 1109 * del_song
#define NOTE_D6  1175 * del_song
#define NOTE_DS6 1245 * del_song
#define NOTE_E6  1319 * del_song
#define NOTE_F6  1397 * del_song
#define NOTE_FS6 1480 * del_song
#define NOTE_G6  1568 * del_song
#define NOTE_GS6 1661 * del_song
#define NOTE_A6  1760 * del_song
#define NOTE_AS6 1865 * del_song
#define NOTE_B6  1976 * del_song
#define NOTE_C7  2093 * del_song
#define NOTE_CS7 2217 * del_song
#define NOTE_D7  2349 * del_song
#define NOTE_DS7 2489 * del_song
#define NOTE_E7  2637 * del_song
#define NOTE_F7  2794 * del_song
#define NOTE_FS7 2960 * del_song
#define NOTE_G7  3136 * del_song
#define NOTE_GS7 3322 * del_song
#define NOTE_A7  3520 * del_song
#define NOTE_AS7 3729 * del_song
#define NOTE_B7  3951 * del_song
#define NOTE_C8  4186 * del_song
#define NOTE_CS8 4435 * del_song
#define NOTE_D8  4699 * del_song
#define NOTE_DS8 4978 * del_song

// Queue data structure to hold the inputs

typedef struct{
unsigned char Data[Q_SIZE];
unsigned int Head; // points to oldest data element
unsigned int Tail; // points to next free space
unsigned int Size; // quantity of elements in queue
} Q_T;
 
volatile Q_T TxQ, RxQ;

//Data Packet for Message Queue
typedef struct {
	uint8_t cmd;
} myDataPkt;

//*****************************************
int running_melody[] = {
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

int end_melody[] = {       //Note of the song, 0 is a rest/pulse
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


//*****************************************
int running_duration[] = {
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

int end_duration[] = {         //duration of each note (in ms) Quarter Note is set to 250 ms
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


/*-------------------------------------------------------------------------------------------------------------------*/

/*-------------------------------------------------------------------------------------------------------------------
 * Initialization and helper functions :
 *-------------------------------------------------------------------------------------------------------------------*/
 
// PWM for motor and buzzer :

// This is the PWM Signal needed for the buzzer

void initPWM_buzzer(void) {
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

// This is the PWM for the motor :

void initPWM_motor(void) {
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

void initMotor(void) {
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

	// active low 
	
	// set everything to 0
	//PTC->PDOR |= MASK(MOTOR_DIR_1);
  /*	
	PTC->PDOR &= ~MASK(MOTOR_DIR_1);

	PTC->PDOR |= MASK(MOTOR_DIR_2);		
	
	//PTB->PDOR |= MASK(MOTOR_DIR_3);		
	PTB->PDOR &= ~MASK(MOTOR_DIR_3);
	
	PTB->PDOR |= MASK(MOTOR_DIR_4);		
	*/	
}

void initLED(void) {
	/* enable clock for port C & D */ 
	SIM->SCGC5 |=  SIM_SCGC5_PORTC_MASK | SIM_SCGC5_PORTD_MASK; 
	
	/* Select GPIO and enable pull-up resistors*/ 
	PORTC->PCR[GREEN_LED_1] |= PORT_PCR_MUX(1) | PORT_PCR_PS_MASK | PORT_PCR_PE_MASK;
	PORTC->PCR[GREEN_LED_2] |= PORT_PCR_MUX(1) | PORT_PCR_PS_MASK | PORT_PCR_PE_MASK;
	PORTC->PCR[GREEN_LED_3] |= PORT_PCR_MUX(1) | PORT_PCR_PS_MASK | PORT_PCR_PE_MASK;
	PORTC->PCR[GREEN_LED_4] |= PORT_PCR_MUX(1) | PORT_PCR_PS_MASK | PORT_PCR_PE_MASK;
	PORTC->PCR[GREEN_LED_5] |= PORT_PCR_MUX(1) | PORT_PCR_PS_MASK | PORT_PCR_PE_MASK;
	PORTC->PCR[GREEN_LED_6] |= PORT_PCR_MUX(1) | PORT_PCR_PS_MASK | PORT_PCR_PE_MASK;
	PORTC->PCR[GREEN_LED_7] |= PORT_PCR_MUX(1) | PORT_PCR_PS_MASK | PORT_PCR_PE_MASK;
	PORTC->PCR[GREEN_LED_8] |= PORT_PCR_MUX(1) | PORT_PCR_PS_MASK | PORT_PCR_PE_MASK;
	
	PORTD->PCR[RED_LED] |= PORT_PCR_MUX(1) | PORT_PCR_PS_MASK | PORT_PCR_PE_MASK;
	
	
	/* Set port C & D switch bit to outputs */	
	PTC->PDDR |= MASK(GREEN_LED_1);
	PTC->PDDR |= MASK(GREEN_LED_2);
	PTC->PDDR |= MASK(GREEN_LED_3);	
	PTC->PDDR |= MASK(GREEN_LED_4);	
	PTC->PDDR |= MASK(GREEN_LED_5);	
	PTC->PDDR |= MASK(GREEN_LED_6);	
	PTC->PDDR |= MASK(GREEN_LED_7);	
	PTC->PDDR |= MASK(GREEN_LED_8);	
	
	PTD->PDDR |= MASK(RED_LED);
}

// Initialize UART Module

void initUART2(uint32_t baud_rate) {
	uint32_t divisor, bus_clock;

	SIM->SCGC4 |= SIM_SCGC4_UART2_MASK;
	SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK;

	PORTD->PCR[2] &= ~PORT_PCR_MUX_MASK;
	PORTD->PCR[2] |= PORT_PCR_MUX(3);

	UART2->C2 &= ~((UART_C2_TE_MASK) | (UART_C2_RE_MASK));

	bus_clock = (DEFAULT_SYSTEM_CLOCK)/2;
	divisor = bus_clock / (baud_rate*16);
	UART2->BDH = UART_BDH_SBR(divisor >> 8);
	UART2->BDL = UART_BDL_SBR(divisor);

	NVIC_SetPriority(UART2_IRQn, 128);
	NVIC_ClearPendingIRQ(UART2_IRQn);
	NVIC_EnableIRQ(UART2_IRQn);

	UART2->C1 = 0;
	UART2->S2 = 0;
	UART2->C3 = 0;
	
	UART2->C2 |= ((UART_C2_TE_MASK) | (UART_C2_RE_MASK) | UART_C2_RIE_MASK);
	Q_Init(&RxQ);
}

// Queue for Data from Bluetooth :

void Q_Init(Q_T * q) {
	unsigned int i;
	for (i=0; i<Q_SIZE; i++)
		q->Data[i] = 0; // to simplify our lives when debugging
	q->Head = 0;
	q->Tail = 0;
	q->Size = 0;
}
 
int Q_Empty(Q_T * q) {
	return q->Size == 0;
}

int Q_Full(Q_T * q) {
	return q->Size == Q_SIZE;
}
 
int Q_Enqueue(Q_T * q, unsigned char d) {
	// What if queue is full?
	if (!Q_Full(q)) {
			q->Data[q->Tail++] = d;
			q->Tail %= Q_SIZE;
			q->Size++;
			return 1; // success
	} else
			return 0; // failure
}

unsigned char Q_Dequeue(Q_T * q) {
    // Must check to see if queue is empty before dequeueing
    unsigned char t=0;
    if (!Q_Empty(q)) {
        t = q->Data[q->Head];
        q->Data[q->Head++] = 0; // to simplify debugging
        q->Head %= Q_SIZE;
        q->Size--;
    }
    return t;
}

/*-------------------------------------------------------------------------------------------------------------------
 * UART Handler and Threads :
 *-------------------------------------------------------------------------------------------------------------------*/

//UART Handler :

void UART2_IRQHandler(void) {
    if (UART2->S1 & UART_S1_RDRF_MASK) {
        // received a character
        if (!Q_Full(&RxQ)) {
        Q_Enqueue(&RxQ, UART2->D);
        } else {
        // error -queue full.
        }
    }
}

/*-------------------------------------------------------------------------------------------------------------------*/


/*-------------------------------------------------------------------------------------------------------------------*/

// Set Frequency :

// Set freq for the buzzer.
void setFrequencyBuzzer(int frequency){
	int mod = (TIMER_CLCK_FREQ / frequency) - 1;
	TPM1->MOD = mod;
	TPM1_C0V = (mod+1)/2;
}

// Set freq for the motor
void setFrequencyMotor(int PWM_module, int PWM_channel, float duty_cycle){
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
/*-------------------------------------------------------------------------------------------------------------------*/

// Move Motor API function :

void move_motor(int motor_num, float speed) {
	
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

void move_forward (float speed, int duration) {
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

void move_backward (float speed, int duration) {
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

void move_left (float speed, int duration) {
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

void move_right (float speed, int duration) {
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

void curl_right (float speed, int duration, int intensity) {
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

void curl_left (float speed, int duration, int intensity) {
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

/*-------------------------------------------------------------------------------------------------------------------*/

// GPIO Motor



/*-------------------------------------------------------------------------------------------------------------------*/

// GPIO LED :



/*-------------------------------------------------------------------------------------------------------------------*/




/*-------------------------------------------------------------------------------------------------------------------*/



// Threads :

void tAudio (void *argument) {
	// ...
	myDataPkt myRxData;
	int isBluetoothConnect = 0;
  for (;;) {
		osMessageQueueGet(audioMsg, &myRxData, NULL, osWaitForever);
		// before starting the flag will be 0 hence there will be no sound.
		// once the connection is established we can play a connection tune and set the flag = 1.
		if(myRxData.cmd == 0x00) {
			
			// play unique connection tune.
			// These for the time beaing are random exerpts from the main tune :D
			for (int i=0;i<5;i++){              
				int wait = end_duration[i] * songspeed;
				setFrequencyBuzzer(end_melody[i]);
				osDelay(wait);
			}
			isBluetoothConnect = 1;
		} 
		else if (myRxData.cmd == 0x07) {     
			// Once challenge is over you can turn off the main tune.
			isBluetoothConnect = 2;
			isFinished = 1;
		}
		if (isBluetoothConnect == 1 && !isFinished) {
			// Main tune if the challenge has started
			int max_count = musicCount+4;
			while (musicCount < max_count) {
				if (musicCount == 34) {
					max_count = 0; musicCount = 0;
				}
				int wait = running_duration[musicCount] * songspeed;
				setFrequencyBuzzer(running_melody[musicCount]);          //tone(pin,frequency,duration)
				musicCount++;
				osDelay(wait);
			}
			
		}	else if (isBluetoothConnect == 2) {
			//play victory tune
			while (isFinished) {
				for (int i=0;i<sizeof(end_melody);i++){              
					int wait = end_duration[i] * songspeed;
					setFrequencyBuzzer(end_melody[i]);
					osDelay(wait);
				}	
				setFrequencyBuzzer(0);
			}
		}
		
	}
}

/*-------------------------------------------------------------------------------------------------------------------*/
void led_green_on() {
	PTC->PDOR |= MASK(GREEN_LED_1);
	PTC->PDOR |= MASK(GREEN_LED_2);
	PTC->PDOR |= MASK(GREEN_LED_3);
	PTC->PDOR |= MASK(GREEN_LED_4);
	PTC->PDOR |= MASK(GREEN_LED_5);
	PTC->PDOR |= MASK(GREEN_LED_6);
	PTC->PDOR |= MASK(GREEN_LED_7);
	PTC->PDOR |= MASK(GREEN_LED_8);
}

void led_green_off() {
	PTC->PDOR &= ~MASK(GREEN_LED_1);
	PTC->PDOR &= ~MASK(GREEN_LED_2);
	PTC->PDOR &= ~MASK(GREEN_LED_3);
	PTC->PDOR &= ~MASK(GREEN_LED_4);
	PTC->PDOR &= ~MASK(GREEN_LED_5);
	PTC->PDOR &= ~MASK(GREEN_LED_6);
	PTC->PDOR &= ~MASK(GREEN_LED_7);
	PTC->PDOR &= ~MASK(GREEN_LED_8);
}

void led_green_running(int delay) {
	PTC->PDOR |= MASK(GREEN_LED_1);		
	osDelay(delay);
	PTC->PDOR &= ~MASK(GREEN_LED_1);
	
	if(!isMotorMoving) return;
	
	PTC->PDOR |= MASK(GREEN_LED_2);		
	osDelay(delay);
	PTC->PDOR &= ~MASK(GREEN_LED_2);
	
	if(!isMotorMoving) return;
	
	PTC->PDOR |= MASK(GREEN_LED_3);		
	osDelay(delay);
	PTC->PDOR &= ~MASK(GREEN_LED_3);
	
	if(!isMotorMoving) return;
	
	PTC->PDOR |= MASK(GREEN_LED_4);		
	osDelay(delay);
	PTC->PDOR &= ~MASK(GREEN_LED_4);
	
	if(!isMotorMoving) return;
	
	PTC->PDOR |= MASK(GREEN_LED_5);		
	osDelay(delay);
	PTC->PDOR &= ~MASK(GREEN_LED_5);
	
	if(!isMotorMoving) return;
	
	PTC->PDOR |= MASK(GREEN_LED_6);		
	osDelay(delay);
	PTC->PDOR &= ~MASK(GREEN_LED_6);
	
	if(!isMotorMoving) return;
	
	PTC->PDOR |= MASK(GREEN_LED_7);		
	osDelay(delay);
	PTC->PDOR &= ~MASK(GREEN_LED_7);
	
	if(!isMotorMoving) return;
	
	PTC->PDOR |= MASK(GREEN_LED_8);		
	osDelay(delay);
	PTC->PDOR &= ~MASK(GREEN_LED_8);
}

void led_green_thread (void *argument) {
	myDataPkt myRxData;
	// ...
  for (;;) {
		osMessageQueueGet(greenMsg, &myRxData, NULL, osWaitForever);
		if (myRxData.cmd == 0x09) {
			led_green_off();
		}
		else if(myRxData.cmd ==0x00){
			led_green_on();
			osDelay(250);				
			led_green_off();
			osDelay(250);	
			led_green_on();
			osDelay(250);				
			led_green_off();
			osDelay(250);
		} else if (myRxData.cmd == 0x07 || myRxData.cmd == 0x08) {
			// light up all together when you are waiting			
			led_green_on();	
			
		} else {
			// light up in sequence while it is moving
			
				led_green_off();
				while (isMotorMoving) {
					led_green_running(80);
				}	
				// osDelay(osDel);
		}
	}
}

/*-------------------------------------------------------------------------------------------------------------------*/


void led_red_on() {
	PTD->PDOR |= MASK(RED_LED);		
}

void led_red_off() {
	PTD->PDOR &= ~MASK(RED_LED);	
}

void led_red_thread (void *argument) {
	myDataPkt myRxData;
  for (;;) {
		osMessageQueueGet(redMsg, &myRxData, NULL, osWaitForever);
		if (myRxData.cmd == 0x00 || myRxData.cmd == 0x09) {
			led_red_off();
		}
		else if(myRxData.cmd == 0x07 || myRxData.cmd == 0x08) {
			while (!isMotorMoving) {
				led_red_on();	
				osDelay(250);
				led_red_off();
				osDelay(250);
			}	
		} else {
				while (isMotorMoving) {
					led_red_on();	
					osDelay(500);
					led_red_off();
					osDelay(500);
				}	
		}
	}
}

/*-------------------------------------------------------------------------------------------------------------------*/

void tBrain (void *argument) {
	// We can have messages of : 
	
	// unmoving
	// 0x00 - Disconnected to connected
	// 0x07 - Finished maze
	// 0x08 - Internally defined as wait
	
	// moving
	// 0x01 - Forward
	// 0x02 - Left
	// 0x03 - Right
	// 0x04 - Backwards
	// 0x05 - Curve Left (CL)
	// 0x06 - Curve Right (CR)
	
	myDataPkt myData;
	myData.cmd = 0x09; // 9 is the start state
  for (;;) {
		// uint8_t data;
		// If there is a command which comes in then you have to execute it 
		
		if (!isFinished) {
			if(!Q_Empty(&RxQ)) {
				myData.cmd = Q_Dequeue(&RxQ);
			} else if (myData.cmd != 0x09) {
					// Once the command is executed or if no command u go back into the waiting state
					myData.cmd = 0x08;
			}
			osMessageQueuePut(audioMsg, &myData, NULL, 0);
			osMessageQueuePut(motorMsg, &myData, NULL, 0);
		}
		
		osDelay(1000); // it is necessary
	}
}

/*-------------------------------------------------------------------------------------------------------------------*/

void tMotorControl (void *argument) {
	myDataPkt myRxData;
  for (;;) {
		int default_movement_duration = 500;
		osMessageQueueGet(motorMsg, &myRxData, NULL, osWaitForever);
			
		osMessageQueuePut(redMsg, &myRxData, NULL, 0);
		osMessageQueuePut(greenMsg, &myRxData, NULL, 0);
		
		if (!(myRxData.cmd == 0x00 || 
			myRxData.cmd == 0x07 || 
			myRxData.cmd == 0x08)) {
			isMotorMoving = 1;
		}
		
		if (myRxData.cmd == 0x01){//Forward
			move_forward(0.9, default_movement_duration);
		} else if (myRxData.cmd == 0x02){//Left
			move_left(0.7, default_movement_duration);
		} else if (myRxData.cmd == 0x03){//Right
			move_right(0.7, default_movement_duration);
		} else if (myRxData.cmd == 0x04){//Back
			move_backward(0.9, default_movement_duration);
		} else if (myRxData.cmd == 0x05){//CL
			move_left(0.7, 200);
			//curl_left(0.9, default_movement_duration, 3);
		} else if (myRxData.cmd == 0x06){//CR
			move_right(0.7, 200);
			// curl_right(0.9, default_movement_duration, 3);
		} else {
			move_forward(0.0, 10);
			// No moving when in wait state, connection state or victory tune state
			// myRxData.cmd == 0x00 || myRxData.cmd == 0x08 || myRxData.cmd == 0x07
		}
		
		osDelay(30); // allow motor to switch off
		isMotorMoving = 0;
		
	}
}

/*-------------------------------------------------------------------------------------------------------------------*/


/*-------------------------------------------------------------------------------------------------------------------
 * Main Function :
 *-------------------------------------------------------------------------------------------------------------------*/


int main(void) {
	// Setup
	SystemCoreClockUpdate();
	initPWM_buzzer();
	initPWM_motor();
	initLED();
	initMotor();
	initUART2(BAUD_RATE);
	
	osKernelInitialize();
	
	//Creation of the threads
	brain_Id = osThreadNew(tBrain, NULL, NULL);
	audio_Id = osThreadNew(tAudio, NULL, NULL);
	redLED_Id = osThreadNew(led_red_thread, NULL, NULL);    // Create application main thread
	greenLED_Id = osThreadNew(led_green_thread, NULL, NULL);
	motor_Id = osThreadNew(tMotorControl, NULL, NULL);
	
	redMsg = osMessageQueueNew(MSG_COUNT, sizeof(myDataPkt), NULL);
	greenMsg = osMessageQueueNew(MSG_COUNT, sizeof(myDataPkt), NULL);
	audioMsg = osMessageQueueNew(MSG_COUNT, sizeof(myDataPkt), NULL);
	motorMsg = osMessageQueueNew(MSG_COUNT, sizeof(myDataPkt), NULL);
	
  osKernelStart();
	for(;;) {}
}

/*-------------------------------------------------------------------------------------------------------------------*/

// Unused functions :

/*
static void delay(volatile uint32_t nof) {
    while(nof!=0) {
        __asm("NOP");
        nof--;
    }
}  
*/