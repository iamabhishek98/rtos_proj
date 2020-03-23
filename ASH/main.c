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

#define MSG_COUNT 4

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

// Queue data structure to hold the inputs

typedef struct{
unsigned char Data[Q_SIZE];
unsigned int Head; // points to oldest data element
unsigned int Tail; // points to next free space
unsigned int Size; // quantity of elements in queue
} Q_T;
 
Q_T TxQ, RxQ;

//Data Packet for Message Queue
typedef struct {
	uint8_t cmd;
} myDataPkt;



/*-------------------------------------------------------------------------------------------------------------------*/

// Def for motor:

#define TIMER_CLCK_FREQ 375000 

#define MOTOR_POWER_1 1 // PTC1
#define MOTOR_POWER_2 2 // PTC2
#define MOTOR_POWER_3 3  // PTB3
#define MOTOR_POWER_4 2 // PTB2

#define MOTOR_CONTROL_1 6 // PTE30 TPM0_CH3
#define MOTOR_CONTROL_2 29 // PTE29 TPM0_CH2
#define MOTOR_CONTROL_3 23 // PTE23 TPM2_CH1
#define MOTOR_CONTROL_4 22 // PTE22 TPM2_CH0


/*-------------------------------------------------------------------------------------------------------------------*/

// Def. Song :

#define PTB0_Pin 0
#define PTB1_Pin 1
#define TIMER_CLCK_FREQ 375000 
float songspeed = 2; //Change to 2 for a slower version of the song, the bigger the number the slower the song

//*****************************************
#define del_song 10
#define NOTE_C4  262*del_song   //Defining note frequency
#define NOTE_D4  294*del_song
#define NOTE_E4  330*del_song
#define NOTE_F4  349*del_song
#define NOTE_G4  392*del_song
#define NOTE_A4  440*del_song
#define NOTE_B4  494*del_song
#define NOTE_C5  523*del_song
#define NOTE_D5  587*del_song
#define NOTE_E5  659*del_song
#define NOTE_F5  698*del_song
#define NOTE_G5  784*del_song
#define NOTE_A5  880*del_song
#define NOTE_B5  988*del_song
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

/*-------------------------------------------------------------------------------------------------------------------*/

/*-------------------------------------------------------------------------------------------------------------------
 * Initialization functions and helper functions :
 *-------------------------------------------------------------------------------------------------------------------*/
 
// PWM for motor and buzzer :

// This is the PWM Signal needed for the buzzer

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

// Set freq for the buzzer.
void setFrequencyBuzzer(int frequency){
	int mod = (TIMER_CLCK_FREQ / frequency) - 1;
	TPM1->MOD = mod;
	TPM1_C0V = (mod+1)/2;
}


/*
// initPWM for the motor.
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
 SIM->SOPT2 |= SIM_SOPT2_TPMSRC(1); //NOT SURE

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

 // for TPM1 module
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

}
 
// Set freq for the moto control
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

*/

/*-------------------------------------------------------------------------------------------------------------------*/

// GPIO

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


/*-------------------------------------------------------------------------------------------------------------------*/

// LED :

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

/*-------------------------------------------------------------------------------------------------------------------*/

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

/*-------------------------------------------------------------------------------------------------------------------*/

//Initialize UART Module

void initUART2(uint32_t baud_rate) {
    uint32_t divisor, bus_clock;
 
    SIM->SCGC4 |= SIM_SCGC4_UART2_MASK;
    SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK;
 
    PORTE->PCR[UART_TX_PORTE22] &= ~PORT_PCR_MUX_MASK;
    PORTE->PCR[UART_TX_PORTE22] |= PORT_PCR_MUX(4);
 
    PORTE->PCR[UART_RX_PORTE23] &= ~PORT_PCR_MUX_MASK;
    PORTE->PCR[UART_RX_PORTE23] |= PORT_PCR_MUX(4);

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
 
    //UART2->C2 |= ((UART_C2_TE_MASK) | (UART_C2_RE_MASK));
    UART2->C2 |= ((UART_C2_TE_MASK) | (UART_C2_RE_MASK) | UART_C2_RIE_MASK);
    Q_Init(&RxQ);
}


/*-------------------------------------------------------------------------------------------------------------------*/


/*-------------------------------------------------------------------------------------------------------------------
 * UART Handler and Threads :
 *-------------------------------------------------------------------------------------------------------------------*/

//UART Handler :

void UART2_IRQHandler(void) {
    // check = 1;
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

// Threads :

void tAudio (void *argument) {
	// ...
	myDataPkt myRxData;
	int flag = 0;
  for (;;) {
		osMessageQueueGet(audioMsg, &myRxData, NULL, osWaitForever);
		// before starting the flag will be 0 hence there will be no sound.
		// once the connection is established we can play a connection tune and set the flag = 1.
		if(myRxData.cmd == 0x00) {
			
			// play unique connection tune.
			// These for the time beaing are random exerpts from the main tune :D
			for (int i=50;i<66;i++){              
			int wait = duration[i] * songspeed;
			setFrequencyBuzzer(notes[i]);
			osDelay(wait);
			}	      
			flag = 1;
		} 
		if (myRxData.cmd == 0x07) {
			
			//play victory tune
			for (int i=100;i<130;i++){              
			int wait = duration[i] * songspeed;
			setFrequencyBuzzer(notes[i]);
			osDelay(wait);
			}	      
			// Once challenge is over you can turn off the main tune.
			flag = 0;
		}
		if (flag ==1) {
			// Main tune if the challenge has started
			for (int i=0;i<203;i++){              //203 is the total number of music notes in the song
			int wait = duration[i] * songspeed;
			setFrequencyBuzzer(notes[i]);          //tone(pin,frequency,duration)
			osDelay(wait);
			}	              
		}
	}
}

/*-------------------------------------------------------------------------------------------------------------------*/

void led_green_thread (void *argument) {
	myDataPkt myRxData;
	// ...
  for (;;) {
		osMessageQueueGet(greenMsg, &myRxData, NULL, osWaitForever);
		if(myRxData.cmd ==0x00 || myRxData.cmd == 0x07 || myRxData.cmd == 0x08) {
			// light up all together when you are waiting
			
			PTC->PDOR |= MASK(GREEN_LED_1);
			PTC->PDOR |= MASK(GREEN_LED_2);
			PTC->PDOR |= MASK(GREEN_LED_3);
			PTC->PDOR |= MASK(GREEN_LED_4);
			PTC->PDOR |= MASK(GREEN_LED_5);
			PTC->PDOR |= MASK(GREEN_LED_6);
			PTC->PDOR |= MASK(GREEN_LED_7);
			PTC->PDOR |= MASK(GREEN_LED_8);
			osDelay(osDel);	
			
		} else {
			// light up in sequence while it is moving
			
			PTC->PDOR |= MASK(GREEN_LED_1);		
			osDelay(osDel);
			//delay(del_global);
			PTC->PDOR &= ~MASK(GREEN_LED_1);
			
			PTC->PDOR |= MASK(GREEN_LED_2);		
			osDelay(osDel);
			//delay(del_global);
			PTC->PDOR &= ~MASK(GREEN_LED_2);
			
			PTC->PDOR |= MASK(GREEN_LED_3);		
			osDelay(osDel);
			//delay(del_global);
			PTC->PDOR &= ~MASK(GREEN_LED_3);
			
			PTC->PDOR |= MASK(GREEN_LED_4);		
			osDelay(osDel);
			//delay(del_global);
			PTC->PDOR &= ~MASK(GREEN_LED_4);
			
			PTC->PDOR |= MASK(GREEN_LED_5);		
			osDelay(osDel);
			//delay(del_global);
			PTC->PDOR &= ~MASK(GREEN_LED_5);
			
			PTC->PDOR |= MASK(GREEN_LED_6);		
			osDelay(osDel);
			//delay(del_global);
			PTC->PDOR &= ~MASK(GREEN_LED_6);
			
			PTC->PDOR |= MASK(GREEN_LED_7);		
			osDelay(osDel);
			//delay(del_global);
			PTC->PDOR &= ~MASK(GREEN_LED_7);
			
			PTC->PDOR |= MASK(GREEN_LED_8);		
			osDelay(osDel);
			//delay(del_global);
			PTC->PDOR &= ~MASK(GREEN_LED_8);
		}
	}
}

/*-------------------------------------------------------------------------------------------------------------------*/

void led_red_thread (void *argument) {
	myDataPkt myRxData;
	// ...
  for (;;) {
		osMessageQueueGet(redMsg, &myRxData, NULL, osWaitForever);
		if(myRxData.cmd ==0x00 || myRxData.cmd == 0x07 || myRxData.cmd == 0x08) {
		PTD->PDOR |= MASK(RED_LED);		
		osDelay(250);
		//delay(del_global);
		PTD->PDOR &= ~MASK(RED_LED);
		osDelay(250);
		//delay(del_global);
		} else {
		PTD->PDOR |= MASK(RED_LED);		
		osDelay(500);
		//delay(del_global);
		PTD->PDOR &= ~MASK(RED_LED);
		osDelay(500);
		//delay(del_global);
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
  for (;;) {
		uint8_t data;
		// If there is a command which comes in then you have to execute it 
		if(!Q_Empty(&RxQ)) {
        data = Q_Dequeue(&RxQ);
				myData.cmd = data;
    } else {
				// Once the command is executed or if no command u go back into the waiting state
				myData.cmd = 0x08;
		}
		osMessageQueuePut(redMsg, &myData, NULL, 0);
		osMessageQueuePut(greenMsg, &myData, NULL, 0);
		osMessageQueuePut(audioMsg, &myData, NULL, 0);
		osMessageQueuePut(motorMsg, &myData, NULL, 0);
		osDelay(osDel);
	}
}

/*-------------------------------------------------------------------------------------------------------------------*/

void tMotorControl (void *argument) {
	
	
	/*
	// Test Code for Motor
	//PTC->PDOR |= MASK(MOTOR_POWER_1);		
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

	setFrequency(1, 0.2);
	osDelay(osDel);
	setFrequency(1, 0.9);
	osDelay(osDel);
	*/
	
	
	myDataPkt myRxData;
  for (;;) {
		osMessageQueueGet(motorMsg, &myRxData, NULL, osWaitForever);
		if(myRxData.cmd == 0x00 || myRxData.cmd == 0x08 || myRxData.cmd == 0x07){
			//No moving when in wait state, connection state or victory tune state
		} else if (myRxData.cmd == 0x01){
			//Forward
		} else if (myRxData.cmd == 0x02){
			//Left
		} else if (myRxData.cmd == 0x03){
			//Right
		} else if (myRxData.cmd == 0x04){
			//Back
		} else if (myRxData.cmd == 0x05){
			//CL
		} else if (myRxData.cmd == 0x06){
			//CR
		} else {
			//Error
		}
		osDelay(osDel);
	}
}

/*-------------------------------------------------------------------------------------------------------------------*/


/*-------------------------------------------------------------------------------------------------------------------
 * Main Function :
 *-------------------------------------------------------------------------------------------------------------------*/


int main(void) {
	// Setup
	SystemCoreClockUpdate();
	initPWM_temp();
	initLED();
	initGPIO();
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