/*----------------------------------------------------------------------------
 * CMSIS-RTOS 'main' function template
 *---------------------------------------------------------------------------*/
 
#include "RTE_Components.h"
#include  CMSIS_device_header
#include "cmsis_os2.h"

#include "MKL25Z4.h"

#define GREEN_LED_1 7
#define GREEN_LED_2 0 
#define GREEN_LED_3 3
#define GREEN_LED_4 4
#define GREEN_LED_5 5
#define GREEN_LED_6 6
#define GREEN_LED_7 10
#define GREEN_LED_8 11

#define RED_LED 6

#define MASK(x) (1 << (x)) 
#define del 0x40000
#define osDel 250

osThreadId_t redLED_Id, greenLED_Id;

void initLED(void) {
	/* enable clock for port C & D */ 
	SIM->SCGC5 |=  SIM_SCGC5_PORTC_MASK | SIM_SCGC5_PORTD_MASK; 
	
	/* Select GPIO and enable pull-up resistors and interrupts on falling edges for pin connected to switch */ 
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

void delay(long long mil) {
	int count = 0xF000;
	for (long long i = mil; i > 0; i--) {
		count++;
	}
}

void led_green_thread (void *argument) {
	
	// ...
  for (;;) {
		PTC->PDOR |= MASK(GREEN_LED_1);		
		osDelay(osDel);
		//delay(del);
		PTC->PDOR &= ~MASK(GREEN_LED_1);
		
		PTC->PDOR |= MASK(GREEN_LED_2);		
		osDelay(osDel);
		//delay(del);
		PTC->PDOR &= ~MASK(GREEN_LED_2);
		
		PTC->PDOR |= MASK(GREEN_LED_3);		
		osDelay(osDel);
		//delay(del);
		PTC->PDOR &= ~MASK(GREEN_LED_3);
		
		PTC->PDOR |= MASK(GREEN_LED_4);		
		osDelay(osDel);
		//delay(del);
		PTC->PDOR &= ~MASK(GREEN_LED_4);
		
		PTC->PDOR |= MASK(GREEN_LED_5);		
		osDelay(osDel);
		//delay(del);
		PTC->PDOR &= ~MASK(GREEN_LED_5);
		
		PTC->PDOR |= MASK(GREEN_LED_6);		
		osDelay(osDel);
		//delay(del);
		PTC->PDOR &= ~MASK(GREEN_LED_6);
		
		PTC->PDOR |= MASK(GREEN_LED_7);		
		osDelay(osDel);
		//delay(del);
		PTC->PDOR &= ~MASK(GREEN_LED_7);
		
		PTC->PDOR |= MASK(GREEN_LED_8);		
		osDelay(osDel);
		//delay(del);
		PTC->PDOR &= ~MASK(GREEN_LED_8);
	}
}

void led_red_thread (void *argument) {
	
	// ...
  for (;;) {
		PTD->PDOR |= MASK(RED_LED);		
		osDelay(osDel);
		//delay(del);
		PTD->PDOR &= ~MASK(RED_LED);
		osDelay(osDel);
		//delay(del);
	}
}

int main(void) {
	SystemCoreClockUpdate();
	 
	initLED();
	
	osKernelInitialize();
	
	redLED_Id = osThreadNew(led_red_thread, NULL, NULL);    // Create application main thread
	greenLED_Id = osThreadNew(led_green_thread, NULL, NULL); 
  osKernelStart();
	for(;;) {}
}
