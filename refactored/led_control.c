#include "led_control.h"

// Initialize LED GPIO configuration
void initLED(void)
{
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

// Turn on Array of 8 LEDs
void ledGreenOn()
{
	PTC->PDOR |= MASK(GREEN_LED_1);
	PTC->PDOR |= MASK(GREEN_LED_2);
	PTC->PDOR |= MASK(GREEN_LED_3);
	PTC->PDOR |= MASK(GREEN_LED_4);
	PTC->PDOR |= MASK(GREEN_LED_5);
	PTC->PDOR |= MASK(GREEN_LED_6);
	PTC->PDOR |= MASK(GREEN_LED_7);
	PTC->PDOR |= MASK(GREEN_LED_8);
}

// Turn off Array of 8 LEDs
void ledGreenOff()
{
	PTC->PDOR &= ~MASK(GREEN_LED_1);
	PTC->PDOR &= ~MASK(GREEN_LED_2);
	PTC->PDOR &= ~MASK(GREEN_LED_3);
	PTC->PDOR &= ~MASK(GREEN_LED_4);
	PTC->PDOR &= ~MASK(GREEN_LED_5);
	PTC->PDOR &= ~MASK(GREEN_LED_6);
	PTC->PDOR &= ~MASK(GREEN_LED_7);
	PTC->PDOR &= ~MASK(GREEN_LED_8);
}

/* 
	Play running sequence of Array of Green LEDs
	while the vehicle is moving. LED sequence
	abrubtly stops once motor stops moving 
  (sequence doesnt complete).
*/
void ledGreenRunning(int delay)
{
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

void ledRedOn()
{
	PTD->PDOR |= MASK(RED_LED);		
}

void ledRedOff()
{
	PTD->PDOR &= ~MASK(RED_LED);	
}
