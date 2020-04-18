#include "uart_control.h"

// Initialize UART Module
void initUART2(uint32_t baud_rate)
{
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

void UART2_IRQHandler(void)
{
    if (UART2->S1 & UART_S1_RDRF_MASK) {
        // received a character
        if (!Q_Full(&RxQ)) {
        Q_Enqueue(&RxQ, UART2->D);
        } else {
        // error -queue full.
        }
    }
}
