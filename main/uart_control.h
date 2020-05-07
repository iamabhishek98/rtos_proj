#ifndef UART_CONTROL_H_
#define UART_CONTROL_H_

#include "common.h"

/*
	Module that helps with the Bluetooth UART setup and handling.
*/

#define FREQ 50
#define BAUD_RATE 9600
#define UART_TX_PORTE22 22
#define UART_RX_PORTE23 23
#define UART_INT_PRIO 128

void initUART2(uint32_t baud_rate);
void UART2_IRQHandler(void);

#endif /* UART_CONTROL_H_ */
