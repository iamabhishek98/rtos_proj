#ifndef CUSTOM_QUEUE_H_
#define CUSTOM_QUEUE_H_

#define Q_SIZE (32)

/*
	Custom Queue Data Structure to handle 
	serial data from bluetooth.
*/

typedef struct
{
	unsigned char Data[Q_SIZE];
	unsigned int Head; // points to oldest data element
	unsigned int Tail; // points to next free space
	unsigned int Size; // quantity of elements in queue
} Q_T;

extern volatile Q_T TxQ, RxQ;

void Q_Init(Q_T * q);
int Q_Empty(Q_T * q);
int Q_Full(Q_T * q);
int Q_Enqueue(Q_T * q, unsigned char d);
unsigned char Q_Dequeue(Q_T * q);

#endif /* CUSTOM_QUEUE_H_ */
