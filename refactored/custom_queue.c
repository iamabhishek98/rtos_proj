#include "custom_queue.h"

// Initialize the Queue
void Q_Init(volatile Q_T * q)
{
	unsigned int i;
	for (i=0; i<Q_SIZE; i++)
		q->Data[i] = 0; // to simplify our lives when debugging
	q->Head = 0;
	q->Tail = 0;
	q->Size = 0;
}
 
int Q_Empty(volatile Q_T * q)
{
	return q->Size == 0;
}

int Q_Full(volatile Q_T * q)
{
	return q->Size == Q_SIZE;
}
 
// Enqueue new data into the queue, 
// returning false if queue is full
int Q_Enqueue(volatile Q_T * q, unsigned char d)
{
	if (!Q_Full(q)) {
			q->Data[q->Tail++] = d;
			q->Tail %= Q_SIZE;
			q->Size++;
			return 1; // success
	} else
			return 0; // failure
}

// Get the oldest data (FIFO) to dequeue if
// the queue is non-empty
unsigned char Q_Dequeue(volatile Q_T * q)
{
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
