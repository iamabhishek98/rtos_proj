#include "RTE_Components.h"
#include  CMSIS_device_header
#include "cmsis_os2.h"
#include "MKL25Z4.h"

#include "common.h"
#include "songs.h"
#include "custom_queue.h"
#include "motor_control.h"
#include "led_control.h"
#include "audio_control.h"
#include "uart_control.h"

#define MSG_COUNT 5

// Thread ids for each individual task
osThreadId_t redLEDThreadTaskId, greenLEDThreadTaskId, motorThreadId, brainThreadId, audioThreadId ;

// Message Queue ids :
osMessageQueueId_t redMsg, greenMsg, motorMsg, audioMsg;

volatile int isFinished = 0;
volatile int musicCount = 0;
volatile int isMotorMoving = 0;
volatile Q_T TxQ, RxQ;

// We can have messages of : 
	
	// unmoving
	// 0x09 - Start state before bluetooth connects
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

enum RobotState
{
	START_STATE =             0x09,
	BLUETOOTH_CONNECT_STATE = 0x00,
	WAIT_STATE =              0x08,
	FINISHED_STATE =          0x07,
	
	MOVING_FORWARD_STATE =    0x01,
	MOVING_LEFT_STATE =       0x02,
	MOVING_RIGHT_STATE =      0x03,
	MOVING_BACKWARD_STATE =   0x04,
	MOVING_CURL_LEFT_STATE =  0x05,
	MOVING_CURL_RIGHT_STATE = 0x06,
};	

//Data Packet for Message Queue
typedef struct
{
	uint8_t cmd;
} myDataPkt;


// Threads :

void redLEDThreadTask (void *argument)
{
	myDataPkt myRxData;
  for (;;)
	{
		osMessageQueueGet(redMsg, &myRxData, NULL, osWaitForever);
		if (myRxData.cmd == BLUETOOTH_CONNECT_STATE || myRxData.cmd == START_STATE)
		{
			ledRedOff();
		}
		else if(myRxData.cmd == FINISHED_STATE || myRxData.cmd == WAIT_STATE)
		{
			// 250ms blink while robot stops.
			while (!isMotorMoving)
			{
				ledRedOn();	
				osDelay(250);
				ledRedOff();
				osDelay(250);
			}	
		} 
		else
		{
			// 500ms blink while robot is moving
			while (isMotorMoving)
			{
				ledRedOn();	
				osDelay(500);
				ledRedOff();
				osDelay(500);
			}	
		}
	}
}

void greenLEDThreadTask (void *argument)
{
	myDataPkt myRxData;
	
  for (;;)
	{
		osMessageQueueGet(greenMsg, &myRxData, NULL, osWaitForever);
		if (myRxData.cmd == START_STATE)
		{
			ledGreenOff();
		}
		else if(myRxData.cmd == BLUETOOTH_CONNECT_STATE)
		{
			ledGreenOn();
			osDelay(250);				
			ledGreenOff();
			osDelay(250);	
			ledGreenOn();
			osDelay(250);				
			ledGreenOff();
			osDelay(250);
		} 
		else if (myRxData.cmd == FINISHED_STATE || myRxData.cmd == WAIT_STATE)
		{
			// light up all together when you are waiting			
			ledGreenOn();	
		} 
		else
		{
			  // light up in sequence while it is moving
			
				ledGreenOff();
				while (isMotorMoving)
				{
					ledGreenRunning(80);
				}	
		}
	}
}


void audioThreadTask (void *argument)
{
	myDataPkt myRxData;
	int bluetoothState = 0;
  for (;;)
	{
		osMessageQueueGet(audioMsg, &myRxData, NULL, osWaitForever);
		// before starting the bluetoothState will be 0 hence there will be no sound.
		// once the connection is established we can play a connection tune and set the bluetoothState = 1.
		if(myRxData.cmd == BLUETOOTH_CONNECT_STATE)
		{
			// Play unique connection tune.
			// Unique connection tune is a small snippet of end song.
			for (int i=0;i<5;i++)
			{              
				int wait = end_duration[i] * songspeed;
				setFrequencyBuzzer(end_melody[i]);
				osDelay(wait);
			}
			bluetoothState = 1;
		} 
		else if (myRxData.cmd == FINISHED_STATE)
		{     
			// Once challenge is over you can turn off the main tune.
			bluetoothState = 2;
			isFinished = 1;
		}
		
		if (bluetoothState == 1 && !isFinished)
		{
			// Main tune if the challenge has started
			int endSongCount = musicCount+4;
			while (musicCount < endSongCount)
			{
				if (musicCount == 34)
				{
					musicCount = 0;
					endSongCount = 0;
				}
				int wait = running_duration[musicCount] * songspeed;
				setFrequencyBuzzer(running_melody[musicCount]);          //tone(pin,frequency,duration)
				musicCount++;
				osDelay(wait);
			}
			
		}	
		else if (bluetoothState == 2)
		{
			// Play victory tune
			while (isFinished)
			{
				for (int i=0;i<sizeof(end_melody);i++)
				{              
					int wait = end_duration[i] * songspeed;
					setFrequencyBuzzer(end_melody[i]);
					osDelay(wait);
				}	
				setFrequencyBuzzer(0);
			}
		}
	}
}

void motorThreadTask (void *argument)
{
	myDataPkt myRxData;
  for (;;)
	{
		int defaultMovementDuration = 500;
		osMessageQueueGet(motorMsg, &myRxData, NULL, osWaitForever);
			
		osMessageQueuePut(redMsg, &myRxData, NULL, 0);
		osMessageQueuePut(greenMsg, &myRxData, NULL, 0);
		
		if (!(myRxData.cmd == BLUETOOTH_CONNECT_STATE || 
			myRxData.cmd == FINISHED_STATE || 
			myRxData.cmd == WAIT_STATE))
		{
			isMotorMoving = 1;
		}
		
		if (myRxData.cmd == MOVING_FORWARD_STATE)
		{
			moveForward(0.9, defaultMovementDuration);
		} 
		else if (myRxData.cmd == MOVING_LEFT_STATE)
		{
			moveLeft(0.7, defaultMovementDuration);
		}
		else if (myRxData.cmd == MOVING_RIGHT_STATE)
		{
			moveRight(0.7, defaultMovementDuration);
		} 
		else if (myRxData.cmd == MOVING_BACKWARD_STATE)
		{
			moveBackward(0.9, defaultMovementDuration);
		} 
		else if (myRxData.cmd == MOVING_CURL_LEFT_STATE)
		{
			curlLeft(0.9, defaultMovementDuration, 3);
		}
		else if (myRxData.cmd == MOVING_CURL_RIGHT_STATE)
		{
			curlRight(0.9, defaultMovementDuration, 3);
		} 
		else
		{
			// myRxData.cmd == BLUETOOTH_CONNECT_STATE || myRxData.cmd == WAIT_STATE || myRxData.cmd == FINISHED_STATE
			// No moving when in wait state, connection state or victory tune state
			moveForward(0.0, 10);
		}
		
		osDelay(30); // allow motor to switch off
		isMotorMoving = 0;
		
	}
}

void brainThreadTask (void *argument)
{
	myDataPkt myData;
	myData.cmd = START_STATE;
  for (;;)
	{
		// If there is a command which comes in then you have to execute it 
		
		if (!isFinished)
		{
			if(!Q_Empty(&RxQ)) 
			{
				myData.cmd = Q_Dequeue(&RxQ);
			} 
			else if (myData.cmd != START_STATE)
			{
					// Once the command is executed or if no command u go back into the waiting state
					myData.cmd = WAIT_STATE;
			}
			osMessageQueuePut(audioMsg, &myData, NULL, 0);
			osMessageQueuePut(motorMsg, &myData, NULL, 0);
		}
		
		osDelay(1000); // Give CPU time for other tasks to run
	}
}


int main(void)
{
	// Setup
	SystemCoreClockUpdate();
	initPWMBuzzer();
	initPWM_motor();
	initLED();
	initGPIOMotor();
	initUART2(BAUD_RATE);
	
	osKernelInitialize();
	
	//Creation of the threads
	brainThreadId = osThreadNew(brainThreadTask, NULL, NULL);
	audioThreadId = osThreadNew(audioThreadTask, NULL, NULL);
	redLEDThreadTaskId = osThreadNew(redLEDThreadTask, NULL, NULL);    // Create application main thread
	greenLEDThreadTaskId = osThreadNew(greenLEDThreadTask, NULL, NULL);
	motorThreadId = osThreadNew(motorThreadTask, NULL, NULL);
	
	// Message Queues
	redMsg = osMessageQueueNew(MSG_COUNT, sizeof(myDataPkt), NULL);
	greenMsg = osMessageQueueNew(MSG_COUNT, sizeof(myDataPkt), NULL);
	audioMsg = osMessageQueueNew(MSG_COUNT, sizeof(myDataPkt), NULL);
	motorMsg = osMessageQueueNew(MSG_COUNT, sizeof(myDataPkt), NULL);
	
  osKernelStart();
	for(;;) {}
}
