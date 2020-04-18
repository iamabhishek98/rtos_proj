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

//thread ids for led
osThreadId_t redLED_Id, greenLED_Id, motor_Id, brain_Id, audio_Id ;

// Message Queue ids :
osMessageQueueId_t redMsg, greenMsg, motorMsg, audioMsg;

volatile int isFinished = 0;
volatile int musicCount = 0;
volatile int isMotorMoving = 0;
volatile Q_T TxQ, RxQ;

//Data Packet for Message Queue
typedef struct {
	uint8_t cmd;
} myDataPkt;


// Threads :

void led_red_thread (void *argument)
{
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

void led_green_thread (void *argument)
{
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


void tAudio (void *argument)
{
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

void tMotorControl (void *argument)
{
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

void tBrain (void *argument)
{
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
