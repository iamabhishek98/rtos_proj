#ifndef AUDIO_CONTROL_H_
#define AUDIO_CONTROL_H_

#include "common.h"

/*
	Module in charge of audio control / buzzer control.
	Used to control frequencies and play songs.
*/

#define BUZZER 0

void initPWMBuzzer(void);
void setFrequencyBuzzer(int frequency);

#endif /* AUDIO_CONTROL_H_ */
