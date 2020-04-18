#ifndef AUDIO_CONTROL_H_
#define AUDIO_CONTROL_H_

#include "common.h"

#define BUZZER 0

void initPWM_buzzer(void);
void setFrequencyBuzzer(int frequency);

#endif /* AUDIO_CONTROL_H_ */
