#ifndef COMMON_H_
#define COMMON_H_

#include "RTE_Components.h"
#include  CMSIS_device_header
#include "cmsis_os2.h"
#include "MKL25Z4.h"
#include "custom_queue.h"

/*
	Common Helper interface.
	Common constants and modules included by most other header files.
*/

#define TIMER_CLCK_FREQ 375000
#define MASK(x) (1 << (x)) 
#define del_global 0x40000
#define osDel 2000

#endif /* COMMON_H_ */
