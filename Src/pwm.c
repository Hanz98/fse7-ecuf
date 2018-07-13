#include <stm32f1xx_hal.h>
#include "pwm.h"

enum {
	kTimeout = 20,
};

void PwmInput_Init(PwmInput* p) {
	p->start = 0;
	p->time = 0;
	p->validAt = 0;
}

int PwmInput_Get(PwmInput* p, uint16_t* value_out) {
	if (HAL_GetTick() > p->validAt + kTimeout)
		return 0;

	// "This should never happen"
	if (p->time < 8)
		return 0;

	// 1 us = 8 timer cycles
	// min. period is    1 us (reading =    0)
	// max. period is 4096 us (reading = 4095)

	// Therefore, time <8, 32768 + 8) maps to output <0; 65535>

	*value_out = (p->time - 8) * 2; // 2 = 65535/4096/8
	return 1;
}
