#ifndef PWM_H_
#define PWM_H_

#include <stdint.h>

typedef struct {
	uint16_t start, time;
	uint32_t validAt;
} PwmInput;

void PwmInput_Init(PwmInput* p);
int PwmInput_Get(PwmInput* p, uint16_t* value_out);

#endif
