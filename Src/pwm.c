#include <stdint.h>
#include <stm32f1xx_hal.h>


typedef struct pwm {
	uint16_t start;
	uint16_t time;
	uint32_t validAt;
} pwm ;
/*
void initPWM(struct pwm* pwm){
	pwm->start = 0;
	pwm->time = 0;
	pwm->validAt = 0;
}

void get(struct pwm* pwm, uint16_t* outputValue){
	if (pwm->validAt + 20 > HAL_GetTick())
		return;
	*outputValue = (pwm->time - 8) * 2;
}
*/
