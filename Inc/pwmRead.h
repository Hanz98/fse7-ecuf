#include "main.h"
#include "stm32f1xx_hal.h"
//#include "../Src/flash.c"

#include "can_ECUF.h"

#define DEGREE 16

extern uint32_t PWM_Period;
extern uint32_t STRW_Sensor_Reading;
extern uint32_t STW_last_interrupt;

struct Calibration{
	uint32_t Left;
	uint32_t Right;
	uint32_t Center;

}Calibration;

extern TIM_HandleTypeDef htim1;
extern ECUF_REQCalibSTW_t calibration;

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim);
