#include "main.h"
#include "stm32f1xx_hal.h"
#include "flash.c"

#define DEGREE 0

uint32_t PWM_Period = 0;
uint32_t STRW_Sensor_Reading = 0;
uint32_t STW_last_interrupt = 0;

struct Calibration{
	uint32_t Left;
	uint32_t Right;
	uint32_t Center;

}Calibration;

extern TIM_HandleTypeDef htim1;
extern ECUF_REQCalibSTW_t calibration;

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim);
