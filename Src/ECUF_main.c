#include "main.h"
#include "stm32f1xx_hal.h"

#include "eforce/tx.h"
#include "can_ECUF.h"
#include "can_eforce_init.h"
#include "eforce/can.h"
#include "ECUF_functions.h"

ECUF_Status_t statusFront;
ECUF_STW_t steering;
ECUF_DISSusp_t disSup;
ECUF_Dashboard_t dashBoard;
ECUF_TEMPSuspF_t temp;

ECUF_REQCalibSTW_t calibration;

ECUA_Status_t statusA;
ECUB_Status_t statusBack;
VDCU_Status_t statusVdcu;
ECUP_Status_t statusP;
ECUA_Estimation_t estimationA;

void setup(){
	HAL_GPIO_WritePin(EN_DASH_VCC_GPIO_Port,EN_DASH_VCC_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(EN_DLTG_VCC_GPIO_Port,EN_DLTG_VCC_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(EN_ECUS_VCC_GPIO_Port,EN_ECUS_VCC_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(EN_TT_VCC_GPIO_Port,EN_TT_VCC_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(EN_ECUP_VCC_GPIO_Port,EN_ECUP_VCC_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(EN_FAN_BAR_VCC_GPIO_Port,EN_FAN_BAR_VCC_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(EN_ECUG_VCC_GPIO_Port,EN_ECUG_VCC_Pin, GPIO_PIN_SET);

	HAL_GPIO_WritePin(EN_I_SENSE_GPIO_Port,EN_I_SENSE_Pin, GPIO_PIN_SET);

	other();
	dashInit();
}

void loop(){
	receiveData();
	checkDash(&dashBoard);
	checkShutdown(&statusFront);
	dashControl();
	sendData();

}
