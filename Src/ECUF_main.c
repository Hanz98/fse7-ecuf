#include "main.h"
#include "stm32f1xx_hal.h"
#include <math.h>

#include "can_ECUF.h"
#include "can_eforce_init.h"
#include "ECUF_functions.h"


extern uint16_t soc;
extern uint64_t timer;

uint16_t counter = 100;
uint16_t counter1 = 0;

uint16_t tempor = 100;





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
	// setting up reading channels
	HAL_GPIO_WritePin(SEL_0_GPIO_Port,SEL_0_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(SEL_1_GPIO_Port,SEL_1_Pin, GPIO_PIN_RESET);

	//	enabling power to other units
	HAL_GPIO_WritePin(EN_DASH_VCC_GPIO_Port,EN_DASH_VCC_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(EN_DLTG_VCC_GPIO_Port,EN_DLTG_VCC_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(EN_ECUS_VCC_GPIO_Port,EN_ECUS_VCC_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(EN_TT_VCC_GPIO_Port,EN_TT_VCC_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(EN_ECUP_VCC_GPIO_Port,EN_ECUP_VCC_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(EN_ECUG_VCC_GPIO_Port,EN_ECUG_VCC_Pin, GPIO_PIN_SET);

	//	disable power to fans because they are turned on after reaching 150°C
	HAL_GPIO_WritePin(EN_FAN_BAR_VCC_GPIO_Port,EN_FAN_BAR_VCC_Pin, GPIO_PIN_RESET);

	// enabling power to sensors
	HAL_GPIO_WritePin(EN_I_SENSE_GPIO_Port,EN_I_SENSE_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(ECUS_DTLG_FRST_GPIO_Port, ECUS_DTLG_FRST_Pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(ECUP_FAN_FRST_GPIO_Port, ECUP_FAN_FRST_Pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(DASH_ECUG_FRST_GPIO_Port, DASH_ECUG_FRST_Pin,GPIO_PIN_RESET);

	// starting sequence of dashboard
	dashInit();
}


void loop(){
	/* temp */
	/*
	if (HAL_GPIO_ReadPin(SW2_GPIO_Port,SW2_Pin) == PRESS ){
		if (timer < HAL_GetTick()){



			counter--;

			if (counter == 0)
				counter = 100;

			if (soc < tempor)
				soc++;
			else if (soc > tempor)
				soc--;

			else {
				tempor = 1/sin(counter1);
				tempor %= 100;
				tempor *=2;
				tempor += 20;
				tempor %= 100;
				counter1++;
				if (counter1 == 100)
					counter1 = 0;

		}
			timer = HAL_GetTick() + 5;
	}
	}
	else if (HAL_GPIO_ReadPin(SW3_GPIO_Port,SW3_Pin) != PRESS ){
		if (timer < HAL_GetTick()){
			timer = HAL_GetTick() + 10;
			counter--;
			if (counter == 0)
				counter = 100;
			soc = counter;
		}

	}
	*/
	receiveData();
	checkDash(&dashBoard);
	checkShutdown(&statusFront);
	dashControl();
	sendData();
	dashBright();
	breakTemp();
	fanCheck();
}
