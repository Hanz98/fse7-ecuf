#include "can_ECUF.h"
#include "main.h"
#include "stm32f1xx_hal.h"


extern SPI_HandleTypeDef hspi2;

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

uint16_t dataRed[4];
uint16_t dataGreen[4];
uint16_t state = 0;
uint16_t soc = 0;
uint16_t lastsoc = 10;
uint64_t timer = 0;
uint64_t led = RED;
uint8_t shift = 0;
uint8_t tmp = 0;

extern ECUF_Status_t statusFront;
extern ECUF_STW_t steering;
extern ECUF_DISSusp_t disSup;
extern ECUF_Dashboard_t dashBoard;
extern ECUF_TEMPSuspF_t temp;

//	extern ECUF_REQCalibSTW_t calibration;

extern ECUA_Status_t statusA;
extern ECUB_Status_t statusBack;
extern VDCU_Status_t statusVdcu;
extern ECUP_Status_t statusP;
extern ECUA_Estimation_t estimationA;

void other(void){
	HAL_GPIO_WritePin(ECUS_DTLG_FRST_GPIO_Port, ECUS_DTLG_FRST_Pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(ECUP_FAN_FRST_GPIO_Port, ECUP_FAN_FRST_Pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(DASH_ECUG_FRST_GPIO_Port, DASH_ECUG_FRST_Pin,GPIO_PIN_RESET);

}


void checkDash(ECUF_Dashboard_t* data){
	if (HAL_GPIO_ReadPin(TSON_GPIO_Port,TSON_Pin) == PRESS)
		data->TSON = 1;
	else
		data->TSON = 0;
	if (HAL_GPIO_ReadPin(START_GPIO_Port,START_Pin) == PRESS)
		data->START = 1;
	else
		data->START = 0;
	if (HAL_GPIO_ReadPin(SW1_GPIO_Port,SW1_Pin) == PRESS )
		data->SW1 = 1;
	else
		data->SW1 = 0;
	if (HAL_GPIO_ReadPin(SW2_GPIO_Port,SW2_Pin) == PRESS )
		data->SW2 = 1;
	else
		data->SW2 = 0;
	if (HAL_GPIO_ReadPin(SW3_GPIO_Port,SW3_Pin) == PRESS )
		data->SW3 = 1;
	else
		data->SW3 = 0;
}

void checkShutdown(ECUF_Status_t* data){

	if (HAL_GPIO_ReadPin(SDBC_GPIO_Port,SDBC_Pin) == BREAK)
		data->SDC_SDBC = 1;
	else
		data->SDC_SDBC = 0;
	if (HAL_GPIO_ReadPin(INERTIA_GPIO_Port,INERTIA_Pin) == BREAK)
		data->SDC_Inertia = 1;
	else
		data->SDC_Inertia = 0;
	if (HAL_GPIO_ReadPin(CIS_GPIO_Port,CIS_Pin == BREAK))
		data->SDC_FWIL = 1;
	else
		data->SDC_FWIL = 0;

}


void dashInit(void){

	dataGreen[0] = 0xFFFF;
	dataRed[0] = 0xFFFF;

	HAL_GPIO_WritePin(DASH_STROBE_GPIO_Port, DASH_STROBE_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi2, (uint8_t*)dataRed, 4, 5);
	HAL_GPIO_WritePin(DASH_STROBE_GPIO_Port, DASH_STROBE_Pin, GPIO_PIN_SET);

	while(soc != 100){
	if (timer < HAL_GetTick()){
			timer = HAL_GetTick() + 5;
			soc++;
	}

	if (soc != lastsoc){
		led = RED;
		lastsoc = soc;
	shift = (100 - soc)/ 4;


	for (int16_t i = 3; i > 0 ; i--){
		led = RED;
		dataRed[i] = led >> (16 * ((i - 4) * -1 ) + shift*2);
		led = GREEN;
		dataGreen[i] = (led) >> (16 * ((i - 4) * -1 ) + shift*2);
		}
	}

		tmp = ((soc*soc)/10000.0)*100;
		state += 1;
		state %= 100;

	HAL_GPIO_WritePin(DASH_STROBE_GPIO_Port, DASH_STROBE_Pin, GPIO_PIN_RESET);

	if (tmp - state  < 0)
		HAL_SPI_Transmit(&hspi2, (uint8_t*)dataRed, 4, 5);
	else
		HAL_SPI_Transmit(&hspi2, (uint8_t*)dataGreen, 4, 5);

	HAL_GPIO_WritePin(DASH_STROBE_GPIO_Port, DASH_STROBE_Pin, GPIO_PIN_SET);
}

	while(soc != 0){
	if (timer < HAL_GetTick()){
			timer = HAL_GetTick() + 5;

			soc--;

	}

	if (soc != lastsoc){
		led = RED;
		lastsoc = soc;
	shift = (100 - soc)/ 4;


	for (int16_t i = 3; i > 0 ; i--){
		led = RED;
		dataRed[i] = led >> (16 * ((i - 4) * -1 ) + shift*2);
		led = GREEN;
		dataGreen[i] = (led) >> (16 * ((i - 4) * -1 ) + shift*2);
		}
	}

		tmp = ((soc*soc)/10000.0)*100;
		state += 1;
		state %= 100;

	HAL_GPIO_WritePin(DASH_STROBE_GPIO_Port, DASH_STROBE_Pin, GPIO_PIN_RESET);

	if (tmp - state  < 0)
		HAL_SPI_Transmit(&hspi2, (uint8_t*)dataRed, 4, 5);
	else
		HAL_SPI_Transmit(&hspi2, (uint8_t*)dataGreen, 4, 5);

	HAL_GPIO_WritePin(DASH_STROBE_GPIO_Port, DASH_STROBE_Pin, GPIO_PIN_SET);

	}
}

void indicatorControl(){
	if (statusBack.CarState < 2 ){
		dataGreen[0] &= EFORCE;
		dataRed[0] &= EFORCE;
	}
	else {
		dataGreen[0] &= 0x03FF;
		dataRed[0] &= 0x03FF;

	}
}
void barControl(){
	soc = estimationA.SOC/2;
	if (soc != lastsoc){
		led = RED;
		lastsoc = soc;
	shift = (100 - soc)/ 4;


	for (int16_t i = 3; i > 0 ; i--){
		led = RED;
		dataRed[i] = led >> (16 * ((i - 4) * -1 ) + shift*2);
		led = GREEN;
		dataGreen[i] = (led) >> (16 * ((i - 4) * -1 ) + shift*2);
		}
	}

		tmp = ((soc*soc)/10000.0)*100;
		state += 1;
		state %= 100;

}


void dashControl(){
	barControl();
	indicatorControl();
	HAL_GPIO_WritePin(DASH_STROBE_GPIO_Port, DASH_STROBE_Pin, GPIO_PIN_RESET);
	if (tmp - state  < 0)
		HAL_SPI_Transmit(&hspi2, (uint8_t*)dataRed, 4, 5);
	else
		HAL_SPI_Transmit(&hspi2, (uint8_t*)dataGreen, 4, 5);

	HAL_GPIO_WritePin(DASH_STROBE_GPIO_Port, DASH_STROBE_Pin, GPIO_PIN_SET);





}


void sendData(void){

	if(ECUF_Status_need_to_send()){
			ECUF_send_Status_s(&statusFront);
			HAL_CAN_Receive_IT(&hcan1,CAN_FIFO0);
	    }

	if (ECUF_Dashboard_need_to_send()){
		ECUF_send_Dashboard_s(&dashBoard);
		HAL_CAN_Receive_IT(&hcan1,CAN_FIFO0);
	 }

	 if (ECUF_DISSusp_need_to_send()){
		ECUF_send_DISSusp_s(&disSup);
  	    HAL_CAN_Receive_IT(&hcan1,CAN_FIFO0);
	}

	if (ECUF_TEMPSuspF_need_to_send()){
		ECUF_send_TEMPSuspF_s(&temp);
		HAL_CAN_Receive_IT(&hcan2,CAN_FIFO0);
	}

	if(ECUF_STW_need_to_send()){
		ECUF_send_STW_s(&steering);
  	    HAL_CAN_Receive_IT(&hcan1,CAN_FIFO0);
	}
	HAL_CAN_Receive_IT(&hcan1,CAN_FIFO0);
}

void receiveData(void){
	  if(ECUB_get_Status(&statusBack) & CAN_MSG_PENDING){
	  }
	  if(ECUP_get_Status(&statusP) & CAN_MSG_PENDING){
	  }
	  if(ECUA_get_Status(&statusA) & CAN_MSG_PENDING){
	  }
	  if(VDCU_get_Status(&statusVdcu) & CAN_MSG_PENDING){
	  }
	  if(ECUA_get_Estimation(&estimationA) & CAN_MSG_PENDING){
	  }


}
