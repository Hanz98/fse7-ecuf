#include "can_ECUF.h"
#include "main.h"
#include "stm32f1xx_hal.h"
#include "pwmRead.h"
extern SPI_HandleTypeDef hspi1;
extern SPI_HandleTypeDef hspi2;

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

int32_t value;
int32_t valueControl = 0;

int16_t tmpPhoto = 50;

uint64_t timer = 0;
uint64_t timer2 = 0;
uint64_t led = RED;

uint16_t dataRed[4];
uint16_t dataGreen[4];
uint16_t brakeTempL[2];
uint16_t brakeTempR[2];
uint16_t state = 0;
uint16_t soc = 0;
uint16_t lastsoc = 10;
uint16_t fanPer;

uint8_t shift = 0;
uint8_t tmp = 0;
uint8_t i = 0;

extern uint32_t adc[9];
extern int time1;
extern enum photoState stateP;
extern uint16_t photo;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim3;

extern ECUF_Status_t statusFront;
extern ECUF_STW_t steering;
extern ECUF_DISSusp_t disSup;
extern ECUF_Dashboard_t dashBoard;
extern ECUF_TEMPSuspF_t temp;

extern ECUA_Status_t statusA;
extern ECUB_Status_t statusBack;
extern VDCU_Status_t statusVdcu;
extern ECUP_Status_t statusP;
extern ECUA_Estimation_t estimationA;


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
		data->WP_ON = 1;
	else
		data->WP_ON = 0;
	if (HAL_GPIO_ReadPin(SW2_GPIO_Port,SW2_Pin) == PRESS )
		data->TCS_ON = 1;
	else
		data->TCS_ON = 0;
	if (HAL_GPIO_ReadPin(SW3_GPIO_Port,SW3_Pin) == PRESS )
		data->YC_ON = 1;
	else
		data->YC_ON = 0;


}

void checkShutdown(ECUF_Status_t* data){
	steering.Angle = time1;
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

		HAL_Delay(100);
		dataGreen[0] += STABILIZATION;
		HAL_GPIO_WritePin(DASH_STROBE_GPIO_Port, DASH_STROBE_Pin, GPIO_PIN_RESET);
			HAL_SPI_Transmit(&hspi2, (uint8_t*)dataGreen, 4, 5);
		HAL_GPIO_WritePin(DASH_STROBE_GPIO_Port, DASH_STROBE_Pin, GPIO_PIN_SET);

		HAL_Delay(100);
		dataGreen[0] += TRACTION;
		HAL_GPIO_WritePin(DASH_STROBE_GPIO_Port, DASH_STROBE_Pin, GPIO_PIN_RESET);
		HAL_SPI_Transmit(&hspi2, (uint8_t*)dataGreen, 4, 5);
		HAL_GPIO_WritePin(DASH_STROBE_GPIO_Port, DASH_STROBE_Pin, GPIO_PIN_SET);

		HAL_Delay(100);
		dataGreen[0] += IMPLAUSILITY;
		HAL_GPIO_WritePin(DASH_STROBE_GPIO_Port, DASH_STROBE_Pin, GPIO_PIN_RESET);
			HAL_SPI_Transmit(&hspi2, (uint8_t*)dataGreen, 4, 5);
		HAL_GPIO_WritePin(DASH_STROBE_GPIO_Port, DASH_STROBE_Pin, GPIO_PIN_SET);

		HAL_Delay(100);
		dataGreen[0] += EFORCE;
		HAL_GPIO_WritePin(DASH_STROBE_GPIO_Port, DASH_STROBE_Pin, GPIO_PIN_RESET);
			HAL_SPI_Transmit(&hspi2, (uint8_t*)dataGreen, 4, 5);
		HAL_GPIO_WritePin(DASH_STROBE_GPIO_Port, DASH_STROBE_Pin, GPIO_PIN_SET);

		HAL_Delay(100);
		dataGreen[0] += TEMP_ER;
		dataGreen[0] += RTD;
		HAL_GPIO_WritePin(DASH_STROBE_GPIO_Port, DASH_STROBE_Pin, GPIO_PIN_RESET);
			HAL_SPI_Transmit(&hspi2, (uint8_t*)dataGreen, 4, 5);
		HAL_GPIO_WritePin(DASH_STROBE_GPIO_Port, DASH_STROBE_Pin, GPIO_PIN_SET);

		HAL_Delay(100);
		dataRed[0] += LW_ER;
		HAL_GPIO_WritePin(DASH_STROBE_GPIO_Port, DASH_STROBE_Pin, GPIO_PIN_RESET);
			HAL_SPI_Transmit(&hspi2, (uint8_t*)dataGreen, 4, 5);
		HAL_GPIO_WritePin(DASH_STROBE_GPIO_Port, DASH_STROBE_Pin, GPIO_PIN_SET);


		HAL_Delay(100);
		dataGreen[0] += CAN_ER;
		HAL_GPIO_WritePin(DASH_STROBE_GPIO_Port, DASH_STROBE_Pin, GPIO_PIN_RESET);
			HAL_SPI_Transmit(&hspi2, (uint8_t*)dataGreen, 4, 5);
		HAL_GPIO_WritePin(DASH_STROBE_GPIO_Port, DASH_STROBE_Pin, GPIO_PIN_SET);


	while(soc != 100){

	if (timer < HAL_GetTick()){
			timer = HAL_GetTick() + 10;
			soc++;
	}

	if (soc != lastsoc){
		led = RED;
		lastsoc = soc;
		soc = (soc * 2) / 2;
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
			timer = HAL_GetTick() + 10;

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
	dataGreen[0] = 0x0000;
	dataRed[0] = 0x0000;
	soc = 50;
}

void indicatorControl(){

	if (statusBack.CarState < 2 ){
		dataGreen[0] |= EFORCE;
		dataRed[0] |= EFORCE;
	}
	else {
		dataGreen[0] &= RE_EFORCE;
		dataRed[0] &= RE_EFORCE;
	}
	/*
	 *if (statusBack.CarState < 2 ){
		dataGreen[0] &= CAN_ER;
		dataRed[0] &= CAN_ER;
	}
	else {
		dataGreen[0] &= RE_CAN_ER;
		dataRed[0] &= RE_CAN_ER;
	}
	if (statusBack.CarState < 2 ){
		dataGreen[0] &= LW_ER;
		dataRed[0] &= LW_ER;
	}
	else {
		dataGreen[0] &= RE_LW_ER;
		dataRed[0] &= RE_LW_ER;
	}

	if (statusBack.CarState < 2 ){
		dataGreen[0] &= TEMP_ER;
		dataRed[0] &= TEMP_ER;
	}

	else {
		dataGreen[0] &= RE_TEMP_ER;
		dataRed[0] &= RE_TEMP_ER;
	}
*/
	if (!statusVdcu.YC_ENABLED ){
		dataGreen[0] |= STABILIZATION;
		dataRed[0] |= STABILIZATION;
	}
	else {
		dataGreen[0] &= RE_STABILIZATION;
		dataRed[0] &= RE_STABILIZATION;
	}
	if (!statusVdcu.TC_ENABLED){
		dataGreen[0] &= TRACTION;
		dataRed[0] &= TRACTION;
	}
	else {
		dataGreen[0] &= RE_TRACTION;
		dataRed[0] &= RE_TRACTION;
	}

	if (statusP.APPS_Plausible){
		dataGreen[0] |= IMPLAUSILITY;
		dataRed[0] |= IMPLAUSILITY;
	}
	else {
		dataGreen[0] &= RE_IMPLAUSILITY;
		dataRed[0] &= RE_IMPLAUSILITY;
	}

	if (statusVdcu.State == 1){
		dataGreen[0] |= RTD;
		dataRed[0] |= RTD;
	}
	else {
		dataGreen[0] &= RE_RTD;
		dataRed[0] &= RE_RTD;
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
int32_t rescale(int32_t value, float oldMin, float oldMax, float newMin, float newMax){
	value = ((newMax - newMin) / (oldMax - oldMin)) * (value - oldMax) + newMax;
	return value;
}

void suspensionDis(){
	disSup.FL = rescale(adc[5], 190, 3531, 0, 75) * 0.01;
	disSup.FR = rescale(adc[6], 192, 3558, 0, 75) * 0.01;
	disSup.RR = rescale(adc[7], 193, 3588, 0, 75) * 0.01;
	disSup.RL = rescale(adc[8], 192, 3554, 0, 75) * 0.01;
}
void breakTemp(){
	uint16_t temperature = 0 ;

	if (!HAL_GPIO_ReadPin(DRDY_Pt1000_R_GPIO_Port,DRDY_Pt1000_R_Pin)){
			HAL_GPIO_WritePin(CS_Pt1000_R_GPIO_Port,CS_Pt1000_R_Pin,GPIO_PIN_RESET);
			HAL_SPI_Transmit(&hspi1, (uint8_t*)0x01, 1, 60);

			for (int x = 0; x < 2; x++ ){
				HAL_SPI_Receive	(&hspi1,(uint8_t*)&brakeTempR[x],1, 5);
			}
			HAL_GPIO_WritePin(CS_Pt1000_R_GPIO_Port,CS_Pt1000_R_Pin,GPIO_PIN_SET);

			temperature = brakeTempR[1] * 256 + brakeTempR[0];
			temperature = temperature / 32 - 256;

			temp.BrakeCal_FR = temperature * 2;

	}
	else {
		HAL_GPIO_WritePin(CS_Pt1000_R_GPIO_Port,CS_Pt1000_R_Pin,GPIO_PIN_SET);
	}

	if (!HAL_GPIO_ReadPin(DRDY_Pt1000_L_GPIO_Port,DRDY_Pt1000_L_Pin)){
			HAL_GPIO_WritePin(CS_Pt1000_L_GPIO_Port,CS_Pt1000_L_Pin,GPIO_PIN_RESET);
			HAL_SPI_Transmit(&hspi1, (uint8_t*)0x01, 1, 60);

			for (int x = 0; x < 2; x++ ){
				HAL_SPI_Receive	(&hspi1,(uint8_t*)&brakeTempL[x],1, 5);
			}
			HAL_GPIO_WritePin(CS_Pt1000_L_GPIO_Port,CS_Pt1000_L_Pin,GPIO_PIN_SET);

			temperature = brakeTempL[1] * 256 + brakeTempL[0];
			temperature = temperature / 32 - 256;


			temp.BrakeCal_FL = temperature * 2;
	}
	else {
		HAL_GPIO_WritePin(CS_Pt1000_L_GPIO_Port,CS_Pt1000_L_Pin,GPIO_PIN_SET);
	}

}


void dashBright(void){


	tmpPhoto = tmpPhoto - (0.25 * (tmpPhoto - photo));
	dashBoard.AmbientLight = tmpPhoto;

	value = tmpPhoto;
	value = rescale(value, 0.0, 255.0, 0.0, 100.0);

	value -= 100;
	value *= -1;

	value = rescale(value, 0.0, 100.0, 20.0, 95.0);

	if (timer2 < HAL_GetTick()){

		timer2 = HAL_GetTick() + 2000;
		if (	stateP == LONG){
		valueControl = value;
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, value);

		}
		stateP = LONG;
	}
	else {
		if (value - 10 > valueControl && value + 10 < valueControl){
			stateP = SHORT;
		}
	}
}



void fanCheck(){

	if (temp.BrakeCal_FL * 2 > 150 || temp.BrakeCal_FR * 2> 150){
		HAL_GPIO_WritePin(EN_FAN_BAR_VCC_GPIO_Port,EN_FAN_BAR_VCC_Pin, GPIO_PIN_SET);

		if (temp.BrakeCal_FL * 2 > 100)
			fanPer = rescale(temp.BrakeCal_FL * 2, 100, 400, 0, 100);
		if (temp.BrakeCal_FR * 2 > 100)
			fanPer = rescale(temp.BrakeCal_FR * 2, 100, 400, 0, 100);

		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, fanPer);
	}
	else {
		HAL_GPIO_WritePin(EN_FAN_BAR_VCC_GPIO_Port,EN_FAN_BAR_VCC_Pin, GPIO_PIN_RESET);

	}

}

void STRWRead(){
	steering.Angle = Get_STRW_Calibrated_Angle() * 0.1;
	if (HAL_GPIO_ReadPin(SWS_ERR_IN_GPIO_Port,SWS_ERR_IN_Pin)){
		steering.FT_STW = 1;
	}
	else
		steering.FT_STW = 0;
}
