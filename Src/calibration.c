#define MAX 0xff
#include <stdint.h>
#include <stdlib.h>
#include <flash.h>
#include <calibration.h>
#include <string.h>

struct STW_Calibration Calibration;

static uint32_t Flash_Address = 0x0803F800;

void SaveCalibration(){
 	Flash_ErasePage(FLASH_BANK_1, (uint8_t*) Flash_Address);
	Flash_Write((uint8_t*)Flash_Address,&Calibration,sizeof(Calibration));
}

void LoadCalibration(){
	memcpy(&Calibration,(uint8_t*)Flash_Address,sizeof(Calibration));
}
