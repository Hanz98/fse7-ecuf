#ifndef __CAN_EFORCE_INIT_H__
#define __CAN_EFORCE_INIT_H__

#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_can.h"
#include "main.h"
#include "string.h"

int CAN_eforce(CAN_HandleTypeDef *hcan,CAN_TypeDef *Instance,CanRxMsgTypeDef*canRxMsg);

#endif
