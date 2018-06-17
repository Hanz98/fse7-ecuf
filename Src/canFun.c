#include <eforce/tx.h>
#include "can_eforce_init.h"
#include "stm32f1xx_hal.h"
#include "can_ECUF.h"
extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
CanTxMsgTypeDef txMsg;

uint32_t txGetTimeMillis() {
    /* Recommended implementation; you can use your own timing functions if needed */
    return (uint32_t)HAL_GetTick();
}

int txHandleCANMessage(uint32_t timestamp, CAN_ID_t id, const void* data, size_t length) {
    /* This can be empty if you don't need to pre-process the received messages */
    return 0;
}

int txSendCANMessage(int bus, CAN_ID_t id, const void* data, size_t length) {
	txMsg.StdId = id;
	txMsg.IDE = CAN_ID_STD;
	txMsg.RTR = CAN_RTR_DATA;
	txMsg.DLC = length;
	memcpy(txMsg.Data, data, length);
	if (bus ==bus_CAN1_powertrain){
		hcan1.pTxMsg = &txMsg;
		return HAL_CAN_Transmit_IT(&hcan1);
	}
	if (bus ==bus_CAN2_aux){
			hcan2.pTxMsg = &txMsg;
			return HAL_CAN_Transmit_IT(&hcan2);
	}
	return HAL_CAN_Transmit_IT(&hcan1);
}
