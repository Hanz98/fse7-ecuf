#include <tx2/can.h>
#include "can_eforce_init.h"
#include "stm32f1xx_hal.h"
#include "can_ECUF.h"
#include "can.h"

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
CanTxMsgTypeDef txMsg;

uint32_t txGetTimeMillis() {
    /* Recommended implementation; you can use your own timing functions if needed */
    return (uint32_t)HAL_GetTick();
}

int txHandleCANMessage(uint32_t timestamp, int bus, CAN_ID_t id, const void* data, size_t length) {
    /* This can be empty if you don't need to pre-process the received messages */
    return 0;
}

static int SendCANMessage(CAN_HandleTypeDef* hcan, CAN_ID_t id, const void* data, size_t length) {
	memset(&txMsg, 0, sizeof(CanTxMsgTypeDef) );
	txMsg.IDE = CAN_ID_STD;
	txMsg.RTR = CAN_RTR_DATA;
	txMsg.StdId = id;
	txMsg.DLC = length;
	memcpy(txMsg.Data, data, length);

	hcan->pTxMsg = &txMsg;

	if (HAL_CAN_Transmit(hcan, 10) == HAL_OK) {
		//status_post(STATUS_OK_CAN_SEND);
		return 1;
	}
	else {
		//status_post(STATUS_WARN_CAN_SEND_FAIL);
		return 0;
	}
}

int txSendCANMessage(int bus, CAN_ID_t id, const void* data, size_t length) {
	if (bus == bus_CAN1_powertrain) {
		SendCANMessage(&hcan1, id, data, length);
	}
	else if (bus == bus_CAN2_aux) {
		SendCANMessage(&hcan2, id, data, length);
	}
}
