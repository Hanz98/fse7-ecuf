#include "can_ECUF.h"
#include <string.h>

CAN_msg_status_t ECUA_Status_status;
ECUA_Status_t ECUA_Status_data;
CAN_msg_status_t ECUA_Estimation_status;
ECUA_Estimation_t ECUA_Estimation_data;
CAN_msg_status_t ECUB_Status_status;
ECUB_Status_t ECUB_Status_data;
CAN_msg_status_t ECUB_Cooling_status;
ECUB_Cooling_t ECUB_Cooling_data;
int32_t ECUF_Status_last_sent;
int32_t ECUF_STW_last_sent;
int32_t ECUF_DISSusp_last_sent;
int32_t ECUF_Dashboard_last_sent;
int32_t ECUF_TEMPSuspF_last_sent;
CAN_msg_status_t ECUF_REQCalibSTW_status;
ECUF_REQCalibSTW_t ECUF_REQCalibSTW_data;
CAN_msg_status_t ECUP_Status_status;
ECUP_Status_t ECUP_Status_data;
CAN_msg_status_t VDCU_Status_status;
VDCU_Status_t VDCU_Status_data;

void candbInit(void) {
    canInitMsgStatus(&ECUA_Status_status, 500);
    canInitMsgStatus(&ECUA_Estimation_status, 500);
    canInitMsgStatus(&ECUB_Status_status, 1000);
    canInitMsgStatus(&ECUB_Cooling_status, -1);
    ECUF_Status_last_sent = -1;
    ECUF_STW_last_sent = -1;
    ECUF_DISSusp_last_sent = -1;
    ECUF_Dashboard_last_sent = -1;
    ECUF_TEMPSuspF_last_sent = -1;
    canInitMsgStatus(&ECUF_REQCalibSTW_status, -1);
    canInitMsgStatus(&ECUP_Status_status, 500);
    canInitMsgStatus(&VDCU_Status_status, 500);
}

int ECUA_decode_Status_s(const uint8_t* bytes, size_t length, ECUA_Status_t* data_out) {
    if (length < 7)
        return 0;

    data_out->SDC_IN_Open = (bytes[0] & 0x01);
    data_out->SDC_HV_ILOCK = ((bytes[0] >> 1) & 0x01);
    data_out->SDC_IMD = ((bytes[0] >> 2) & 0x01);
    data_out->SDC_AMS = ((bytes[0] >> 3) & 0x01);
    data_out->SDC_OUT = ((bytes[0] >> 4) & 0x01);
    data_out->SDC_END = ((bytes[0] >> 5) & 0x01);
    data_out->LATCH_SDC_AMS = ((bytes[0] >> 7) & 0x01);
    data_out->AIRsState = (bytes[1] & 0x0F);
    data_out->ChargingState = ((bytes[1] >> 4) & 0x0F);
    data_out->AMSState = (enum ECUA_StateAMS) (bytes[2]);
    data_out->FT_ACP_OT = (bytes[3] & 0x01);
    data_out->FT_AIRS = ((bytes[3] >> 1) & 0x01);
    data_out->FT_DCDC = ((bytes[3] >> 2) & 0x01);
    data_out->FT_FAN1 = ((bytes[3] >> 3) & 0x01);
    data_out->FT_FAN2 = ((bytes[3] >> 4) & 0x01);
    data_out->FT_FAN3 = ((bytes[3] >> 5) & 0x01);
    data_out->FT_HV_OV = ((bytes[3] >> 6) & 0x01);
    data_out->FT_HV_UV = ((bytes[3] >> 7) & 0x01);
    data_out->FT_GLV_UV = (bytes[4] & 0x01);
    data_out->FT_GLV_OV = ((bytes[4] >> 1) & 0x01);
    data_out->FT_AMS = ((bytes[4] >> 2) & 0x01);
    data_out->FT_ANY = ((bytes[4] >> 3) & 0x01);
    data_out->WARN_TEMP_Cell = (bytes[5] & 0x01);
    data_out->WARN_TEMP_DCDC = ((bytes[5] >> 1) & 0x01);
    data_out->WARN_TEMP_Bal = ((bytes[5] >> 2) & 0x01);
    data_out->PWR_ECUB = ((bytes[5] >> 6) & 0x01);
    data_out->FANS_EN = ((bytes[5] >> 7) & 0x01);
    data_out->SEQ = ((bytes[6] >> 4) & 0x0F);
    return 1;
}

int ECUA_decode_Status(const uint8_t* bytes, size_t length, uint8_t* SDC_IN_Open_out, uint8_t* SDC_HV_ILOCK_out, uint8_t* SDC_IMD_out, uint8_t* SDC_AMS_out, uint8_t* SDC_OUT_out, uint8_t* SDC_END_out, uint8_t* LATCH_SDC_AMS_out, uint8_t* AIRsState_out, uint8_t* ChargingState_out, enum ECUA_StateAMS* AMSState_out, uint8_t* FT_ACP_OT_out, uint8_t* FT_AIRS_out, uint8_t* FT_DCDC_out, uint8_t* FT_FAN1_out, uint8_t* FT_FAN2_out, uint8_t* FT_FAN3_out, uint8_t* FT_HV_OV_out, uint8_t* FT_HV_UV_out, uint8_t* FT_GLV_UV_out, uint8_t* FT_GLV_OV_out, uint8_t* FT_AMS_out, uint8_t* FT_ANY_out, uint8_t* WARN_TEMP_Cell_out, uint8_t* WARN_TEMP_DCDC_out, uint8_t* WARN_TEMP_Bal_out, uint8_t* PWR_ECUB_out, uint8_t* FANS_EN_out, uint8_t* SEQ_out) {
    if (length < 7)
        return 0;

    *SDC_IN_Open_out = (bytes[0] & 0x01);
    *SDC_HV_ILOCK_out = ((bytes[0] >> 1) & 0x01);
    *SDC_IMD_out = ((bytes[0] >> 2) & 0x01);
    *SDC_AMS_out = ((bytes[0] >> 3) & 0x01);
    *SDC_OUT_out = ((bytes[0] >> 4) & 0x01);
    *SDC_END_out = ((bytes[0] >> 5) & 0x01);
    *LATCH_SDC_AMS_out = ((bytes[0] >> 7) & 0x01);
    *AIRsState_out = (bytes[1] & 0x0F);
    *ChargingState_out = ((bytes[1] >> 4) & 0x0F);
    *AMSState_out = (enum ECUA_StateAMS) (bytes[2]);
    *FT_ACP_OT_out = (bytes[3] & 0x01);
    *FT_AIRS_out = ((bytes[3] >> 1) & 0x01);
    *FT_DCDC_out = ((bytes[3] >> 2) & 0x01);
    *FT_FAN1_out = ((bytes[3] >> 3) & 0x01);
    *FT_FAN2_out = ((bytes[3] >> 4) & 0x01);
    *FT_FAN3_out = ((bytes[3] >> 5) & 0x01);
    *FT_HV_OV_out = ((bytes[3] >> 6) & 0x01);
    *FT_HV_UV_out = ((bytes[3] >> 7) & 0x01);
    *FT_GLV_UV_out = (bytes[4] & 0x01);
    *FT_GLV_OV_out = ((bytes[4] >> 1) & 0x01);
    *FT_AMS_out = ((bytes[4] >> 2) & 0x01);
    *FT_ANY_out = ((bytes[4] >> 3) & 0x01);
    *WARN_TEMP_Cell_out = (bytes[5] & 0x01);
    *WARN_TEMP_DCDC_out = ((bytes[5] >> 1) & 0x01);
    *WARN_TEMP_Bal_out = ((bytes[5] >> 2) & 0x01);
    *PWR_ECUB_out = ((bytes[5] >> 6) & 0x01);
    *FANS_EN_out = ((bytes[5] >> 7) & 0x01);
    *SEQ_out = ((bytes[6] >> 4) & 0x0F);
    return 1;
}

int ECUA_get_Status(ECUA_Status_t* data_out) {
    if (!(ECUA_Status_status.flags & CAN_MSG_RECEIVED))
        return 0;

#ifndef CANDB_IGNORE_TIMEOUTS
    if (txGetTimeMillis() > ECUA_Status_status.timestamp + ECUA_Status_timeout)
        return 0;
#endif

    if (data_out)
        memcpy(data_out, &ECUA_Status_data, sizeof(ECUA_Status_t));

    int flags = ECUA_Status_status.flags;
    ECUA_Status_status.flags &= ~CAN_MSG_PENDING;
    return flags;
}

void ECUA_Status_on_receive(int (*callback)(ECUA_Status_t* data)) {
    ECUA_Status_status.on_receive = (void (*)(void)) callback;
}

int ECUA_decode_Estimation_s(const uint8_t* bytes, size_t length, ECUA_Estimation_t* data_out) {
    if (length < 5)
        return 0;

    data_out->Charge_OUT = bytes[0] | bytes[1] << 8;
    data_out->Charge_IN = bytes[2] | bytes[3] << 8;
    data_out->SOC = bytes[4];
    return 1;
}

int ECUA_decode_Estimation(const uint8_t* bytes, size_t length, uint16_t* Charge_OUT_out, uint16_t* Charge_IN_out, uint8_t* SOC_out) {
    if (length < 5)
        return 0;

    *Charge_OUT_out = bytes[0] | bytes[1] << 8;
    *Charge_IN_out = bytes[2] | bytes[3] << 8;
    *SOC_out = bytes[4];
    return 1;
}

int ECUA_get_Estimation(ECUA_Estimation_t* data_out) {
    if (!(ECUA_Estimation_status.flags & CAN_MSG_RECEIVED))
        return 0;

#ifndef CANDB_IGNORE_TIMEOUTS
    if (txGetTimeMillis() > ECUA_Estimation_status.timestamp + ECUA_Estimation_timeout)
        return 0;
#endif

    if (data_out)
        memcpy(data_out, &ECUA_Estimation_data, sizeof(ECUA_Estimation_t));

    int flags = ECUA_Estimation_status.flags;
    ECUA_Estimation_status.flags &= ~CAN_MSG_PENDING;
    return flags;
}

void ECUA_Estimation_on_receive(int (*callback)(ECUA_Estimation_t* data)) {
    ECUA_Estimation_status.on_receive = (void (*)(void)) callback;
}

int ECUB_decode_Status_s(const uint8_t* bytes, size_t length, ECUB_Status_t* data_out) {
    if (length < 8)
        return 0;

    data_out->SDC_FRONT = (bytes[0] & 0x01);
    data_out->SDC_SDBL = ((bytes[0] >> 1) & 0x01);
    data_out->SDC_SDBR = ((bytes[0] >> 2) & 0x01);
    data_out->SDC_HVD = ((bytes[0] >> 3) & 0x01);
    data_out->SDC_BSPD = ((bytes[0] >> 4) & 0x01);
    data_out->SDC_MCUR = ((bytes[0] >> 5) & 0x01);
    data_out->SDC_AMS = ((bytes[0] >> 6) & 0x01);
    data_out->SDC_TSMS = ((bytes[0] >> 7) & 0x01);
    data_out->CarState = (enum ECUB_CarState) ((bytes[1] & 0x0F));
    data_out->CarState_Notready = (enum ECUB_Notready_reason) (((bytes[1] >> 4) & 0x0F));
    data_out->PowerSource = (enum ECUB_GLV_PowerSource) ((bytes[2] & 0x03));
    data_out->Det_MOD1 = ((bytes[2] >> 4) & 0x01);
    data_out->Det_MOD2 = ((bytes[2] >> 5) & 0x01);
    data_out->Det_MOD3 = ((bytes[2] >> 6) & 0x01);
    data_out->Det_MOD4 = ((bytes[2] >> 7) & 0x01);
    data_out->FT_PWR1_OT = (bytes[3] & 0x01);
    data_out->FT_PWR2_OT = ((bytes[3] >> 1) & 0x01);
    data_out->FT_PWR3_OT = ((bytes[3] >> 2) & 0x01);
    data_out->FT_PWR4_OT = ((bytes[3] >> 3) & 0x01);
    data_out->FT_PWR5_OT = ((bytes[3] >> 4) & 0x01);
    data_out->FT_L2_OT = ((bytes[3] >> 5) & 0x01);
    data_out->FT_ANY = ((bytes[3] >> 6) & 0x01);
    data_out->FT_L1_OT = ((bytes[3] >> 7) & 0x01);
    data_out->FT_PWR_ECUF_OC = (bytes[4] & 0x01);
    data_out->FT_PWR_ECUA_OC = ((bytes[4] >> 1) & 0x01);
    data_out->FT_PWR_MCF_OC = ((bytes[4] >> 2) & 0x01);
    data_out->FT_PWR_MCR_OC = ((bytes[4] >> 3) & 0x01);
    data_out->FT_CAN1 = ((bytes[4] >> 4) & 0x01);
    data_out->FT_CAN2 = ((bytes[4] >> 5) & 0x01);
    data_out->PWR_ECUF_EN = (bytes[5] & 0x01);
    data_out->PWR_ECUA_EN = ((bytes[5] >> 1) & 0x01);
    data_out->PWR_MCUF_EN = ((bytes[5] >> 2) & 0x01);
    data_out->PWR_MCUR_EN = ((bytes[5] >> 3) & 0x01);
    data_out->PWR_EM_EN = ((bytes[5] >> 4) & 0x01);
    data_out->PWR_WP1_EN = ((bytes[5] >> 5) & 0x01);
    data_out->PWR_WP2_EN = ((bytes[5] >> 6) & 0x01);
    data_out->PWR_FAN1_EN = ((bytes[5] >> 7) & 0x01);
    data_out->PWR_FAN2_EN = (bytes[6] & 0x01);
    data_out->PWR_FAN3_EN = ((bytes[6] >> 1) & 0x01);
    data_out->PWR_WS_EN = ((bytes[6] >> 2) & 0x01);
    data_out->PWR_AUX1_EN = ((bytes[6] >> 3) & 0x01);
    data_out->PWR_AUX2_EN = ((bytes[6] >> 4) & 0x01);
    data_out->PWR_AUX3_EN = ((bytes[6] >> 5) & 0x01);
    data_out->RTDS_EN = ((bytes[6] >> 6) & 0x01);
    data_out->SDBR_LED_EN = ((bytes[6] >> 7) & 0x01);
    data_out->SDBL_LED_EN = (bytes[7] & 0x01);
    data_out->BrakeLight_EN = ((bytes[7] >> 1) & 0x01);
    data_out->TSAL_Override = ((bytes[7] >> 2) & 0x01);
    data_out->SEQ = ((bytes[7] >> 4) & 0x0F);
    return 1;
}

int ECUB_decode_Status(const uint8_t* bytes, size_t length, uint8_t* SDC_FRONT_out, uint8_t* SDC_SDBL_out, uint8_t* SDC_SDBR_out, uint8_t* SDC_HVD_out, uint8_t* SDC_BSPD_out, uint8_t* SDC_MCUR_out, uint8_t* SDC_AMS_out, uint8_t* SDC_TSMS_out, enum ECUB_CarState* CarState_out, enum ECUB_Notready_reason* CarState_Notready_out, enum ECUB_GLV_PowerSource* PowerSource_out, uint8_t* Det_MOD1_out, uint8_t* Det_MOD2_out, uint8_t* Det_MOD3_out, uint8_t* Det_MOD4_out, uint8_t* FT_PWR1_OT_out, uint8_t* FT_PWR2_OT_out, uint8_t* FT_PWR3_OT_out, uint8_t* FT_PWR4_OT_out, uint8_t* FT_PWR5_OT_out, uint8_t* FT_L2_OT_out, uint8_t* FT_ANY_out, uint8_t* FT_L1_OT_out, uint8_t* FT_PWR_ECUF_OC_out, uint8_t* FT_PWR_ECUA_OC_out, uint8_t* FT_PWR_MCF_OC_out, uint8_t* FT_PWR_MCR_OC_out, uint8_t* FT_CAN1_out, uint8_t* FT_CAN2_out, uint8_t* PWR_ECUF_EN_out, uint8_t* PWR_ECUA_EN_out, uint8_t* PWR_MCUF_EN_out, uint8_t* PWR_MCUR_EN_out, uint8_t* PWR_EM_EN_out, uint8_t* PWR_WP1_EN_out, uint8_t* PWR_WP2_EN_out, uint8_t* PWR_FAN1_EN_out, uint8_t* PWR_FAN2_EN_out, uint8_t* PWR_FAN3_EN_out, uint8_t* PWR_WS_EN_out, uint8_t* PWR_AUX1_EN_out, uint8_t* PWR_AUX2_EN_out, uint8_t* PWR_AUX3_EN_out, uint8_t* RTDS_EN_out, uint8_t* SDBR_LED_EN_out, uint8_t* SDBL_LED_EN_out, uint8_t* BrakeLight_EN_out, uint8_t* TSAL_Override_out, uint8_t* SEQ_out) {
    if (length < 8)
        return 0;

    *SDC_FRONT_out = (bytes[0] & 0x01);
    *SDC_SDBL_out = ((bytes[0] >> 1) & 0x01);
    *SDC_SDBR_out = ((bytes[0] >> 2) & 0x01);
    *SDC_HVD_out = ((bytes[0] >> 3) & 0x01);
    *SDC_BSPD_out = ((bytes[0] >> 4) & 0x01);
    *SDC_MCUR_out = ((bytes[0] >> 5) & 0x01);
    *SDC_AMS_out = ((bytes[0] >> 6) & 0x01);
    *SDC_TSMS_out = ((bytes[0] >> 7) & 0x01);
    *CarState_out = (enum ECUB_CarState) ((bytes[1] & 0x0F));
    *CarState_Notready_out = (enum ECUB_Notready_reason) (((bytes[1] >> 4) & 0x0F));
    *PowerSource_out = (enum ECUB_GLV_PowerSource) ((bytes[2] & 0x03));
    *Det_MOD1_out = ((bytes[2] >> 4) & 0x01);
    *Det_MOD2_out = ((bytes[2] >> 5) & 0x01);
    *Det_MOD3_out = ((bytes[2] >> 6) & 0x01);
    *Det_MOD4_out = ((bytes[2] >> 7) & 0x01);
    *FT_PWR1_OT_out = (bytes[3] & 0x01);
    *FT_PWR2_OT_out = ((bytes[3] >> 1) & 0x01);
    *FT_PWR3_OT_out = ((bytes[3] >> 2) & 0x01);
    *FT_PWR4_OT_out = ((bytes[3] >> 3) & 0x01);
    *FT_PWR5_OT_out = ((bytes[3] >> 4) & 0x01);
    *FT_L2_OT_out = ((bytes[3] >> 5) & 0x01);
    *FT_ANY_out = ((bytes[3] >> 6) & 0x01);
    *FT_L1_OT_out = ((bytes[3] >> 7) & 0x01);
    *FT_PWR_ECUF_OC_out = (bytes[4] & 0x01);
    *FT_PWR_ECUA_OC_out = ((bytes[4] >> 1) & 0x01);
    *FT_PWR_MCF_OC_out = ((bytes[4] >> 2) & 0x01);
    *FT_PWR_MCR_OC_out = ((bytes[4] >> 3) & 0x01);
    *FT_CAN1_out = ((bytes[4] >> 4) & 0x01);
    *FT_CAN2_out = ((bytes[4] >> 5) & 0x01);
    *PWR_ECUF_EN_out = (bytes[5] & 0x01);
    *PWR_ECUA_EN_out = ((bytes[5] >> 1) & 0x01);
    *PWR_MCUF_EN_out = ((bytes[5] >> 2) & 0x01);
    *PWR_MCUR_EN_out = ((bytes[5] >> 3) & 0x01);
    *PWR_EM_EN_out = ((bytes[5] >> 4) & 0x01);
    *PWR_WP1_EN_out = ((bytes[5] >> 5) & 0x01);
    *PWR_WP2_EN_out = ((bytes[5] >> 6) & 0x01);
    *PWR_FAN1_EN_out = ((bytes[5] >> 7) & 0x01);
    *PWR_FAN2_EN_out = (bytes[6] & 0x01);
    *PWR_FAN3_EN_out = ((bytes[6] >> 1) & 0x01);
    *PWR_WS_EN_out = ((bytes[6] >> 2) & 0x01);
    *PWR_AUX1_EN_out = ((bytes[6] >> 3) & 0x01);
    *PWR_AUX2_EN_out = ((bytes[6] >> 4) & 0x01);
    *PWR_AUX3_EN_out = ((bytes[6] >> 5) & 0x01);
    *RTDS_EN_out = ((bytes[6] >> 6) & 0x01);
    *SDBR_LED_EN_out = ((bytes[6] >> 7) & 0x01);
    *SDBL_LED_EN_out = (bytes[7] & 0x01);
    *BrakeLight_EN_out = ((bytes[7] >> 1) & 0x01);
    *TSAL_Override_out = ((bytes[7] >> 2) & 0x01);
    *SEQ_out = ((bytes[7] >> 4) & 0x0F);
    return 1;
}

int ECUB_get_Status(ECUB_Status_t* data_out) {
    if (!(ECUB_Status_status.flags & CAN_MSG_RECEIVED))
        return 0;

#ifndef CANDB_IGNORE_TIMEOUTS
    if (txGetTimeMillis() > ECUB_Status_status.timestamp + ECUB_Status_timeout)
        return 0;
#endif

    if (data_out)
        memcpy(data_out, &ECUB_Status_data, sizeof(ECUB_Status_t));

    int flags = ECUB_Status_status.flags;
    ECUB_Status_status.flags &= ~CAN_MSG_PENDING;
    return flags;
}

void ECUB_Status_on_receive(int (*callback)(ECUB_Status_t* data)) {
    ECUB_Status_status.on_receive = (void (*)(void)) callback;
}

int ECUB_decode_Cooling_s(const uint8_t* bytes, size_t length, ECUB_Cooling_t* data_out) {
    if (length < 7)
        return 0;

    data_out->WP1 = (bytes[0] & 0x0F);
    data_out->WP2 = ((bytes[0] >> 4) & 0x0F);
    data_out->FAN1 = (bytes[1] & 0x0F);
    data_out->FAN2 = ((bytes[1] >> 4) & 0x0F);
    data_out->FAN3 = (bytes[2] & 0x0F);
    data_out->WARN_MOT_FR_TEMP = (bytes[3] & 0x01);
    data_out->WARN_MOT_FL_TEMP = ((bytes[3] >> 1) & 0x01);
    data_out->WARN_MOT_RR_TEMP = ((bytes[3] >> 2) & 0x01);
    data_out->WARN_MOT_RL_TEMP = ((bytes[3] >> 3) & 0x01);
    data_out->WARN_MCU_FR_TEMP = ((bytes[3] >> 4) & 0x01);
    data_out->WARN_MCU_FL_TEMP = ((bytes[3] >> 5) & 0x01);
    data_out->WARN_MCU_RR_TEMP = ((bytes[3] >> 6) & 0x01);
    data_out->WARN_MCU_RL_TEMP = ((bytes[3] >> 7) & 0x01);
    data_out->WARN_Brake_RR_TEMP = (bytes[4] & 0x01);
    data_out->WARN_Brake_RL_TEMP = ((bytes[4] >> 1) & 0x01);
    data_out->FT_MOT_FR_OT = (bytes[5] & 0x01);
    data_out->FT_MOT_FL_OT = ((bytes[5] >> 1) & 0x01);
    data_out->FT_MOT_RR_OT = ((bytes[5] >> 2) & 0x01);
    data_out->FT_MOT_RL_OT = ((bytes[5] >> 3) & 0x01);
    data_out->FT_MCU_FR_OT = ((bytes[5] >> 4) & 0x01);
    data_out->FT_MCU_FL_OT = ((bytes[5] >> 5) & 0x01);
    data_out->FT_MCU_RR_OT = ((bytes[5] >> 6) & 0x01);
    data_out->FT_MCU_RL_OT = ((bytes[5] >> 7) & 0x01);
    data_out->FT_Brake_RR_OT = (bytes[6] & 0x01);
    data_out->FT_Brake_RL_OT = ((bytes[6] >> 1) & 0x01);
    return 1;
}

int ECUB_decode_Cooling(const uint8_t* bytes, size_t length, uint8_t* WP1_out, uint8_t* WP2_out, uint8_t* FAN1_out, uint8_t* FAN2_out, uint8_t* FAN3_out, uint8_t* WARN_MOT_FR_TEMP_out, uint8_t* WARN_MOT_FL_TEMP_out, uint8_t* WARN_MOT_RR_TEMP_out, uint8_t* WARN_MOT_RL_TEMP_out, uint8_t* WARN_MCU_FR_TEMP_out, uint8_t* WARN_MCU_FL_TEMP_out, uint8_t* WARN_MCU_RR_TEMP_out, uint8_t* WARN_MCU_RL_TEMP_out, uint8_t* WARN_Brake_RR_TEMP_out, uint8_t* WARN_Brake_RL_TEMP_out, uint8_t* FT_MOT_FR_OT_out, uint8_t* FT_MOT_FL_OT_out, uint8_t* FT_MOT_RR_OT_out, uint8_t* FT_MOT_RL_OT_out, uint8_t* FT_MCU_FR_OT_out, uint8_t* FT_MCU_FL_OT_out, uint8_t* FT_MCU_RR_OT_out, uint8_t* FT_MCU_RL_OT_out, uint8_t* FT_Brake_RR_OT_out, uint8_t* FT_Brake_RL_OT_out) {
    if (length < 7)
        return 0;

    *WP1_out = (bytes[0] & 0x0F);
    *WP2_out = ((bytes[0] >> 4) & 0x0F);
    *FAN1_out = (bytes[1] & 0x0F);
    *FAN2_out = ((bytes[1] >> 4) & 0x0F);
    *FAN3_out = (bytes[2] & 0x0F);
    *WARN_MOT_FR_TEMP_out = (bytes[3] & 0x01);
    *WARN_MOT_FL_TEMP_out = ((bytes[3] >> 1) & 0x01);
    *WARN_MOT_RR_TEMP_out = ((bytes[3] >> 2) & 0x01);
    *WARN_MOT_RL_TEMP_out = ((bytes[3] >> 3) & 0x01);
    *WARN_MCU_FR_TEMP_out = ((bytes[3] >> 4) & 0x01);
    *WARN_MCU_FL_TEMP_out = ((bytes[3] >> 5) & 0x01);
    *WARN_MCU_RR_TEMP_out = ((bytes[3] >> 6) & 0x01);
    *WARN_MCU_RL_TEMP_out = ((bytes[3] >> 7) & 0x01);
    *WARN_Brake_RR_TEMP_out = (bytes[4] & 0x01);
    *WARN_Brake_RL_TEMP_out = ((bytes[4] >> 1) & 0x01);
    *FT_MOT_FR_OT_out = (bytes[5] & 0x01);
    *FT_MOT_FL_OT_out = ((bytes[5] >> 1) & 0x01);
    *FT_MOT_RR_OT_out = ((bytes[5] >> 2) & 0x01);
    *FT_MOT_RL_OT_out = ((bytes[5] >> 3) & 0x01);
    *FT_MCU_FR_OT_out = ((bytes[5] >> 4) & 0x01);
    *FT_MCU_FL_OT_out = ((bytes[5] >> 5) & 0x01);
    *FT_MCU_RR_OT_out = ((bytes[5] >> 6) & 0x01);
    *FT_MCU_RL_OT_out = ((bytes[5] >> 7) & 0x01);
    *FT_Brake_RR_OT_out = (bytes[6] & 0x01);
    *FT_Brake_RL_OT_out = ((bytes[6] >> 1) & 0x01);
    return 1;
}

int ECUB_get_Cooling(ECUB_Cooling_t* data_out) {
    if (!(ECUB_Cooling_status.flags & CAN_MSG_RECEIVED))
        return 0;

    if (data_out)
        memcpy(data_out, &ECUB_Cooling_data, sizeof(ECUB_Cooling_t));

    int flags = ECUB_Cooling_status.flags;
    ECUB_Cooling_status.flags &= ~CAN_MSG_PENDING;
    return flags;
}

void ECUB_Cooling_on_receive(int (*callback)(ECUB_Cooling_t* data)) {
    ECUB_Cooling_status.on_receive = (void (*)(void)) callback;
}

int ECUF_send_Status_s(const ECUF_Status_t* data) {
    uint8_t buffer[6];
    buffer[0] = (data->SDC_SDBC ? 1 : 0) | (data->SDC_Inertia ? 2 : 0) | (data->SDC_FWIL ? 4 : 0);
    buffer[1] = (data->PWR_ECUP_EN ? 1 : 0) | (data->PWR_ECUG_EN ? 2 : 0) | (data->PWR_DTLG_EN ? 4 : 0) | (data->PWR_ECUS_EN ? 8 : 0) | (data->PWR_DASH_EN ? 16 : 0) | (data->PWR_FAN_BrakeF_EN ? 32 : 0) | (data->WARN_Brake_FR_TEMP ? 64 : 0) | (data->WARN_Brake_FL_TEMP ? 128 : 0);
    buffer[2] = (data->FT_PWR_ECUP ? 1 : 0) | (data->FT_PWR_ECUG ? 2 : 0) | (data->FT_PWR_ECUS ? 4 : 0) | (data->FT_PWR_DTLG ? 8 : 0) | (data->FT_PWR_DASH ? 16 : 0) | (data->FT_PWR_FAN_BrakeF ? 32 : 0) | (data->FT_STW_Sensor ? 64 : 0) | (data->FT_STW_Cal ? 128 : 0);
    buffer[3] = (data->FT_DisFR ? 1 : 0) | (data->FT_DisFL ? 2 : 0) | (data->FT_DisRR ? 4 : 0) | (data->FT_DisRL ? 8 : 0) | (data->FT_DisFR_Cal ? 16 : 0) | (data->FT_DisFL_Cal ? 32 : 0) | (data->FT_DisRR_Cal ? 64 : 0) | (data->FT_DisRL_Cal ? 128 : 0);
    buffer[4] = (data->FT_Brake_FR_OT ? 1 : 0) | (data->FT_Brake_FL_OT ? 2 : 0);
    buffer[5] = data->Volt_GLV_In;
    ECUF_Status_last_sent = txGetTimeMillis();
    return txSendCANMessage(bus_CAN1_powertrain, ECUF_Status_id, buffer, sizeof(buffer));
}

int ECUF_send_Status(uint8_t SDC_SDBC, uint8_t SDC_Inertia, uint8_t SDC_FWIL, uint8_t PWR_ECUP_EN, uint8_t PWR_ECUG_EN, uint8_t PWR_DTLG_EN, uint8_t PWR_ECUS_EN, uint8_t PWR_DASH_EN, uint8_t PWR_FAN_BrakeF_EN, uint8_t WARN_Brake_FR_TEMP, uint8_t WARN_Brake_FL_TEMP, uint8_t FT_PWR_ECUP, uint8_t FT_PWR_ECUG, uint8_t FT_PWR_ECUS, uint8_t FT_PWR_DTLG, uint8_t FT_PWR_DASH, uint8_t FT_PWR_FAN_BrakeF, uint8_t FT_STW_Sensor, uint8_t FT_STW_Cal, uint8_t FT_DisFR, uint8_t FT_DisFL, uint8_t FT_DisRR, uint8_t FT_DisRL, uint8_t FT_DisFR_Cal, uint8_t FT_DisFL_Cal, uint8_t FT_DisRR_Cal, uint8_t FT_DisRL_Cal, uint8_t FT_Brake_FR_OT, uint8_t FT_Brake_FL_OT, uint8_t Volt_GLV_In) {
    uint8_t buffer[6];
    buffer[0] = (SDC_SDBC ? 1 : 0) | (SDC_Inertia ? 2 : 0) | (SDC_FWIL ? 4 : 0);
    buffer[1] = (PWR_ECUP_EN ? 1 : 0) | (PWR_ECUG_EN ? 2 : 0) | (PWR_DTLG_EN ? 4 : 0) | (PWR_ECUS_EN ? 8 : 0) | (PWR_DASH_EN ? 16 : 0) | (PWR_FAN_BrakeF_EN ? 32 : 0) | (WARN_Brake_FR_TEMP ? 64 : 0) | (WARN_Brake_FL_TEMP ? 128 : 0);
    buffer[2] = (FT_PWR_ECUP ? 1 : 0) | (FT_PWR_ECUG ? 2 : 0) | (FT_PWR_ECUS ? 4 : 0) | (FT_PWR_DTLG ? 8 : 0) | (FT_PWR_DASH ? 16 : 0) | (FT_PWR_FAN_BrakeF ? 32 : 0) | (FT_STW_Sensor ? 64 : 0) | (FT_STW_Cal ? 128 : 0);
    buffer[3] = (FT_DisFR ? 1 : 0) | (FT_DisFL ? 2 : 0) | (FT_DisRR ? 4 : 0) | (FT_DisRL ? 8 : 0) | (FT_DisFR_Cal ? 16 : 0) | (FT_DisFL_Cal ? 32 : 0) | (FT_DisRR_Cal ? 64 : 0) | (FT_DisRL_Cal ? 128 : 0);
    buffer[4] = (FT_Brake_FR_OT ? 1 : 0) | (FT_Brake_FL_OT ? 2 : 0);
    buffer[5] = Volt_GLV_In;
    ECUF_Status_last_sent = txGetTimeMillis();
    return txSendCANMessage(bus_CAN1_powertrain, ECUF_Status_id, buffer, sizeof(buffer));
}

int ECUF_Status_need_to_send(void) {
    return (ECUF_Status_last_sent == -1) || (txGetTimeMillis() >= ECUF_Status_last_sent + 200);
}

int ECUF_send_STW_s(const ECUF_STW_t* data) {
    uint8_t buffer[6];
    buffer[0] = data->Angle;
    buffer[1] = (data->Angle >> 8);
    buffer[2] = data->AngularSpeed;
    buffer[3] = (data->AngularSpeed >> 8);
    buffer[4] = (data->FT_STW ? 1 : 0);
    buffer[5] = ((data->SEQ & 0x0F) << 4);
    ECUF_STW_last_sent = txGetTimeMillis();
    return txSendCANMessage(bus_CAN1_powertrain, ECUF_STW_id, buffer, sizeof(buffer));
}

int ECUF_send_STW(int16_t Angle, int16_t AngularSpeed, uint8_t FT_STW, uint8_t SEQ) {
    uint8_t buffer[6];
    buffer[0] = Angle;
    buffer[1] = (Angle >> 8);
    buffer[2] = AngularSpeed;
    buffer[3] = (AngularSpeed >> 8);
    buffer[4] = (FT_STW ? 1 : 0);
    buffer[5] = ((SEQ & 0x0F) << 4);
    ECUF_STW_last_sent = txGetTimeMillis();
    return txSendCANMessage(bus_CAN1_powertrain, ECUF_STW_id, buffer, sizeof(buffer));
}

int ECUF_STW_need_to_send(void) {
    return (ECUF_STW_last_sent == -1) || (txGetTimeMillis() >= ECUF_STW_last_sent + 10);
}

int ECUF_send_DISSusp_s(const ECUF_DISSusp_t* data) {
    uint8_t buffer[8];
    buffer[0] = data->FR;
    buffer[1] = (data->FR >> 8);
    buffer[2] = data->FL;
    buffer[3] = (data->FL >> 8);
    buffer[4] = data->RR;
    buffer[5] = (data->RR >> 8);
    buffer[6] = data->RL;
    buffer[7] = (data->RL >> 8);
    ECUF_DISSusp_last_sent = txGetTimeMillis();
    return txSendCANMessage(bus_CAN1_powertrain, ECUF_DISSusp_id, buffer, sizeof(buffer));
}

int ECUF_send_DISSusp(uint16_t FR, uint16_t FL, uint16_t RR, uint16_t RL) {
    uint8_t buffer[8];
    buffer[0] = FR;
    buffer[1] = (FR >> 8);
    buffer[2] = FL;
    buffer[3] = (FL >> 8);
    buffer[4] = RR;
    buffer[5] = (RR >> 8);
    buffer[6] = RL;
    buffer[7] = (RL >> 8);
    ECUF_DISSusp_last_sent = txGetTimeMillis();
    return txSendCANMessage(bus_CAN1_powertrain, ECUF_DISSusp_id, buffer, sizeof(buffer));
}

int ECUF_DISSusp_need_to_send(void) {
    return (ECUF_DISSusp_last_sent == -1) || (txGetTimeMillis() >= ECUF_DISSusp_last_sent + 5);
}

int ECUF_send_Dashboard_s(const ECUF_Dashboard_t* data) {
    uint8_t buffer[3];
    buffer[0] = (data->TSON ? 1 : 0) | (data->START ? 2 : 0) | (data->WP_ON ? 4 : 0) | (data->TCS_ON ? 8 : 0) | (data->YC_ON ? 16 : 0);
    buffer[1] = data->AmbientLight;
    buffer[2] = data->AmbientTemp;
    ECUF_Dashboard_last_sent = txGetTimeMillis();
    return txSendCANMessage(bus_CAN1_powertrain, ECUF_Dashboard_id, buffer, sizeof(buffer));
}

int ECUF_send_Dashboard(uint8_t TSON, uint8_t START, uint8_t WP_ON, uint8_t TCS_ON, uint8_t YC_ON, uint8_t AmbientLight, uint8_t AmbientTemp) {
    uint8_t buffer[3];
    buffer[0] = (TSON ? 1 : 0) | (START ? 2 : 0) | (WP_ON ? 4 : 0) | (TCS_ON ? 8 : 0) | (YC_ON ? 16 : 0);
    buffer[1] = AmbientLight;
    buffer[2] = AmbientTemp;
    ECUF_Dashboard_last_sent = txGetTimeMillis();
    return txSendCANMessage(bus_CAN1_powertrain, ECUF_Dashboard_id, buffer, sizeof(buffer));
}

int ECUF_Dashboard_need_to_send(void) {
    return (ECUF_Dashboard_last_sent == -1) || (txGetTimeMillis() >= ECUF_Dashboard_last_sent + 100);
}

int ECUF_send_TEMPSuspF_s(const ECUF_TEMPSuspF_t* data) {
    uint8_t buffer[8];
    buffer[0] = data->BrakeCal_FR;
    buffer[1] = data->BrakeCal_FL;
    buffer[2] = data->TireI_FR;
    buffer[3] = data->TireC_FR;
    buffer[4] = data->TireO_FR;
    buffer[5] = data->TireI_FL;
    buffer[6] = data->TireC_FL;
    buffer[7] = data->TireO_FL;
    ECUF_TEMPSuspF_last_sent = txGetTimeMillis();
    return txSendCANMessage(bus_CAN2_aux, ECUF_TEMPSuspF_id, buffer, sizeof(buffer));
}

int ECUF_send_TEMPSuspF(uint8_t BrakeCal_FR, uint8_t BrakeCal_FL, uint8_t TireI_FR, uint8_t TireC_FR, uint8_t TireO_FR, uint8_t TireI_FL, uint8_t TireC_FL, uint8_t TireO_FL) {
    uint8_t buffer[8];
    buffer[0] = BrakeCal_FR;
    buffer[1] = BrakeCal_FL;
    buffer[2] = TireI_FR;
    buffer[3] = TireC_FR;
    buffer[4] = TireO_FR;
    buffer[5] = TireI_FL;
    buffer[6] = TireC_FL;
    buffer[7] = TireO_FL;
    ECUF_TEMPSuspF_last_sent = txGetTimeMillis();
    return txSendCANMessage(bus_CAN2_aux, ECUF_TEMPSuspF_id, buffer, sizeof(buffer));
}

int ECUF_TEMPSuspF_need_to_send(void) {
    return (ECUF_TEMPSuspF_last_sent == -1) || (txGetTimeMillis() >= ECUF_TEMPSuspF_last_sent + 50);
}

int ECUF_decode_REQCalibSTW_s(const uint8_t* bytes, size_t length, ECUF_REQCalibSTW_t* data_out) {
    if (length < 1)
        return 0;

    data_out->which = (enum ECUF_CAL_STWIndex) ((bytes[0] & 0x0F));
    return 1;
}

int ECUF_decode_REQCalibSTW(const uint8_t* bytes, size_t length, enum ECUF_CAL_STWIndex* which_out) {
    if (length < 1)
        return 0;

    *which_out = (enum ECUF_CAL_STWIndex) ((bytes[0] & 0x0F));
    return 1;
}

int ECUF_get_REQCalibSTW(ECUF_REQCalibSTW_t* data_out) {
    if (!(ECUF_REQCalibSTW_status.flags & CAN_MSG_RECEIVED))
        return 0;

    if (data_out)
        memcpy(data_out, &ECUF_REQCalibSTW_data, sizeof(ECUF_REQCalibSTW_t));

    int flags = ECUF_REQCalibSTW_status.flags;
    ECUF_REQCalibSTW_status.flags &= ~CAN_MSG_PENDING;
    return flags;
}

void ECUF_REQCalibSTW_on_receive(int (*callback)(ECUF_REQCalibSTW_t* data)) {
    ECUF_REQCalibSTW_status.on_receive = (void (*)(void)) callback;
}

int ECUP_decode_Status_s(const uint8_t* bytes, size_t length, ECUP_Status_t* data_out) {
    if (length < 2)
        return 0;

    data_out->SDC_BOTS = (bytes[0] & 0x01);
    data_out->FT_ANY = (bytes[1] & 0x01);
    data_out->APPS_Plausible = ((bytes[1] >> 1) & 0x01);
    data_out->BPPC_Latch = ((bytes[1] >> 2) & 0x01);
    data_out->BrakeActive = ((bytes[1] >> 3) & 0x01);
    data_out->BrakeActive_BSPD = ((bytes[1] >> 4) & 0x01);
    return 1;
}

int ECUP_decode_Status(const uint8_t* bytes, size_t length, uint8_t* SDC_BOTS_out, uint8_t* FT_ANY_out, uint8_t* APPS_Plausible_out, uint8_t* BPPC_Latch_out, uint8_t* BrakeActive_out, uint8_t* BrakeActive_BSPD_out) {
    if (length < 2)
        return 0;

    *SDC_BOTS_out = (bytes[0] & 0x01);
    *FT_ANY_out = (bytes[1] & 0x01);
    *APPS_Plausible_out = ((bytes[1] >> 1) & 0x01);
    *BPPC_Latch_out = ((bytes[1] >> 2) & 0x01);
    *BrakeActive_out = ((bytes[1] >> 3) & 0x01);
    *BrakeActive_BSPD_out = ((bytes[1] >> 4) & 0x01);
    return 1;
}

int ECUP_get_Status(ECUP_Status_t* data_out) {
    if (!(ECUP_Status_status.flags & CAN_MSG_RECEIVED))
        return 0;

#ifndef CANDB_IGNORE_TIMEOUTS
    if (txGetTimeMillis() > ECUP_Status_status.timestamp + ECUP_Status_timeout)
        return 0;
#endif

    if (data_out)
        memcpy(data_out, &ECUP_Status_data, sizeof(ECUP_Status_t));

    int flags = ECUP_Status_status.flags;
    ECUP_Status_status.flags &= ~CAN_MSG_PENDING;
    return flags;
}

void ECUP_Status_on_receive(int (*callback)(ECUP_Status_t* data)) {
    ECUP_Status_status.on_receive = (void (*)(void)) callback;
}

int VDCU_decode_Status_s(const uint8_t* bytes, size_t length, VDCU_Status_t* data_out) {
    if (length < 5)
        return 0;

    data_out->State = (enum VDCU_VDCU_State) ((bytes[1] & 0x0F));
    data_out->TV_ENABLED = (bytes[2] & 0x01);
    data_out->TC_ENABLED = ((bytes[2] >> 1) & 0x01);
    data_out->YC_ENABLED = ((bytes[2] >> 2) & 0x01);
    data_out->FT_Dis_Cal = (bytes[3] & 0x01);
    data_out->FT_Sensor = ((bytes[3] >> 1) & 0x01);
    data_out->TEMP_derating = (bytes[4] & 0x01);
    data_out->ACP_derating = ((bytes[4] >> 1) & 0x01);
    data_out->Disch_ACT = ((bytes[4] >> 2) & 0x01);
    data_out->Reverse_ACT = ((bytes[4] >> 3) & 0x01);
    data_out->TC_ACT = ((bytes[4] >> 4) & 0x01);
    data_out->YC_ACT = ((bytes[4] >> 5) & 0x01);
    return 1;
}

int VDCU_decode_Status(const uint8_t* bytes, size_t length, enum VDCU_VDCU_State* State_out, uint8_t* TV_ENABLED_out, uint8_t* TC_ENABLED_out, uint8_t* YC_ENABLED_out, uint8_t* FT_Dis_Cal_out, uint8_t* FT_Sensor_out, uint8_t* TEMP_derating_out, uint8_t* ACP_derating_out, uint8_t* Disch_ACT_out, uint8_t* Reverse_ACT_out, uint8_t* TC_ACT_out, uint8_t* YC_ACT_out) {
    if (length < 5)
        return 0;

    *State_out = (enum VDCU_VDCU_State) ((bytes[1] & 0x0F));
    *TV_ENABLED_out = (bytes[2] & 0x01);
    *TC_ENABLED_out = ((bytes[2] >> 1) & 0x01);
    *YC_ENABLED_out = ((bytes[2] >> 2) & 0x01);
    *FT_Dis_Cal_out = (bytes[3] & 0x01);
    *FT_Sensor_out = ((bytes[3] >> 1) & 0x01);
    *TEMP_derating_out = (bytes[4] & 0x01);
    *ACP_derating_out = ((bytes[4] >> 1) & 0x01);
    *Disch_ACT_out = ((bytes[4] >> 2) & 0x01);
    *Reverse_ACT_out = ((bytes[4] >> 3) & 0x01);
    *TC_ACT_out = ((bytes[4] >> 4) & 0x01);
    *YC_ACT_out = ((bytes[4] >> 5) & 0x01);
    return 1;
}

int VDCU_get_Status(VDCU_Status_t* data_out) {
    if (!(VDCU_Status_status.flags & CAN_MSG_RECEIVED))
        return 0;

#ifndef CANDB_IGNORE_TIMEOUTS
    if (txGetTimeMillis() > VDCU_Status_status.timestamp + VDCU_Status_timeout)
        return 0;
#endif

    if (data_out)
        memcpy(data_out, &VDCU_Status_data, sizeof(VDCU_Status_t));

    int flags = VDCU_Status_status.flags;
    VDCU_Status_status.flags &= ~CAN_MSG_PENDING;
    return flags;
}

void VDCU_Status_on_receive(int (*callback)(VDCU_Status_t* data)) {
    VDCU_Status_status.on_receive = (void (*)(void)) callback;
}

void candbHandleMessage(uint32_t timestamp, int bus, CAN_ID_t id, const uint8_t* payload, size_t payload_length) {
    switch (id) {
    case ECUA_Status_id: {
        if (!ECUA_decode_Status_s(payload, payload_length, &ECUA_Status_data))
            break;

        canUpdateMsgStatusOnReceive(&ECUA_Status_status, timestamp);

        if (ECUA_Status_status.on_receive)
            ((int (*)(ECUA_Status_t*)) ECUA_Status_status.on_receive)(&ECUA_Status_data);

        break;
    }
    case ECUA_Estimation_id: {
        if (!ECUA_decode_Estimation_s(payload, payload_length, &ECUA_Estimation_data))
            break;

        canUpdateMsgStatusOnReceive(&ECUA_Estimation_status, timestamp);

        if (ECUA_Estimation_status.on_receive)
            ((int (*)(ECUA_Estimation_t*)) ECUA_Estimation_status.on_receive)(&ECUA_Estimation_data);

        break;
    }
    case ECUB_Status_id: {
        if (!ECUB_decode_Status_s(payload, payload_length, &ECUB_Status_data))
            break;

        canUpdateMsgStatusOnReceive(&ECUB_Status_status, timestamp);

        if (ECUB_Status_status.on_receive)
            ((int (*)(ECUB_Status_t*)) ECUB_Status_status.on_receive)(&ECUB_Status_data);

        break;
    }
    case ECUB_Cooling_id: {
        if (!ECUB_decode_Cooling_s(payload, payload_length, &ECUB_Cooling_data))
            break;

        canUpdateMsgStatusOnReceive(&ECUB_Cooling_status, timestamp);

        if (ECUB_Cooling_status.on_receive)
            ((int (*)(ECUB_Cooling_t*)) ECUB_Cooling_status.on_receive)(&ECUB_Cooling_data);

        break;
    }
    case ECUF_REQCalibSTW_id: {
        if (!ECUF_decode_REQCalibSTW_s(payload, payload_length, &ECUF_REQCalibSTW_data))
            break;

        canUpdateMsgStatusOnReceive(&ECUF_REQCalibSTW_status, timestamp);

        if (ECUF_REQCalibSTW_status.on_receive)
            ((int (*)(ECUF_REQCalibSTW_t*)) ECUF_REQCalibSTW_status.on_receive)(&ECUF_REQCalibSTW_data);

        break;
    }
    case ECUP_Status_id: {
        if (!ECUP_decode_Status_s(payload, payload_length, &ECUP_Status_data))
            break;

        canUpdateMsgStatusOnReceive(&ECUP_Status_status, timestamp);

        if (ECUP_Status_status.on_receive)
            ((int (*)(ECUP_Status_t*)) ECUP_Status_status.on_receive)(&ECUP_Status_data);

        break;
    }
    case VDCU_Status_id: {
        if (!VDCU_decode_Status_s(payload, payload_length, &VDCU_Status_data))
            break;

        canUpdateMsgStatusOnReceive(&VDCU_Status_status, timestamp);

        if (VDCU_Status_status.on_receive)
            ((int (*)(VDCU_Status_t*)) VDCU_Status_status.on_receive)(&VDCU_Status_data);

        break;
    }
    }
}
