#ifndef CAN_ECUF_H
#define CAN_ECUF_H

#include <tx2/can.h>

#ifdef __cplusplus
extern "C" {
#endif

enum {
    bus_CAN1_powertrain = 0,
    bus_CAN2_aux = 1,
};

enum { ECUA_Status_id = 0x090 };
enum { ECUA_Status_timeout = 500 };
enum { ECUA_Estimation_id = 0x394 };
enum { ECUA_Estimation_timeout = 500 };
enum { ECUB_Status_id = 0x0A0 };
enum { ECUB_Status_timeout = 1000 };
enum { ECUB_Cooling_id = 0x4A4 };
enum { ECUF_Status_id = 0x0C0 };
enum { ECUF_Status_timeout = 1000 };
enum { ECUF_STW_id = 0x1C1 };
enum { ECUF_STW_timeout = 50 };
enum { ECUF_DISSusp_id = 0x3C2 };
enum { ECUF_Dashboard_id = 0x3C5 };
enum { ECUF_TEMPSuspF_id = 0x4C3 };
enum { ECUF_REQCalibSTW_id = 0x4CD };
enum { ECUP_Status_id = 0x040 };
enum { ECUP_Status_timeout = 500 };
enum { VDCU_Status_id = 0x050 };
enum { VDCU_Status_timeout = 500 };

enum ECUA_StateAMS {
    /* without_fault */
    ECUA_StateAMS_All_OK = 0,
    /* All_fucked */
    ECUA_StateAMS_SHIT = 1,
};

enum ECUF_CAL_STWIndex {
    /* None */
    ECUF_CAL_STWIndex_None = 0,
    /* Left position (output is positive) */
    ECUF_CAL_STWIndex_STWLeft = 1,
    /* Center position */
    ECUF_CAL_STWIndex_STWCenter = 2,
    /* Right position (output is negative) */
    ECUF_CAL_STWIndex_STWRight = 3,
};

enum VDCU_CAL_DisIndex {
    /* None */
    VDCU_CAL_DisIndex_None = 0,
    /* Front right displacement sensor calibration */
    VDCU_CAL_DisIndex_DisFR = 1,
    /* Front left displacement sensor calibration */
    VDCU_CAL_DisIndex_DisFL = 2,
    /* Rear right displacement sensor calibration */
    VDCU_CAL_DisIndex_DisRR = 3,
    /* Rear left displacement sensor calibration */
    VDCU_CAL_DisIndex_DisRL = 4,
};

enum VDCU_VDCU_Param {
    /* Torque gain (0-128) */
    VDCU_VDCU_Param_TorqueGain = 0,
    /* Maximum power in kW (0-140) */
    VDCU_VDCU_Param_PowerMax = 1,
    /* Pedal map selector (0-4) */
    VDCU_VDCU_Param_Ped_MAP = 2,
    /* Racing mode (OFF,accel, skid, autox, endu ...) (0-9) */
    VDCU_VDCU_Param_RaceMode = 3,
    /* Maximum regenerative power in kW (0-40) */
    VDCU_VDCU_Param_REGEN_PowerMax = 4,
    /* ENABLE regenerative braking [0,1] */
    VDCU_VDCU_Param_REGEN_EN = 5,
    /* Field weakening (0-100) */
    VDCU_VDCU_Param_FW = 6,
    /* Request for reverse [0,1] */
    VDCU_VDCU_Param_Reverse_REQ = 7,
    /* Request for not limited performance [0,1] */
    VDCU_VDCU_Param_Boost_REQ = 8,
    /* Torque distribution between front/rear (0-100) */
    VDCU_VDCU_Param_Torque_Dist = 12,
    /* Torque vectoring gain on front axle (0-200) */
    VDCU_VDCU_Param_TV_GainF = 13,
    /* Torque vectoring gain on rear axle (0-200) */
    VDCU_VDCU_Param_TV_GainR = 14,
    /* Maximum torque for front axle (0-32) */
    VDCU_VDCU_Param_TorqueMaxF = 15,
    /* Maximum torque for rear axle (0-96) */
    VDCU_VDCU_Param_TorqueMaxR = 16,
    /* Slip ratio controller slip setpoint (4-20) */
    VDCU_VDCU_Param_TC_SpSlip = 20,
    /* Slip ratio controller threshold */
    VDCU_VDCU_Param_TC_Threshold = 21,
    /* Slip ratio controller Kp gain (0-) */
    VDCU_VDCU_Param_TC_Kp = 22,
    /* Slip ratio controller Ki gain (0-) */
    VDCU_VDCU_Param_TC_Ki = 23,
    /* Slip ratio controller Kb gain(0-) */
    VDCU_VDCU_Param_TC_Kb = 24,
    /* Yaw controller overal gain (0-200) */
    VDCU_VDCU_Param_YC_Gain = 30,
    /* Yaw controller SpeedGain (0-1) */
    VDCU_VDCU_Param_YC_SpeedGain = 31,
    /* Yaw controller Kp gain (0-) */
    VDCU_VDCU_Param_YC_Kp = 32,
    /* Yaw controller Ki gain (0-) */
    VDCU_VDCU_Param_YC_Ki = 33,
    /* Yaw controller Kb gain (0-) */
    VDCU_VDCU_Param_YC_Kb = 34,
};

enum VDCU_VDCU_State {
    /* Waiting for RTD */
    VDCU_VDCU_State_STANDBY = 0,
    /* Torque enabled */
    VDCU_VDCU_State_TORQUE_EN = 1,
    /* Error state */
    VDCU_VDCU_State_ERROR = 2,
};

enum ECUB_Batt_code {
    /* No power drawn nor charged */
    ECUB_Batt_code_IDLE = 0,
    /* Charging with balancing */
    ECUB_Batt_code_CHARGING = 1,
    /* Charging without balancing */
    ECUB_Batt_code_FAST_CHARGING = 2,
    /* Only balancing */
    ECUB_Batt_code_BALANCING = 3,
    /* Is being discharged */
    ECUB_Batt_code_DISCHARGING = 4,
    /* Fully charged */
    ECUB_Batt_code_FULL = 5,
    /* Is in error state */
    ECUB_Batt_code_ERROR = 6,
};

enum ECUB_CarState {
    /* SDC interrupted -> not ready for start */
    ECUB_CarState_NOT_READY = 0,
    /* Fatal error \w SDC latching */
    ECUB_CarState_LATCHED = 1,
    /* Ready for TSON button */
    ECUB_CarState_TS_READY = 2,
    /* ACP is being precharged -> waiting for ECUA status */
    ECUB_CarState_PRECHARGE = 3,
    /* Ready for START */
    ECUB_CarState_TS_ON = 4,
    /* Waiting for completion of RTDS */
    ECUB_CarState_WAITING_FOR_RTDS = 5,
    /* Drive! */
    ECUB_CarState_STARTED = 6,
};

enum ECUB_GLV_PowerSource {
    /* ACP */
    ECUB_GLV_PowerSource_ACP = 0,
    /* GLV battery */
    ECUB_GLV_PowerSource_GLV_BATTERY = 1,
    /* Service box input */
    ECUB_GLV_PowerSource_SERVICE_INPUT = 2,
};

enum ECUB_Notready_reason {
    /* No error */
    ECUB_Notready_reason_NONE = 0,
    /* Vehicle was latched at the startup */
    ECUB_Notready_reason_LATCH_START = 1,
    /* Vehicle was latched due to BSPD error */
    ECUB_Notready_reason_LATCH_BSPD = 2,
    /* Vehicle was latched due to AMS error */
    ECUB_Notready_reason_LATCH_AMS = 3,
    /* Error in SDC chain */
    ECUB_Notready_reason_SDC_FAILURE = 4,
    /* Motor controller CAN timeout */
    ECUB_Notready_reason_TIMEOUT_MC = 5,
    /* AMS CAN timeout */
    ECUB_Notready_reason_TIMEOUT_ECUA = 6,
    /* ECU Front CAN timeout */
    ECUB_Notready_reason_TIMEOUT_ECUF = 7,
    /* Pedal unit CAN timeout */
    ECUB_Notready_reason_TIMEOUT_ECUP = 8,
    /* VDCU CAN timeout */
    ECUB_Notready_reason_TIMEOUT_VDCU = 9,
    /* Fault on PWM_fault pins */
    ECUB_Notready_reason_PWM_FAULT = 10,
};

enum ECUP_CAL_PedalIndex {
    /* None */
    ECUP_CAL_PedalIndex_None = 0,
    /* Minimum apps position */
    ECUP_CAL_PedalIndex_AppsMin = 1,
    /* Maximum apps position */
    ECUP_CAL_PedalIndex_AppsMax = 2,
    /* Minimum brake position */
    ECUP_CAL_PedalIndex_BrakeMin = 3,
    /* Maximum brake position */
    ECUP_CAL_PedalIndex_BrakeMax = 4,
    /* Maximum regenerative position */
    ECUP_CAL_PedalIndex_RegenMax = 5,
};

enum ECUP_CalibrationIndex {
    /* FIX THIS SHIT */
    ECUP_CalibrationIndex_dummy = 1,
};

/*
 * Base status of ECUA and its subsystems (AMS)
 */
typedef struct ECUA_Status_t {
	/* Lead from ECUB (1 = open) */
	uint8_t	SDC_IN_Open;

	/* True if HV interlock is closed */
	uint8_t	SDC_HV_ILOCK;

	/* True if SDC is not broken by IMD */
	uint8_t	SDC_IMD;

	/* AMS = ECUA+BMS */
	uint8_t	SDC_AMS;

	/* SDC out to final stretch (BSPD etc...) */
	uint8_t	SDC_OUT;

	/* End of SDC (input to AIRS) */
	uint8_t	SDC_END;

	/* HW latch engaged (caused by IMD or AMS) */
	uint8_t	LATCH_SDC_AMS;

	/*  */
	uint8_t	AIRsState;

	/*  */
	uint8_t	ChargingState;

	/* Error state of AMS */
	enum ECUA_StateAMS	AMSState;

	/* Fault of ACP overtemperature */
	uint8_t	FT_ACP_OT;

	/* Fault of AIRs */
	uint8_t	FT_AIRS;

	/* Fault of DCDC GLV converter */
	uint8_t	FT_DCDC;

	/* FAN1 dead */
	uint8_t	FT_FAN1;

	/* FAN2 dead */
	uint8_t	FT_FAN2;

	/* FAN3 dead */
	uint8_t	FT_FAN3;

	/* Fault HV overvoltage */
	uint8_t	FT_HV_OV;

	/* Fault HV undervoltage */
	uint8_t	FT_HV_UV;

	/* Fault of GLV (undervoltage measurement) */
	uint8_t	FT_GLV_UV;

	/* Fault of GLV (overvoltage measurement) */
	uint8_t	FT_GLV_OV;

	/* Internal ECUA/BMS faults */
	uint8_t	FT_AMS;

	/* If any error is present */
	uint8_t	FT_ANY;

	/* Warning for cell tempreature (near limits) */
	uint8_t	WARN_TEMP_Cell;

	/* Warning for dc-dc temperature (near limits) */
	uint8_t	WARN_TEMP_DCDC;

	/* Warning for balancer temperature (near limits) */
	uint8_t	WARN_TEMP_Bal;

	/* ECUB powering output is enabled */
	uint8_t	PWR_ECUB;

	/* Fans enabled */
	uint8_t	FANS_EN;

	/* Message up counter for safety */
	uint8_t	SEQ;
} ECUA_Status_t;

/*
 * Calculated data in ACP
 */
typedef struct ECUA_Estimation_t {
	/* Charge lost in motoring */
	uint16_t	Charge_OUT;

	/* Charge reccuperated */
	uint16_t	Charge_IN;

	/* State of charge estimation */
	uint8_t	SOC;
} ECUA_Estimation_t;

/*
 * ECUB Status report
 */
typedef struct ECUB_Status_t {
	/* Shutdown circuit - Front */
	uint8_t	SDC_FRONT;

	/* Shutdown circuit - Shutdown button left */
	uint8_t	SDC_SDBL;

	/* Shutdown circuit - Shutdown button right */
	uint8_t	SDC_SDBR;

	/* Shutdown circuit - High voltage disconnect */
	uint8_t	SDC_HVD;

	/* Shutdown circuit - Brake plausibility device */
	uint8_t	SDC_BSPD;

	/* Shutdown circuit - Motor controller rear */
	uint8_t	SDC_MCUR;

	/* Shutdown circuit - Accumulator management system */
	uint8_t	SDC_AMS;

	/* Shutdown circuit - Tractive system master switch */
	uint8_t	SDC_TSMS;

	/* Current vehicle state */
	enum ECUB_CarState	CarState;

	/* Reason for latest not-ready CarState */
	enum ECUB_Notready_reason	CarState_Notready;

	/* Current powering source */
	enum ECUB_GLV_PowerSource	PowerSource;

	/* Module 1 detected */
	uint8_t	Det_MOD1;

	/* Module 2 detected */
	uint8_t	Det_MOD2;

	/* Module 3 detected */
	uint8_t	Det_MOD3;

	/* Module 4 detected */
	uint8_t	Det_MOD4;

	/* Fault temperature shutdown for driver on PWR_ECUF_EN, PWR_ECUA_EN */
	uint8_t	FT_PWR1_OT;

	/* Fault temperature shutdown for driver on PWR_ECUMF_EN, PWR_ECUMR_EN */
	uint8_t	FT_PWR2_OT;

	/* Fault temperature shutdown for driver on WP1_EN, WP2_EN */
	uint8_t	FT_PWR3_OT;

	/* Fault temperature shutdown for driver on EM, FAN3_EN */
	uint8_t	FT_PWR4_OT;

	/* Fault temperature shutdown for driver on FAN1_EN, FAN2_EN */
	uint8_t	FT_PWR5_OT;

	/* Fault temperature shutdown for driver on RTDS, SDB LED R i L, BrakeLight */
	uint8_t	FT_L2_OT;

	/* If any error present */
	uint8_t	FT_ANY;

	/* Fault temperature shutdown for driver on WS, AUX1,2,3 */
	uint8_t	FT_L1_OT;

	/* Fault ECUF power (Short to VCC) */
	uint8_t	FT_PWR_ECUF_OC;

	/* Fault ECUA power (Short to VCC) */
	uint8_t	FT_PWR_ECUA_OC;

	/* Fault MCF power (Short to VCC) */
	uint8_t	FT_PWR_MCF_OC;

	/* Fault MCR power (Short to VCC) */
	uint8_t	FT_PWR_MCR_OC;

	/* Fault of CAN Bus (CAN errors or controller mode is error) */
	uint8_t	FT_CAN1;

	/* Fault of CAN Bus (CAN errors or controller mode is error) */
	uint8_t	FT_CAN2;

	/* Power to ECUF enabled */
	uint8_t	PWR_ECUF_EN;

	/* Power to ECUA enabled */
	uint8_t	PWR_ECUA_EN;

	/* Power to MCUF enabled */
	uint8_t	PWR_MCUF_EN;

	/* Power to MCUR enabled */
	uint8_t	PWR_MCUR_EN;

	/* Power to Energy Meter enabled */
	uint8_t	PWR_EM_EN;

	/* Power to Waterpump 1 enabled */
	uint8_t	PWR_WP1_EN;

	/* Power to Waterpump 2 enabled */
	uint8_t	PWR_WP2_EN;

	/* Power to FAN1 enabled */
	uint8_t	PWR_FAN1_EN;

	/* Power to FAN2 enabled */
	uint8_t	PWR_FAN2_EN;

	/* Power to FAN3 enabled */
	uint8_t	PWR_FAN3_EN;

	/* Power to Wheel speed sensor enabled */
	uint8_t	PWR_WS_EN;

	/* Power to aux1 enabled aka LIDK1 (Low I Don't Know #1) */
	uint8_t	PWR_AUX1_EN;

	/* Power to aux2 enabled aka LIDK2 */
	uint8_t	PWR_AUX2_EN;

	/* Power to aux3 enabled aka LIDK3 */
	uint8_t	PWR_AUX3_EN;

	/* Ready to drive sound enabled */
	uint8_t	RTDS_EN;

	/* Shutdown button right led enabled */
	uint8_t	SDBR_LED_EN;

	/* Shutdown button left led enabled */
	uint8_t	SDBL_LED_EN;

	/* Brakelight enabled */
	uint8_t	BrakeLight_EN;

	/* TSAL ""Test"" or ""Override"" totally not a hack enabled */
	uint8_t	TSAL_Override;

	/* Message up counter for safety check */
	uint8_t	SEQ;
} ECUB_Status_t;

/*
 * PWM for each fan and waterpump
 */
typedef struct ECUB_Cooling_t {
	/* Waterpump cooler circuit PWM duty */
	uint8_t	WP1;

	/* Waterpump cooler circuit PWM duty */
	uint8_t	WP2;

	/* Cooler FAN in PWM duty */
	uint8_t	FAN1;

	/* Cooler FAN in PWM duty */
	uint8_t	FAN2;

	/* Cooler FAN in PWM duty */
	uint8_t	FAN3;

	/* Warning - Motor temperature over nominal threshold */
	uint8_t	WARN_MOT_FR_TEMP;

	/* Warning - Motor temperature over nominal threshold */
	uint8_t	WARN_MOT_FL_TEMP;

	/* Warning - Motor temperature over nominal threshold */
	uint8_t	WARN_MOT_RR_TEMP;

	/* Warning - Motor temperature over nominal threshold */
	uint8_t	WARN_MOT_RL_TEMP;

	/* Warning - Motor controller temperature over nominal threshold */
	uint8_t	WARN_MCU_FR_TEMP;

	/* Warning - Motor controller temperature over nominal threshold */
	uint8_t	WARN_MCU_FL_TEMP;

	/* Warning - Motor controller temperature over nominal threshold */
	uint8_t	WARN_MCU_RR_TEMP;

	/* Warning - Motor controller temperature over nominal threshold */
	uint8_t	WARN_MCU_RL_TEMP;

	/* Warning - Brake temperature over nominal threshold */
	uint8_t	WARN_Brake_RR_TEMP;

	/* Warning - Brake temperature over nominal threshold */
	uint8_t	WARN_Brake_RL_TEMP;

	/* Fault - motor overtemperature */
	uint8_t	FT_MOT_FR_OT;

	/* Fault - motor overtemperature */
	uint8_t	FT_MOT_FL_OT;

	/* Fault - motor overtemperature */
	uint8_t	FT_MOT_RR_OT;

	/* Fault - motor overtemperature */
	uint8_t	FT_MOT_RL_OT;

	/* Fault - motor controller overtemperature */
	uint8_t	FT_MCU_FR_OT;

	/* Fault - motor controller overtemperature */
	uint8_t	FT_MCU_FL_OT;

	/* Fault - motor controller overtemperature */
	uint8_t	FT_MCU_RR_OT;

	/* Fault - motor controller overtemperature */
	uint8_t	FT_MCU_RL_OT;

	/* Fault - brake overtemperature */
	uint8_t	FT_Brake_RR_OT;

	/* Fault - brake overtemperature */
	uint8_t	FT_Brake_RL_OT;
} ECUB_Cooling_t;

/*
 * Summary of unit status and faults
 */
typedef struct ECUF_Status_t {
	/* Shutdown circuit from SDB_Cockpit */
	uint8_t	SDC_SDBC;

	/* Inertia switch shutdown circuit */
	uint8_t	SDC_Inertia;

	/* Front Wheel Interlock (Connetore Italiano Supreme) */
	uint8_t	SDC_FWIL;

	/* Power to ECUP is enabled */
	uint8_t	PWR_ECUP_EN;

	/* Power to ECUG is enabled */
	uint8_t	PWR_ECUG_EN;

	/* Power to DTLG is enabled */
	uint8_t	PWR_DTLG_EN;

	/* Power to ECUS is enabled */
	uint8_t	PWR_ECUS_EN;

	/* Power to Dash is enabled */
	uint8_t	PWR_DASH_EN;

	/* Power to front brake fans is enabled */
	uint8_t	PWR_FAN_BrakeF_EN;

	/* Warning - Brake temperature over nominal threshold */
	uint8_t	WARN_Brake_FR_TEMP;

	/* Warning - Brake temperature over nominal threshold */
	uint8_t	WARN_Brake_FL_TEMP;

	/* ECUP enabled but no current is drawn */
	uint8_t	FT_PWR_ECUP;

	/* ECUG enabled but no current is drawn */
	uint8_t	FT_PWR_ECUG;

	/* ECUS enabled but no current is drawn */
	uint8_t	FT_PWR_ECUS;

	/* DTLG enabled but no current is drawn */
	uint8_t	FT_PWR_DTLG;

	/* DASH enabled but no current is drawn */
	uint8_t	FT_PWR_DASH;

	/* FAN enabled but no current is drawn */
	uint8_t	FT_PWR_FAN_BrakeF;

	/* STW sensor disconnected or destroyed */
	uint8_t	FT_STW_Sensor;

	/* Fault of steering wheel calibration */
	uint8_t	FT_STW_Cal;

	/* Fault of suspension displacement sensor FR */
	uint8_t	FT_DisFR;

	/* Fault of suspension displacement sensor FL */
	uint8_t	FT_DisFL;

	/* Fault of suspension displacement sensor RR */
	uint8_t	FT_DisRR;

	/* Fault of suspension displacement sensor RL */
	uint8_t	FT_DisRL;

	/* Fault of suspension displacement sensor FR calibration */
	uint8_t	FT_DisFR_Cal;

	/* Fault of suspension displacement sensor FL calibration */
	uint8_t	FT_DisFL_Cal;

	/* Fault of suspension displacement sensor RR calibration */
	uint8_t	FT_DisRR_Cal;

	/* Fault of suspension displacement sensor RL calibration */
	uint8_t	FT_DisRL_Cal;

	/* Fault - brake overtemperature */
	uint8_t	FT_Brake_FR_OT;

	/* Fault - brake overtemperature */
	uint8_t	FT_Brake_FL_OT;

	/* GLV Voltage at ECUF input */
	uint8_t	Volt_GLV_In;
} ECUF_Status_t;

/*
 * Steering wheel angle measurements.
 */
typedef struct ECUF_STW_t {
	/* Steering wheel angle. Left positive */
	int16_t	Angle;

	/* Steering wheel angularspeed. Left positive */
	int16_t	AngularSpeed;

	/* Steering wheel measurement fault */
	uint8_t	FT_STW;

	/* Message up counter for safety */
	uint8_t	SEQ;
} ECUF_STW_t;

/*
 * Suspension displacement sensors (Spring lengths)
 */
typedef struct ECUF_DISSusp_t {
	/* Displacement of front right spring */
	uint16_t	FR;

	/* Displacement of front left spring */
	uint16_t	FL;

	/* Displacement of rear right spring */
	uint16_t	RR;

	/* Displacement of rear left spring */
	uint16_t	RL;
} ECUF_DISSusp_t;

/*
 * Dashboard buttons & switches, but not only that :thinking:
 */
typedef struct ECUF_Dashboard_t {
	/* 1-pressed / 0-not pressed */
	uint8_t	TSON;

	/* 1-pressed / 0-not pressed */
	uint8_t	START;

	/* Waterpump override switch */
	uint8_t	WP_ON;

	/* Traction control switch */
	uint8_t	TCS_ON;

	/* Yaw control stabilization switch */
	uint8_t	YC_ON;

	/* Ambient light level */
	uint8_t	AmbientLight;

	/* Ambient temperature */
	uint8_t	AmbientTemp;
} ECUF_Dashboard_t;

/*
 * Front suspension temperatures
 */
typedef struct ECUF_TEMPSuspF_t {
	/* Front right brake caliper temperature */
	uint8_t	BrakeCal_FR;

	/* Front left brake caliper temperature */
	uint8_t	BrakeCal_FL;

	/* Front right tire inboard temperature */
	uint8_t	TireI_FR;

	/* Front right tire center temperature */
	uint8_t	TireC_FR;

	/* Front right tire outboard temperature */
	uint8_t	TireO_FR;

	/* Front left tire inboard  temperature */
	uint8_t	TireI_FL;

	/* Front left tire center temperature */
	uint8_t	TireC_FL;

	/* Front left tire outboard temperature */
	uint8_t	TireO_FL;
} ECUF_TEMPSuspF_t;

/*
 * Request calibration of steering wheel
 */
typedef struct ECUF_REQCalibSTW_t {
	/* Which steering wheel position to calibrate */
	enum ECUF_CAL_STWIndex	which;
} ECUF_REQCalibSTW_t;

/*
 * ECUP Status message
 */
typedef struct ECUP_Status_t {
	/* SDC is powered after BOTS */
	uint8_t	SDC_BOTS;

	/* If any error is present */
	uint8_t	FT_ANY;

	/* Accelerator pedals plausible */
	uint8_t	APPS_Plausible;

	/* Brake Pedal Plausibility Check (software bspd) */
	uint8_t	BPPC_Latch;

	/* Brake active (Signal mainly for brakelight) */
	uint8_t	BrakeActive;

	/* Brake active for the purposes of BSPD */
	uint8_t	BrakeActive_BSPD;
} ECUP_Status_t;

/*
 * VDCU systems status
 */
typedef struct VDCU_Status_t {
	/* VDCU State machine */
	enum VDCU_VDCU_State	State;

	/* Torque vectoring enabled */
	uint8_t	TV_ENABLED;

	/* Slip ratio control enabled (traction control) */
	uint8_t	TC_ENABLED;

	/* Yaw controller enabled */
	uint8_t	YC_ENABLED;

	/* Fault of displacement sensor calibration */
	uint8_t	FT_Dis_Cal;

	/*  */
	uint8_t	FT_Sensor;

	/* Derating due to components temperature */
	uint8_t	TEMP_derating;

	/* Derating due to ACP limits */
	uint8_t	ACP_derating;

	/* Discharge is active */
	uint8_t	Disch_ACT;

	/* Reverse is activated */
	uint8_t	Reverse_ACT;

	/* Slip ratio control is limiting torque */
	uint8_t	TC_ACT;

	/* Yaw controller is producing yaw torque */
	uint8_t	YC_ACT;
} VDCU_Status_t;

void candbInit(void);

int ECUA_decode_Status_s(const uint8_t* bytes, size_t length, ECUA_Status_t* data_out);
int ECUA_decode_Status(const uint8_t* bytes, size_t length, uint8_t* SDC_IN_Open_out, uint8_t* SDC_HV_ILOCK_out, uint8_t* SDC_IMD_out, uint8_t* SDC_AMS_out, uint8_t* SDC_OUT_out, uint8_t* SDC_END_out, uint8_t* LATCH_SDC_AMS_out, uint8_t* AIRsState_out, uint8_t* ChargingState_out, enum ECUA_StateAMS* AMSState_out, uint8_t* FT_ACP_OT_out, uint8_t* FT_AIRS_out, uint8_t* FT_DCDC_out, uint8_t* FT_FAN1_out, uint8_t* FT_FAN2_out, uint8_t* FT_FAN3_out, uint8_t* FT_HV_OV_out, uint8_t* FT_HV_UV_out, uint8_t* FT_GLV_UV_out, uint8_t* FT_GLV_OV_out, uint8_t* FT_AMS_out, uint8_t* FT_ANY_out, uint8_t* WARN_TEMP_Cell_out, uint8_t* WARN_TEMP_DCDC_out, uint8_t* WARN_TEMP_Bal_out, uint8_t* PWR_ECUB_out, uint8_t* FANS_EN_out, uint8_t* SEQ_out);
int ECUA_get_Status(ECUA_Status_t* data_out);
void ECUA_Status_on_receive(int (*callback)(ECUA_Status_t* data));

int ECUA_decode_Estimation_s(const uint8_t* bytes, size_t length, ECUA_Estimation_t* data_out);
int ECUA_decode_Estimation(const uint8_t* bytes, size_t length, uint16_t* Charge_OUT_out, uint16_t* Charge_IN_out, uint8_t* SOC_out);
int ECUA_get_Estimation(ECUA_Estimation_t* data_out);
void ECUA_Estimation_on_receive(int (*callback)(ECUA_Estimation_t* data));

int ECUB_decode_Status_s(const uint8_t* bytes, size_t length, ECUB_Status_t* data_out);
int ECUB_decode_Status(const uint8_t* bytes, size_t length, uint8_t* SDC_FRONT_out, uint8_t* SDC_SDBL_out, uint8_t* SDC_SDBR_out, uint8_t* SDC_HVD_out, uint8_t* SDC_BSPD_out, uint8_t* SDC_MCUR_out, uint8_t* SDC_AMS_out, uint8_t* SDC_TSMS_out, enum ECUB_CarState* CarState_out, enum ECUB_Notready_reason* CarState_Notready_out, enum ECUB_GLV_PowerSource* PowerSource_out, uint8_t* Det_MOD1_out, uint8_t* Det_MOD2_out, uint8_t* Det_MOD3_out, uint8_t* Det_MOD4_out, uint8_t* FT_PWR1_OT_out, uint8_t* FT_PWR2_OT_out, uint8_t* FT_PWR3_OT_out, uint8_t* FT_PWR4_OT_out, uint8_t* FT_PWR5_OT_out, uint8_t* FT_L2_OT_out, uint8_t* FT_ANY_out, uint8_t* FT_L1_OT_out, uint8_t* FT_PWR_ECUF_OC_out, uint8_t* FT_PWR_ECUA_OC_out, uint8_t* FT_PWR_MCF_OC_out, uint8_t* FT_PWR_MCR_OC_out, uint8_t* FT_CAN1_out, uint8_t* FT_CAN2_out, uint8_t* PWR_ECUF_EN_out, uint8_t* PWR_ECUA_EN_out, uint8_t* PWR_MCUF_EN_out, uint8_t* PWR_MCUR_EN_out, uint8_t* PWR_EM_EN_out, uint8_t* PWR_WP1_EN_out, uint8_t* PWR_WP2_EN_out, uint8_t* PWR_FAN1_EN_out, uint8_t* PWR_FAN2_EN_out, uint8_t* PWR_FAN3_EN_out, uint8_t* PWR_WS_EN_out, uint8_t* PWR_AUX1_EN_out, uint8_t* PWR_AUX2_EN_out, uint8_t* PWR_AUX3_EN_out, uint8_t* RTDS_EN_out, uint8_t* SDBR_LED_EN_out, uint8_t* SDBL_LED_EN_out, uint8_t* BrakeLight_EN_out, uint8_t* TSAL_Override_out, uint8_t* SEQ_out);
int ECUB_get_Status(ECUB_Status_t* data_out);
void ECUB_Status_on_receive(int (*callback)(ECUB_Status_t* data));

int ECUB_decode_Cooling_s(const uint8_t* bytes, size_t length, ECUB_Cooling_t* data_out);
int ECUB_decode_Cooling(const uint8_t* bytes, size_t length, uint8_t* WP1_out, uint8_t* WP2_out, uint8_t* FAN1_out, uint8_t* FAN2_out, uint8_t* FAN3_out, uint8_t* WARN_MOT_FR_TEMP_out, uint8_t* WARN_MOT_FL_TEMP_out, uint8_t* WARN_MOT_RR_TEMP_out, uint8_t* WARN_MOT_RL_TEMP_out, uint8_t* WARN_MCU_FR_TEMP_out, uint8_t* WARN_MCU_FL_TEMP_out, uint8_t* WARN_MCU_RR_TEMP_out, uint8_t* WARN_MCU_RL_TEMP_out, uint8_t* WARN_Brake_RR_TEMP_out, uint8_t* WARN_Brake_RL_TEMP_out, uint8_t* FT_MOT_FR_OT_out, uint8_t* FT_MOT_FL_OT_out, uint8_t* FT_MOT_RR_OT_out, uint8_t* FT_MOT_RL_OT_out, uint8_t* FT_MCU_FR_OT_out, uint8_t* FT_MCU_FL_OT_out, uint8_t* FT_MCU_RR_OT_out, uint8_t* FT_MCU_RL_OT_out, uint8_t* FT_Brake_RR_OT_out, uint8_t* FT_Brake_RL_OT_out);
int ECUB_get_Cooling(ECUB_Cooling_t* data_out);
void ECUB_Cooling_on_receive(int (*callback)(ECUB_Cooling_t* data));

int ECUF_send_Status_s(const ECUF_Status_t* data);
int ECUF_send_Status(uint8_t SDC_SDBC, uint8_t SDC_Inertia, uint8_t SDC_FWIL, uint8_t PWR_ECUP_EN, uint8_t PWR_ECUG_EN, uint8_t PWR_DTLG_EN, uint8_t PWR_ECUS_EN, uint8_t PWR_DASH_EN, uint8_t PWR_FAN_BrakeF_EN, uint8_t WARN_Brake_FR_TEMP, uint8_t WARN_Brake_FL_TEMP, uint8_t FT_PWR_ECUP, uint8_t FT_PWR_ECUG, uint8_t FT_PWR_ECUS, uint8_t FT_PWR_DTLG, uint8_t FT_PWR_DASH, uint8_t FT_PWR_FAN_BrakeF, uint8_t FT_STW_Sensor, uint8_t FT_STW_Cal, uint8_t FT_DisFR, uint8_t FT_DisFL, uint8_t FT_DisRR, uint8_t FT_DisRL, uint8_t FT_DisFR_Cal, uint8_t FT_DisFL_Cal, uint8_t FT_DisRR_Cal, uint8_t FT_DisRL_Cal, uint8_t FT_Brake_FR_OT, uint8_t FT_Brake_FL_OT, uint8_t Volt_GLV_In);
int ECUF_Status_need_to_send(void);

int ECUF_send_STW_s(const ECUF_STW_t* data);
int ECUF_send_STW(int16_t Angle, int16_t AngularSpeed, uint8_t FT_STW, uint8_t SEQ);
int ECUF_STW_need_to_send(void);

int ECUF_send_DISSusp_s(const ECUF_DISSusp_t* data);
int ECUF_send_DISSusp(uint16_t FR, uint16_t FL, uint16_t RR, uint16_t RL);
int ECUF_DISSusp_need_to_send(void);

int ECUF_send_Dashboard_s(const ECUF_Dashboard_t* data);
int ECUF_send_Dashboard(uint8_t TSON, uint8_t START, uint8_t WP_ON, uint8_t TCS_ON, uint8_t YC_ON, uint8_t AmbientLight, uint8_t AmbientTemp);
int ECUF_Dashboard_need_to_send(void);

int ECUF_send_TEMPSuspF_s(const ECUF_TEMPSuspF_t* data);
int ECUF_send_TEMPSuspF(uint8_t BrakeCal_FR, uint8_t BrakeCal_FL, uint8_t TireI_FR, uint8_t TireC_FR, uint8_t TireO_FR, uint8_t TireI_FL, uint8_t TireC_FL, uint8_t TireO_FL);
int ECUF_TEMPSuspF_need_to_send(void);

int ECUF_decode_REQCalibSTW_s(const uint8_t* bytes, size_t length, ECUF_REQCalibSTW_t* data_out);
int ECUF_decode_REQCalibSTW(const uint8_t* bytes, size_t length, enum ECUF_CAL_STWIndex* which_out);
int ECUF_get_REQCalibSTW(ECUF_REQCalibSTW_t* data_out);
void ECUF_REQCalibSTW_on_receive(int (*callback)(ECUF_REQCalibSTW_t* data));

int ECUP_decode_Status_s(const uint8_t* bytes, size_t length, ECUP_Status_t* data_out);
int ECUP_decode_Status(const uint8_t* bytes, size_t length, uint8_t* SDC_BOTS_out, uint8_t* FT_ANY_out, uint8_t* APPS_Plausible_out, uint8_t* BPPC_Latch_out, uint8_t* BrakeActive_out, uint8_t* BrakeActive_BSPD_out);
int ECUP_get_Status(ECUP_Status_t* data_out);
void ECUP_Status_on_receive(int (*callback)(ECUP_Status_t* data));

int VDCU_decode_Status_s(const uint8_t* bytes, size_t length, VDCU_Status_t* data_out);
int VDCU_decode_Status(const uint8_t* bytes, size_t length, enum VDCU_VDCU_State* State_out, uint8_t* TV_ENABLED_out, uint8_t* TC_ENABLED_out, uint8_t* YC_ENABLED_out, uint8_t* FT_Dis_Cal_out, uint8_t* FT_Sensor_out, uint8_t* TEMP_derating_out, uint8_t* ACP_derating_out, uint8_t* Disch_ACT_out, uint8_t* Reverse_ACT_out, uint8_t* TC_ACT_out, uint8_t* YC_ACT_out);
int VDCU_get_Status(VDCU_Status_t* data_out);
void VDCU_Status_on_receive(int (*callback)(VDCU_Status_t* data));

#ifdef __cplusplus
}
#endif

#endif
