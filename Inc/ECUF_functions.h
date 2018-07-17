#ifndef __ECUF_FUNCTIONS_H__
#define __ECUF_FUNCTIONS_H__


typedef struct {
	uint8_t state;
	uint16_t mask;
	int currentTick, lastTick, period;
}ledBlink_T;


void bspdInit(ledBlink_T *led);
void tractionInit(ledBlink_T *led);
void stabilizationInit(ledBlink_T *led);



void checkDash(ECUF_Dashboard_t* data);
void checkShutdown(ECUF_Status_t* data);
void checkPwm();
void dashInit(void);
void dashControl(void);
void idicatroControl(void);
void barControl(uint16_t soc);
void sendData(void);
void receiveData(void);
void fanCheck(void);
void dashBright(void);
void breakTemp(void);
void suspensionDis(void);
int32_t rescale(int32_t value, float oldMin, float oldMax, float newMin, float newMax);
void dashUpdate(uint8_t *dataGreen, uint8_t *dataRed);
void ledInit();


#endif
