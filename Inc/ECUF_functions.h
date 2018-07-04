


void checkDash(ECUF_Dashboard_t* data);
void checkShutdown(ECUF_Status_t* data);
void checkPwm();
void dashInit(void);
void dashControl(void);
void idicatroControl(void);
void barControl(void);
void sendData(void);
void receiveData(void);
void fanCheck(void);
void dashBright(void);
void breakTemp(void);
void suspensionDis(void);
int32_t rescale(int32_t value, float oldMin, float oldMax, float newMin, float newMax);
