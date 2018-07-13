#ifndef CALIBRATION_H
#define CALIBRATION_H

#include <stdint.h>

// 1 degree = 16, to get more precision for internal processing
#define DEGREE		16

struct STW_Calibration {
	uint16_t Left;
	uint16_t Center;
	uint16_t Right;
};

extern struct STW_Calibration Calibration;

void SaveCalibration(void);
void LoadCalibration(void);

#endif
