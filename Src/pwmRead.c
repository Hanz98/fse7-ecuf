#include "calibration.h"
#include "can_ECUF.h"
#include "pwm.h"

// FIXME: handle errors

extern PwmInput pwmSWS;

static uint16_t Get_Raw_Angle() {
	uint16_t raw;
	PwmInput_Get(&pwmSWS, &raw);

	return (360 * DEGREE * raw / 0xffff);
}

int Get_Steering_Angle(int *angle){

	// TODO: check direction!
	*angle = (Get_Raw_Angle() - Calibration.Center);

	if (abs(*angle) > 110 * DEGREE) // Check overrange maximum possible steering wheel angle is 110 Deg
		return 0;
	//angle=ApplyCalibration(Get_PWM_Duty_Cycle(),4096,Calibration.Left,Calibration.Center,Calibration.Right);

	return 1;
}

void calib(ECUF_REQCalibSTW_t* reqCalib){

	if(reqCalib->which == ECUF_CAL_STWIndex_STWLeft) {
		//PwmInput_Get(&pwmSWS, &Calibration.Left);
	}

	if(reqCalib->which == ECUF_CAL_STWIndex_STWRight) {
		//PwmInput_Get(&pwmSWS, &Calibration.Right);
	}

	if(reqCalib->which == ECUF_CAL_STWIndex_STWCenter) {
		//PwmInput_Get(&pwmSWS, &Calibration.Center);
		Calibration.Center = Get_Raw_Angle();
	}
	SaveCalibration();
}

/*
 *
 *		filter
 *
void MedianInit(MEDIAN_TypeDef* median){

    median->index=0;
    median->len=MEDIAN_FILTER_WINDOW;
    median->win_len=0;
    for(int i=0;i<median->len;++i){
        median->arr[i]=0;
    }

}

void sortBubble(int32_t *array, int32_t size)
{
    int32_t i, j;
    int32_t tmp;

    for(i = 0; i < size-1; i++)
    {
        for(j = 0; j < size-i-1; j++)
        {
            if(array[j+1] < array[j])
            {
                tmp = array[j+1];
                array[j+1] = array[j];
                array[j] = tmp;
            }
        }
    }
}
int32_t medianFilt(MEDIAN_TypeDef *median)
{
    int32_t temp[median->win_len];
    int8_t i,j;

    // Find the corresponding data
    for(i=0; i < median->win_len; i++)
    {
        j = median->index - i;
        if(j < 0) j =  median->len + j;

        temp[i] = median->arr[j];
    }

    sortBubble(temp, median->win_len);


    // Median filter calculation
    if(median->win_len % 2)
    {
        return temp[median->win_len>>1];
    }
    else
    {
        return ( ( temp[median->win_len>>1] + temp[(median->win_len>>1)-1] ) >> 1);
    }
}

*/
