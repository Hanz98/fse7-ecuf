#include "pwmRead.h"

//extern TIM_HandleTypeDef htim1;


void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance==TIM1)
        {
            if(HAL_GPIO_ReadPin(SWS_SIG_IN_GPIO_Port,SWS_SIG_IN_Pin)){
                PWM_Period=__HAL_TIM_GET_COMPARE(&htim1, TIM_CHANNEL_1);
                __HAL_TIM_SET_COUNTER(&htim1, 0);
            }
            else{
            	STRW_Sensor_Reading=__HAL_TIM_GET_COMPARE(&htim1, TIM_CHANNEL_1);
            	STW_last_interrupt=HAL_GetTick();
            }

        }
}

uint16_t Get_STRW_Raw_Angle() {
    return (360 * DEGREE * STRW_Sensor_Reading / PWM_Period);
}
int Get_STRW_Calibrated_Angle(){
    int angle;
    angle = (Get_STRW_Raw_Angle() - Calibration.Center);

    if (angle < -128 * DEGREE)
        angle = -128 * DEGREE;
    else if (angle > 127 * DEGREE)
        angle = 127 * DEGREE;
    return angle;
}

void calib(){
	if(calibration.which == ECUF_CAL_STWIndex_STWLeft) {

	}
}

/*
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
