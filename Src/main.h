/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H__
#define __MAIN_H__

/* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define DASH_STROBE_Pin GPIO_PIN_2
#define DASH_STROBE_GPIO_Port GPIOE
#define EN_DASH_VCC_Pin GPIO_PIN_3
#define EN_DASH_VCC_GPIO_Port GPIOE
#define EN_DLTG_VCC_Pin GPIO_PIN_4
#define EN_DLTG_VCC_GPIO_Port GPIOE
#define EN_I_SENSE_Pin GPIO_PIN_5
#define EN_I_SENSE_GPIO_Port GPIOE
#define SEL_0_Pin GPIO_PIN_6
#define SEL_0_GPIO_Port GPIOE
#define ECUS_DTLG_FRST_Pin GPIO_PIN_2
#define ECUS_DTLG_FRST_GPIO_Port GPIOC
#define EN_ECUS_VCC_Pin GPIO_PIN_3
#define EN_ECUS_VCC_GPIO_Port GPIOC
#define I_DLTG_ECUS_Pin GPIO_PIN_0
#define I_DLTG_ECUS_GPIO_Port GPIOA
#define I_ECUP_FAN_Pin GPIO_PIN_1
#define I_ECUP_FAN_GPIO_Port GPIOA
#define I_DASH_ECUG_Pin GPIO_PIN_2
#define I_DASH_ECUG_GPIO_Port GPIOA
#define V_PHOTOTRAN_Pin GPIO_PIN_3
#define V_PHOTOTRAN_GPIO_Port GPIOA
#define VCC_Pin GPIO_PIN_4
#define VCC_GPIO_Port GPIOA
#define SWS_SIG_IN_Pin GPIO_PIN_9
#define SWS_SIG_IN_GPIO_Port GPIOE
#define SWS_ERR_IN_Pin GPIO_PIN_10
#define SWS_ERR_IN_GPIO_Port GPIOE
#define TSON_Pin GPIO_PIN_12
#define TSON_GPIO_Port GPIOE
#define SDBC_Pin GPIO_PIN_13
#define SDBC_GPIO_Port GPIOE
#define CIS_Pin GPIO_PIN_14
#define CIS_GPIO_Port GPIOE
#define INERTIA_Pin GPIO_PIN_15
#define INERTIA_GPIO_Port GPIOE
#define EN_LED_SDBC_Pin GPIO_PIN_12
#define EN_LED_SDBC_GPIO_Port GPIOB
#define EN_TT_VCC_Pin GPIO_PIN_14
#define EN_TT_VCC_GPIO_Port GPIOB
#define DASH_SDI_Pin GPIO_PIN_15
#define DASH_SDI_GPIO_Port GPIOB
#define EN_ECUP_VCC_Pin GPIO_PIN_9
#define EN_ECUP_VCC_GPIO_Port GPIOD
#define EN_FAN_BAR_VCC_Pin GPIO_PIN_10
#define EN_FAN_BAR_VCC_GPIO_Port GPIOD
#define ECUP_FAN_FRST_Pin GPIO_PIN_11
#define ECUP_FAN_FRST_GPIO_Port GPIOD
#define SEL_1_Pin GPIO_PIN_14
#define SEL_1_GPIO_Port GPIOD
#define DASH_ECUG_FRST_Pin GPIO_PIN_2
#define DASH_ECUG_FRST_GPIO_Port GPIOD
#define EN_ECUG_VCC_Pin GPIO_PIN_3
#define EN_ECUG_VCC_GPIO_Port GPIOD
#define START_Pin GPIO_PIN_4
#define START_GPIO_Port GPIOD
#define SW1_Pin GPIO_PIN_5
#define SW1_GPIO_Port GPIOD
#define SW2_Pin GPIO_PIN_6
#define SW2_GPIO_Port GPIOD
#define SW3_Pin GPIO_PIN_7
#define SW3_GPIO_Port GPIOD
#define DASH_CS_Pin GPIO_PIN_4
#define DASH_CS_GPIO_Port GPIOB
#define CS_Pt1000_R_Pin GPIO_PIN_7
#define CS_Pt1000_R_GPIO_Port GPIOB
#define DRDY_Pt1000_R_Pin GPIO_PIN_8
#define DRDY_Pt1000_R_GPIO_Port GPIOB
#define CS_Pt1000_L_Pin GPIO_PIN_9
#define CS_Pt1000_L_GPIO_Port GPIOB
#define DRDY_Pt1000_L_Pin GPIO_PIN_0
#define DRDY_Pt1000_L_GPIO_Port GPIOE

/* ########################## Assert Selection ############################## */
/**
  * @brief Uncomment the line below to expanse the "assert_param" macro in the 
  *        HAL drivers code
  */
/* #define USE_FULL_ASSERT    1U */

/* USER CODE BEGIN Private defines */
#define RED 0x5555555555555555
#define GREEN 0xAAAAAAAAAAAAAAAA

#define EFORCE 0xFC00
#define	CAN_ER 0x0080
#define LW_ER 0x0040
#define TEMP_ER 0x0020
#define STABILIZATION 0X0008
#define TRACTION 0x0004
#define IMPLAUSILITY 0x0001
#define RTD 0x0100

#define RE_EFORCE 0x03FF
#define	RE_CAN_ER 0xFF7F
#define RE_LW_ER 0xFFCF
#define RE_TEMP_ER 0xFFDF
#define RE_STABILIZATION 0XFFF7
#define RE_TRACTION 0xFFFD
#define RE_IMPLAUSILITY 0xFFFE
#define RE_RTD 0xFEFF

#define PRESS 0
#define BREAK 0
#define LPF 0.25

enum photoState {LONG = 1, SHORT = 0};
enum photoState stateP;

/* USER CODE END Private defines */

#ifdef __cplusplus
 extern "C" {
#endif
void _Error_Handler(char *, int);

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)
#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
