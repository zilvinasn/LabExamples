/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l0xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define B1_EXTI_IRQn EXTI4_15_IRQn
#define MCO_Pin GPIO_PIN_0
#define MCO_GPIO_Port GPIOH
#define Seg_G_Pin GPIO_PIN_0
#define Seg_G_GPIO_Port GPIOC
#define Seg_D_Pin GPIO_PIN_1
#define Seg_D_GPIO_Port GPIOC
#define Seg_E_Pin GPIO_PIN_2
#define Seg_E_GPIO_Port GPIOC
#define Seg_C_Pin GPIO_PIN_3
#define Seg_C_GPIO_Port GPIOC
#define LDR_ADC1_Pin GPIO_PIN_1
#define LDR_ADC1_GPIO_Port GPIOA
#define LD2_Pin GPIO_PIN_5
#define LD2_GPIO_Port GPIOA
#define Seg_B_Pin GPIO_PIN_4
#define Seg_B_GPIO_Port GPIOC
#define Seg_F_Pin GPIO_PIN_5
#define Seg_F_GPIO_Port GPIOC
#define Button_1_Kamami_Pin GPIO_PIN_0
#define Button_1_Kamami_GPIO_Port GPIOB
#define GPIO_OutPB15_Pin GPIO_PIN_15
#define GPIO_OutPB15_GPIO_Port GPIOB
#define Seg_A_Pin GPIO_PIN_6
#define Seg_A_GPIO_Port GPIOC
#define Decimal_Point_Pin GPIO_PIN_7
#define Decimal_Point_GPIO_Port GPIOC
#define COM4_Pin GPIO_PIN_8
#define COM4_GPIO_Port GPIOC
#define COM3_Pin GPIO_PIN_9
#define COM3_GPIO_Port GPIOC
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define COM2_Pin GPIO_PIN_10
#define COM2_GPIO_Port GPIOC
#define COM1_Pin GPIO_PIN_11
#define COM1_GPIO_Port GPIOC
#define RGB_LD4_Pin GPIO_PIN_6
#define RGB_LD4_GPIO_Port GPIOB
#define RGB_LD1_Pin GPIO_PIN_9
#define RGB_LD1_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
