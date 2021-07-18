/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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
#include "stm32l4xx_hal.h"

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
#define DIODE_0_Pin GPIO_PIN_13
#define DIODE_0_GPIO_Port GPIOC
#define ADC_4_Pin GPIO_PIN_0
#define ADC_4_GPIO_Port GPIOC
#define ADC_3_Pin GPIO_PIN_1
#define ADC_3_GPIO_Port GPIOC
#define ADC_2_Pin GPIO_PIN_2
#define ADC_2_GPIO_Port GPIOC
#define ADC_1_Pin GPIO_PIN_3
#define ADC_1_GPIO_Port GPIOC
#define ADC_0_Pin GPIO_PIN_0
#define ADC_0_GPIO_Port GPIOA
#define RTD_input_Pin GPIO_PIN_4
#define RTD_input_GPIO_Port GPIOC
#define RTD_input_EXTI_IRQn EXTI4_IRQn
#define BRK_input_Pin GPIO_PIN_2
#define BRK_input_GPIO_Port GPIOB
#define BRK_input_EXTI_IRQn EXTI2_IRQn
#define D_OUTPUT_4_Pin GPIO_PIN_8
#define D_OUTPUT_4_GPIO_Port GPIOA
#define D_OUTPUT_3_Pin GPIO_PIN_9
#define D_OUTPUT_3_GPIO_Port GPIOA
#define D_OUTPUT_2_Pin GPIO_PIN_10
#define D_OUTPUT_2_GPIO_Port GPIOA
#define D_OUTPUT_1_Pin GPIO_PIN_11
#define D_OUTPUT_1_GPIO_Port GPIOA
#define JUMPER_Pin GPIO_PIN_6
#define JUMPER_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
