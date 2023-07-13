/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "stm32f4xx_hal.h"

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
extern UART_HandleTypeDef huart2;
extern SPI_HandleTypeDef Button_hspi1;
extern SPI_HandleTypeDef EL_hspi2;
extern SPI_HandleTypeDef ESP32_hspi3;

#define Cat1 1
#define Cat2 2
#define Cat3 3
#define Cat4 4
#define Cat5 5
#define Cat6 6
#define Cat7 7
#define Cat8 8
#define Cat9 9
#define Cat10 10
#define Sel_B_Module1 1
#define Sel_B_Module2 2
#define Sel_B_Module3 3
#define Sel_B_Module4 4
#define Sel_EL_Module 5
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define nRF_EL_CSN_Pin GPIO_PIN_0
#define nRF_EL_CSN_GPIO_Port GPIOC
#define nRF_EL_IRQ_Pin GPIO_PIN_1
#define nRF_EL_IRQ_GPIO_Port GPIOC
#define nRF_EL_IRQ_EXTI_IRQn EXTI1_IRQn
#define nRF_EL_MISO_Pin GPIO_PIN_2
#define nRF_EL_MISO_GPIO_Port GPIOC
#define nRF_EL_MOSI_Pin GPIO_PIN_3
#define nRF_EL_MOSI_GPIO_Port GPIOC
#define nRF_EL_CE_Pin GPIO_PIN_0
#define nRF_EL_CE_GPIO_Port GPIOA
#define nRF_B_SCK_Pin GPIO_PIN_5
#define nRF_B_SCK_GPIO_Port GPIOA
#define nRF_B_MISO_Pin GPIO_PIN_6
#define nRF_B_MISO_GPIO_Port GPIOA
#define nRF_B_MOSI_Pin GPIO_PIN_7
#define nRF_B_MOSI_GPIO_Port GPIOA
#define nRF_B_IRQ1_Pin GPIO_PIN_4
#define nRF_B_IRQ1_GPIO_Port GPIOC
#define nRF_B_IRQ1_EXTI_IRQn EXTI4_IRQn
#define nRF_B_CSN1_Pin GPIO_PIN_5
#define nRF_B_CSN1_GPIO_Port GPIOC
#define nRF_B_CE1_Pin GPIO_PIN_0
#define nRF_B_CE1_GPIO_Port GPIOB
#define nRF_B_CSN2_Pin GPIO_PIN_1
#define nRF_B_CSN2_GPIO_Port GPIOB
#define nRF_B_CE2_Pin GPIO_PIN_2
#define nRF_B_CE2_GPIO_Port GPIOB
#define nRF_EL_SCK_Pin GPIO_PIN_10
#define nRF_EL_SCK_GPIO_Port GPIOB
#define nRF_B_CSN3_Pin GPIO_PIN_11
#define nRF_B_CSN3_GPIO_Port GPIOB
#define nRF_B_IRQ2_Pin GPIO_PIN_12
#define nRF_B_IRQ2_GPIO_Port GPIOB
#define nRF_B_IRQ2_EXTI_IRQn EXTI15_10_IRQn
#define nRF_B_CE3_Pin GPIO_PIN_13
#define nRF_B_CE3_GPIO_Port GPIOB
#define nRF_B_IRQ3_Pin GPIO_PIN_14
#define nRF_B_IRQ3_GPIO_Port GPIOB
#define nRF_B_IRQ3_EXTI_IRQn EXTI15_10_IRQn
#define nRF_B_IRQ4_Pin GPIO_PIN_6
#define nRF_B_IRQ4_GPIO_Port GPIOC
#define nRF_B_IRQ4_EXTI_IRQn EXTI9_5_IRQn
#define nRF_B_CE4_Pin GPIO_PIN_7
#define nRF_B_CE4_GPIO_Port GPIOC
#define nRF_B_CSN4_Pin GPIO_PIN_8
#define nRF_B_CSN4_GPIO_Port GPIOC
#define T_LED2_Pin GPIO_PIN_9
#define T_LED2_GPIO_Port GPIOC
#define T_Key2_Pin GPIO_PIN_8
#define T_Key2_GPIO_Port GPIOA
#define T_LED1_Pin GPIO_PIN_9
#define T_LED1_GPIO_Port GPIOA
#define T_Key1_Pin GPIO_PIN_10
#define T_Key1_GPIO_Port GPIOA
#define ESP32_SCK_Pin GPIO_PIN_10
#define ESP32_SCK_GPIO_Port GPIOC
#define ESP32_MISO_Pin GPIO_PIN_11
#define ESP32_MISO_GPIO_Port GPIOC
#define ESP32_MOSI_Pin GPIO_PIN_12
#define ESP32_MOSI_GPIO_Port GPIOC
#define ESP32_CS_Pin GPIO_PIN_2
#define ESP32_CS_GPIO_Port GPIOD

/* USER CODE BEGIN Private defines */
//#define Debug
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
