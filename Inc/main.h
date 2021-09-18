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
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
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
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
void vTaskOpros (void *argument);
void vHandlerTask (void *argument);
void vTaskRasZasl (void *argument);
void vTaskVoda (void *argument);
void vTaskTime (void *argument);
uint8_t BunkerStatus (void);
void inESP (void);
void inNextion (void);
float map(int x, int in_min, int in_max, int out_min, int out_max);
int constrain(int x, int a, int b);
void setServo(uint8_t numServo, int angle);
int ReadTempBunker (void);
int ReadTempRiser (void);
void SendUSART1 (char chr);
void SendStringUSART1 (char* str);
void SendDataUSART1 (uint8_t data);
void SendUSART3 (char chr);
void SendStringUSART3 (char* str);
void SendDataUSART3 (uint8_t data);
void InitSPI (void);
void InitUSART1 (void);
void InitUSART3 (void);
void myDelay(int32_t delay);
void calcPrimary(void);
void calcSecondary(void);
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
#define SECOND				1000
#define AMPLITUDE 		544				// 1500, if amplitude = 90; 544, if amplitude = 180
#define NASOS_ON 			GPIOC->BSRR |= GPIO_BSRR_BS14;
#define NASOS_OFF 		GPIOC->BSRR |= GPIO_BSRR_BR14;
#define CS1_LOW    		GPIOB->BSRR |= GPIO_BSRR_BR8;
#define CS1_HIGH   		GPIOB->BSRR |= GPIO_BSRR_BS8;

#define CS2_LOW    		GPIOB->BSRR |= GPIO_BSRR_BR9;
#define CS2_HIGH   		GPIOB->BSRR |= GPIO_BSRR_BS9;

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED_Pin GPIO_PIN_13
#define LED_GPIO_Port GPIOC
#define Relay_Pin GPIO_PIN_14
#define Relay_GPIO_Port GPIOC
#define DS18B20_Pin GPIO_PIN_2
#define DS18B20_GPIO_Port GPIOA
#define CS1_Pin GPIO_PIN_8
#define CS1_GPIO_Port GPIOB
#define CS2_Pin GPIO_PIN_9
#define CS2_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
