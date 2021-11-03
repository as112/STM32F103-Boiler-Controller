/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "iwdg.h"
#include "rtc.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
extern SPI_HandleTypeDef hspi1;
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI2_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  MX_TIM1_Init();
  MX_IWDG_Init();
  MX_RTC_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
	HAL_SPI_MspDeInit(&hspi2);
	HAL_UART_MspDeInit(&huart1);
	InitSPI();
	InitUSART1();
	BKP->RTCCR &= ~BKP_RTCCR_ASOE;
//	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
//	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
//	InitUSART3();
  /* USER CODE END 2 */

  /* Call init function for freertos objects (in freertos.c) */
  MX_FREERTOS_Init(); 
  /* Start scheduler */
  osKernelStart();
 
  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

float map(int x, int in_min, int in_max, int out_min, int out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

int constrain(int x, int a, int b) {
	if(x > b) x = b;
	if(x < a) x = a;
	return x;
}

void setServo(uint8_t numServo, int angle) {
	uint8_t send_data[5];
	int angleMap = (int)(map(angle, 0, 180, AMPLITUDE, 2400));
	send_data[0] = numServo;
	send_data[1] = (angleMap >> 24) & 0xFF;
	send_data[2] = (angleMap >> 16) & 0xFF;
	send_data[3] = (angleMap >> 8) & 0xFF;
	send_data[4] = angleMap & 0xFF;
	
	CS_SERVO_LOW
	HAL_SPI_Transmit(&hspi1, send_data, 5, 50);
	CS_SERVO_HIGH
	
//	if(numServo == 1) TIM3->CCR1 = (int)(map(angle, 0, 180, AMPLITUDE, 2400));
//	if(numServo == 2) TIM3->CCR2 = (int)(map(angle, 0, 180, AMPLITUDE, 2400));
}

void InitSPI (void){
	
		RCC->APB1ENR |= RCC_APB1ENR_SPI2EN;
	  RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;
		RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;
	
	  GPIOB->CRH &= ~GPIO_CRH_CNF13_0;                            // PB13 SCK - выход, альтернативная функция push-pull
	  GPIOB->CRH |= GPIO_CRH_CNF13_1;
	  GPIOB->CRH |= (GPIO_CRH_MODE13_0 | GPIO_CRH_MODE13_1);			// gpio speed 50 MHz
	
	  GPIOB->CRH &= ~GPIO_CRH_CNF8;                              // PB8 CS1 выход push-sull
	  GPIOB->CRH |= (GPIO_CRH_MODE8_0 | GPIO_CRH_MODE8_1);			// gpio speed 50 MHz
	
		GPIOB->CRH &= ~GPIO_CRH_CNF9;                              // PB9 CS2 выход push-sull
	  GPIOB->CRH |= (GPIO_CRH_MODE9_0 | GPIO_CRH_MODE9_1);			// gpio speed 50 MHz

	
	  GPIOB->CRH |= GPIO_CRH_CNF14_0;                              // PB14 MISO вход Input Floating 
	  GPIOB->CRH &= ~GPIO_CRH_MODE14;	
	
    SPI2->CR1 |= (SPI_CR1_BR_2 | SPI_CR1_BR_1 | SPI_CR1_BR_0);   // br = 111  F/256
		SPI2->CR1 |= SPI_CR1_DFF;                                    // 16 bit data
		SPI2->CR1 &= ~SPI_CR1_LSBFIRST;                              // младшим байтом вперед
		SPI2->CR1 |= (SPI_CR1_SSM | SPI_CR1_SSI);
		SPI2->CR1 |= SPI_CR1_MSTR;                                   // MASTER
		SPI2->CR1 |= SPI_CR1_SPE;                                    // SPI включить
}

int ReadTempBunker (void){
		
		float T;
		uint16_t T1;
		CS1_LOW
		SPI2->DR = 0;
		while (!(SPI2->SR & SPI_SR_RXNE));
		T1 = SPI2->DR;
		T = T1>>5;						// 3
		CS1_HIGH
		T = ((T == 0) || (T > 2000)) ? 0 : T*1;
		return ((int)T);
}
int ReadTempRiser (void){
		
		float T;
	  uint16_t T1;
		uint16_t T2;
		CS2_LOW	
		//while (!(SPI2->SR & SPI_SR_TXE))
	  SPI2->DR = 0;
    while (!(SPI2->SR & SPI_SR_RXNE));
		T1 = SPI2->DR;

	  SPI2->DR = 0;
		while (!(SPI2->SR & SPI_SR_RXNE));
	  T2 = SPI2->DR;
		T = T1>>4;						//2
	  CS2_HIGH
		T = ((T == 0) || (T > 4000)) ? 0 : T*1;
		return ((int)T);
}

void InitUSART1 (void){
	
	RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
	RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;    
  RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;
	
	GPIOA->CRH |= GPIO_CRH_CNF9_1;
	GPIOA->CRH |= GPIO_CRH_MODE9;
	
	GPIOA->CRH |= GPIO_CRH_CNF10_0;
	GPIOA->CRH &= ~GPIO_CRH_MODE10;
	
//	USART1->BRR = 0x1D4C;                      // 9600   468.75 F=72000000
	USART1->BRR = 0x271;											// 115200
	
	USART1->CR1 |= USART_CR1_TE;
	USART1->CR1 |= USART_CR1_RE;
	USART1->CR1 |= USART_CR1_UE;
	
	USART1->CR1 |= USART_CR1_RXNEIE;	
	HAL_NVIC_SetPriority(USART1_IRQn, 5, 0);
	NVIC_EnableIRQ(USART1_IRQn);
	
}

void InitUSART3 (void){
	
	RCC->APB1ENR |= RCC_APB1ENR_USART3EN;
	RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;    
  RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;
	
	GPIOB->CRH |= GPIO_CRH_CNF10_1;
	GPIOB->CRH |= GPIO_CRH_MODE10;
	
	GPIOB->CRH |= GPIO_CRH_CNF11_0;
	GPIOB->CRH &= ~GPIO_CRH_MODE11;
	
	USART3->BRR = 0x139;                      // 115200    19.53125     F=36000000
	
	
	USART3->CR1 |= USART_CR1_TE;
	USART3->CR1 |= USART_CR1_RE;
	USART3->CR1 |= USART_CR1_UE;
	
	USART3->CR1 |= USART_CR1_RXNEIE;	
	//NVIC_EnableIRQ(USART3_IRQn);
	
}

void SendUSART1 (char chr){

	while (!(USART1->SR & USART_SR_TC));
	USART1->DR = chr;
	
}

void SendStringUSART1 (char* str){
	
	uint8_t i = 0;
	
	while(str[i])
	SendUSART1 (str[i++]);
	
}

void SendDataUSART1 (uint8_t data){

	while (!(USART1->SR & USART_SR_TC));
	USART1->DR = data;
	
}


void SendUSART3 (char chr){
	
	while (!(USART3->SR & USART_SR_TC));
	USART3->DR = chr;
	
}

void SendStringUSART3 (char* str){
	
	uint8_t i = 0;
	
	while(str[i])
	SendUSART3 (str[i++]);
	
}

void SendDataUSART3 (uint8_t data){
	while (!(USART3->SR & USART_SR_TC));
	USART3->DR = data;
}

void myDelay(int32_t delay) {
	for(int i = 0; i < delay; i++) {
		for(int j = 1; j < 10000; j++) {
		}
	}
}
/* USER CODE END 4 */

 /**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM4 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM4) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
