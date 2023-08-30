/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */     
#include "ds18b20.h"
#include "queue.h"
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart3;
extern IWDG_HandleTypeDef hiwdg;
extern RTC_HandleTypeDef hrtc;
RTC_TimeTypeDef timeS;
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

float tempdeficite;
float RizerDiff;
float spid;
float Pod;									// pod
float PodOld = 0;
float dPod;
int Riser, RiserNotFiltred;
int RiserOld;
int dRiser;
int Bunker, BunkerNotFiltred;
int BunkerOld;
int dBunker;

int workservoprim, workservoprimOld;
float servoprim_speed = 0.25;
float Obr;									// obratka
int TempPodProm;					// temp prom
int TrebTempPod = 0;
int servosec, servosecOld;
int servoprim;
int preservoprim;
int D_preservoprim;
int P_preservoprim;
float Rashod;																// расход воды в системе (л/мин)
float Power;																// выдаваема€ мощность котла (¬т)
uint32_t worktime = 0;
uint8_t waterFlag = 0;
uint8_t bunkerFlag = 0;
uint8_t manualControl;
uint8_t	SOS;
//int BunkerNoFiltr, RiserNoFiltr;
char str[30];
char * pstr = &str[0];
uint8_t runTask1, runTask2, runTask3, runTask4, runTask5;

struct {
char Pod[4];
char Obr[4];
char Riser[4];
char Bunker[4];
char Rashod[4];
char Power[5];
char i1[3];
char i2[3];
char TrebTempPod[2];	
char strtime[8];
char SOS[3];
} String;
char * pString = &String.Pod[0];

/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId vHandlerTaskHandle;
osThreadId vTaskOprosHandle;
osThreadId vTaskRasZaslHandle;
osThreadId vTaskVodaHandle;
osThreadId vTaskTimeHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
   xQueueHandle FromNextion;
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void Handler(void const * argument);
void Opros(void const * argument);
void Zasl(void const * argument);
void Voda(void const * argument);
void Time(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];
  
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}                   
/* USER CODE END GET_IDLE_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
  
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  FromNextion = xQueueCreate(1, sizeof(uint8_t));
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of vHandlerTask */
  osThreadDef(vHandlerTask, Handler, osPriorityLow, 0, 256);
  vHandlerTaskHandle = osThreadCreate(osThread(vHandlerTask), NULL);

  /* definition and creation of vTaskOpros */
  osThreadDef(vTaskOpros, Opros, osPriorityLow, 0, 512);
  vTaskOprosHandle = osThreadCreate(osThread(vTaskOpros), NULL);

  /* definition and creation of vTaskRasZasl */
  osThreadDef(vTaskRasZasl, Zasl, osPriorityLow, 0, 128);
  vTaskRasZaslHandle = osThreadCreate(osThread(vTaskRasZasl), NULL);

  /* definition and creation of vTaskVoda */
  osThreadDef(vTaskVoda, Voda, osPriorityLow, 0, 128);
  vTaskVodaHandle = osThreadCreate(osThread(vTaskVoda), NULL);

  /* definition and creation of vTaskTime */
  osThreadDef(vTaskTime, Time, osPriorityLow, 0, 128);
  vTaskTimeHandle = osThreadCreate(osThread(vTaskTime), NULL);

  /* USER CODE BEGIN RTOS_THREADS */

  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN StartDefaultTask */
	TrebTempPod = BKP->DR1;
	if (TrebTempPod == 0) {
		TrebTempPod = 65;
		BKP->DR1 = TrebTempPod;
	}
	int servoptimFiltr, servosecFiltr;
  /* Infinite loop */
  
  for(;;)
  {
		if(runTask1 && runTask2 && runTask4 && runTask5) { // runTask3
			HAL_IWDG_Refresh(&hiwdg);
			runTask1 = 0;
			runTask2 = 0;
			runTask3 = 0;
			runTask4 = 0;
			runTask5 = 0;
		}
//		Bunker = (int)((1-0.3) * Bunker + 0.3 * BunkerNoFiltr);
		osDelay(SECOND);
		if(manualControl == 0) {
			servoptimFiltr = (int)((1-0.3) * servoptimFiltr + 0.3 * workservoprim);
			setServo(1, servoptimFiltr);
			osDelay(SECOND);
			servosecFiltr = (int)((1-0.3) * servosecFiltr + 0.3 * servosec);
			setServo(2, servosecFiltr);
		}
	}
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_Handler */
/**
* @brief Function implementing the vHandlerTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Handler */
void Handler(void const * argument)
{
  /* USER CODE BEGIN Handler */
	uint8_t comand;
  /* Infinite loop */
  for(;;)
  {
		if (uxQueueMessagesWaiting(FromNextion) != 0){
			xQueueReceive(FromNextion, &comand, 1);
			if(comand == 0xAA) BKP->DR1 = ++TrebTempPod;
			if(comand == 0xCC) BKP->DR1 = --TrebTempPod;
			if(comand == 0xBB) {
//				worktime = 0;
				timeS.Hours = 0;
				timeS.Minutes = 0;
				timeS.Seconds = 0;
				HAL_RTC_SetTime(&hrtc, &timeS, RTC_FORMAT_BIN);
				BKP->DR2 = 1; 																			// set combustion mode to 1
			} 
			if(comand == 0xDD) manualControl = 1;
			if(comand == 0xEE) manualControl = 0;
			if((comand == 0xE1) && (manualControl == 1)) {
				workservoprim += 10;
				workservoprim = constrain(workservoprim, 0, 180);
				setServo(1, workservoprim);
			}
			if((comand == 0xE2) && (manualControl == 1)) {
				workservoprim -= 10;
				workservoprim = constrain(workservoprim, 0, 180);
				setServo(1, workservoprim);
			}
			if((comand == 0xE3) && (manualControl == 1)) {
				servosec += 10;
				servosec = constrain(servosec, 0, 180);
				setServo(2, servosec);
			}
			if((comand == 0xE4) && (manualControl == 1)) {
				servosec -= 10;
				servosec = constrain(servosec, 0, 180);
				setServo(2, servosec);
			}
		}
		runTask1 = 1;
    osDelay(100);
  }
  /* USER CODE END Handler */
}

/* USER CODE BEGIN Header_Opros */
/**
* @brief Function implementing the vTaskOpros thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Opros */
void Opros(void const * argument)
{
  /* USER CODE BEGIN Opros */
	Ds18b20_Init(osPriorityRealtime);
	uint8_t nasosFlag = 1;
	NASOS_ON;
	float tmp;
  /* Infinite loop */
  for(;;)
  {
		
//		Pod = (!ds18b20[0].DataIsValid) ? 0 : ds18b20[0].Temperature;             	// get t from sensor 1
//		Obr = (!ds18b20[1].DataIsValid) ? 0 : ds18b20[1].Temperature;             	// get t from sensor 2
		
   	tmp = (!ds18b20[0].DataIsValid) ? 0 : ds18b20[0].Temperature;             	// get t from sensor 1
		Pod = (tmp == 0) ? Pod : tmp;
    tmp = (!ds18b20[1].DataIsValid) ? 0 : ds18b20[1].Temperature;             	// get t from sensor 2
		Obr = (tmp == 0) ? Obr : tmp;
		
		if((Pod >= 1) && (Pod <= 60) && (nasosFlag == 1)) {
			NASOS_OFF;
			nasosFlag = 0;
		}
		if((Pod >= 1) && (Pod >= 61) && (nasosFlag == 0)) {
			NASOS_ON;
			nasosFlag = 1;
		}
		if(Pod < 1) {
			NASOS_ON;
			nasosFlag = 1;
		}
		RiserNotFiltred = ReadTempRiser();
		Riser = (int)((1-0.3) * Riser + 0.3 * RiserNotFiltred);
		GPIOC->ODR ^= GPIO_ODR_ODR13;
		osDelay(SECOND);  
		
	  BunkerNotFiltred = ReadTempBunker();
		Bunker = (int)((1-0.3) * Bunker + 0.3 * BunkerNotFiltred);
		GPIOC->ODR ^= GPIO_ODR_ODR13;
		osDelay(SECOND);
		
//		Power = 4.2 * Rashod * 60 * (Pod - Obr);
		Power = (Pod - Obr) * Rashod * 60 * 1.16 / 1000;
	
		runTask2 = 1;
  }
  /* USER CODE END Opros */
}

/* USER CODE BEGIN Header_Zasl */
/**
* @brief Function implementing the vTaskRasZasl thread.
* @param argument: Not used
* @retval None
*/

/* USER CODE END Header_Zasl */
void Zasl(void const * argument)
{
  /* USER CODE BEGIN Zasl */
	
  /* Infinite loop */
  for(;;)
  {
		if(manualControl == 0) {
			
			dRiser = (RiserOld == 0) ? 0 : (Riser - RiserOld);
			RiserOld = Riser;
			P_preservoprim = workservoprim;
			
			calcPrimary();
			calcSecondary();
			
			osDelay(SECOND * 5);
			
			USART1->CR1 &= ~USART_CR1_TE;												// выключаем передатчик USART
			USART1->CR1 |= USART_CR1_TE;												// включаем передатчик USART
		}
		else {
			osDelay(SECOND);
		}
		runTask3 = 1;
  }
  /* USER CODE END Zasl */
}

/* USER CODE BEGIN Header_Voda */
/**
* @brief Function implementing the vTaskVoda thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Voda */
void Voda(void const * argument)
{
  /* USER CODE BEGIN Voda */
	float tic;
  /* Infinite loop */
  for(;;)
  {
		TIM2->CR1 |= TIM_CR1_CEN;
		osDelay(SECOND);
		TIM2->CR1 &= ~TIM_CR1_CEN;
		tic = TIM2->CNT;
		Rashod = tic / 5.5;
		TIM2->CNT = 0;
		runTask4 = 1;
  }
  /* USER CODE END Voda */
}

/* USER CODE BEGIN Header_Time */
/**
* @brief Function implementing the vTaskTime thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Time */
void Time(void const * argument)
{
  /* USER CODE BEGIN Time */
	uint32_t worktimeOld;
  /* Infinite loop */
  for(;;)
  {
		HAL_RTC_GetTime(&hrtc, &timeS, RTC_FORMAT_BIN);
		worktime = (uint32_t)(((uint32_t)timeS.Hours * 3600U) + \
                          ((uint32_t)timeS.Minutes * 60U) + \
                          ((uint32_t)timeS.Seconds));
		if((worktime != worktimeOld)) {
			inNextion();
		}
		if(((worktime % 60) == 0) && (worktime != worktimeOld)) {
			dPod = (PodOld == 0) ? 0 : (Pod - PodOld);
      PodOld = Pod;
			waterFlag = 1;
			BunkerStatus();
		}
		if(((worktime % 300) == 0) && (worktime != worktimeOld)) {
			dBunker = (BunkerOld == 0) ? 0 : (Bunker - BunkerOld);
      BunkerOld = Bunker;
			bunkerFlag = 1;
		}
		if(((worktime % 10) == 0) && (worktime != worktimeOld)) {
			inESP();
		}
		worktimeOld = worktime;
		runTask5 = 1;
		osDelay(SECOND);
  }
  /* USER CODE END Time */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void inNextion (void) {
	uint8_t i = 0;
	
	sprintf(String.strtime,"%02d:%02d:%02d", timeS.Hours, timeS.Minutes, timeS.Seconds);

	sprintf(str, "page0.t8.txt=\"%.1f\"€€€", Pod);
	SendStringUSART1(str);
	for (i = 20; i < 30; i++) str[i] = '4';
	
	sprintf(str, "page0.t9.txt=\"%.1f\"€€€", Obr);
	SendStringUSART1(str);
	for (i = 20; i < 30; i++) str[i] = '4';
	
	sprintf(str, "page0.t10.txt=\"%d\"€€€", Riser);
	SendStringUSART1(str);
	for (i = 20; i < 30; i++) str[i] = '4';
	
	sprintf(str, "page0.t11.txt=\"%d\"€€€", Bunker);
	SendStringUSART1(str);
	for (i = 20; i < 30; i++) str[i] = '4';
	
	sprintf(str, "page0.t13.txt=\"%.1f\"€€€", Rashod);
	SendStringUSART1(str);
	for (i = 20; i < 30; i++) str[i] = '4';
	
	sprintf(str, "page0.t12.txt=\"%.1f\"€€€", Power);
	SendStringUSART1(str);
	for (i = 20; i < 30; i++) str[i] = '4';
	
	sprintf(str, "page0.t15.txt=\"%3d\"€€€", workservoprim);
	SendStringUSART1(str);
	for (i = 20; i < 30; i++) str[i] = '4';
	
	sprintf(str, "page0.t17.txt=\"%3d\"€€€", servosec);
	SendStringUSART1(str);
	for (i = 20; i < 30; i++) str[i] = '4';
	
	sprintf(str, "page1.t2.txt=\"%3d\"€€€", workservoprim);
	SendStringUSART1(str);
	for (i = 20; i < 30; i++) str[i] = '4';
	
	sprintf(str, "page1.t3.txt=\"%3d\"€€€", servosec);
	SendStringUSART1(str);
	for (i = 20; i < 30; i++) str[i] = '4';
	
	sprintf(str, "page1.t1.txt=\"%2d\"€€€", TrebTempPod);
	SendStringUSART1(str);
	for (i = 20; i < 30; i++) str[i] = '4';
	
	sprintf(str, "page0.t1.txt=\"%s\"€€€", String.strtime);
	SendStringUSART1(str);
	for (i = 20; i < 30; i++) str[i] = '4';
	
}

/******************************************************************************** 80293381743 slava*/
void inESP (void){
	uint8_t mask[1];
	
	mask[0] = 'W';
	if(SOS == 1) {
		String.SOS[0] = 'S';
		String.SOS[1] = 'O';
		String.SOS[2] = 'S';
	}
	else {
		String.SOS[0] = '0';
		String.SOS[1] = '0';
		String.SOS[2] = '0';
	}
	
	taskENTER_CRITICAL();
	sprintf(String.Pod, "%.1f", Pod);
	sprintf(String.Obr, "%.1f", Obr);		
	sprintf(String.Riser, "%d", Riser);
	sprintf(String.Bunker, "%d", Bunker);
	sprintf(String.Rashod, "%.1f", Rashod);
	sprintf(String.Power, "%.1f", Power);
	taskEXIT_CRITICAL();
	
	HAL_UART_Transmit(&huart3, mask, 1, 50); 
	HAL_UART_Transmit(&huart3, (uint8_t*)String.Pod, 4, 50);
	HAL_UART_Transmit(&huart3, (uint8_t*)String.Obr, 4, 50);
	HAL_UART_Transmit(&huart3, (uint8_t*)String.Riser, 4, 50);
	HAL_UART_Transmit(&huart3, (uint8_t*)String.Bunker, 4, 50);
	HAL_UART_Transmit(&huart3, (uint8_t*)String.Rashod, 4, 50);
	HAL_UART_Transmit(&huart3, (uint8_t*)String.Power, 5, 50);
	HAL_UART_Transmit(&huart3, (uint8_t*)String.strtime, 8, 50);
	HAL_UART_Transmit(&huart3, (uint8_t*)String.SOS, 3, 50);
	mask[0] = 'Q';
	HAL_UART_Transmit(&huart3, mask, 1, 50);
}

uint8_t BunkerStatus (void){
//	SOS = ((worktime > 14400) && (Bunker < 480) && (Bunker < BunkerOld) && (Riser/Bunker < 1.04)) ? 1 : 0;
	SOS = 0;
	return SOS;
}

void calcPrimary(void) {
uint32_t mode;
	mode = BKP->DR2;
	workservoprim = BKP->DR3;
	if(mode == 0) mode = 1;
	
	if((dRiser > 15) && (Riser > 680)) {
		workservoprim = 0;
		setServo(1, workservoprim);
		osDelay(20 * SECOND);
		return;
	}
	
	switch(mode) {
		case 1: {																																	// sushka
			workservoprim = (Riser < 900) ? 180 : 0;
			if(dBunker > 1) {
				BKP->DR2 = 2;
				workservoprim = 90;
				setServo(1, workservoprim);
				osDelay(20 * SECOND);
			}
			break;
		}
		case 2: {																																		// razgon do 700	
			if(dRiser >= 8) workservoprim -= dRiser * 3;
			if(dRiser < 0) workservoprim -= dRiser * 3;
			if(dRiser == 0) workservoprim += 5;
			if(Riser >= 750) BKP->DR2 = 3;
			break;
		}
		case 3: {																																	// piroliz
			if((Riser > 700) && (Riser < 850)) workservoprim -= dRiser * 3;
			if(Riser < 700) BKP->DR2 = 2;
			if(Riser > 850) workservoprim = 0;
			break;
		}
	}
	workservoprim = (int)constrain(workservoprim, 0, 180);
	BKP->DR3 = workservoprim;
}

void calcSecondary(void) {
	
//	if (RiserOld < Riser)		RizerDiff = RizerDiff + (Riser - RiserOld);
//	if (RiserOld > Riser)		RizerDiff = RizerDiff - (RiserOld - Riser);
//	RizerDiff = constrain (RizerDiff, 2, 180);

	if (Riser <= 450) 	servosec = 0;
	if (Riser > 450 && Riser <= 500) servosec = 60;
	if (Riser > 500 && Riser <= 600) servosec = 100;
	if (Riser > 600 && Riser <= 700) servosec = 150;
	if (Riser > 700) servosec = 180;
	
//	if (Riser > 450 && Riser <= 500) {
//		servosec = (map ( RizerDiff, 2, 180, 5, 180) + map ( Riser, 450, 500, 5, 80)) / 2;
//	}
//	if (Riser > 500 && Riser <= 600) {
//		servosec = (map ( RizerDiff, 2, 180, 60, 180) + map ( Riser, 500, 600, 60, 120)) / 2;
//	}
//	if (Riser > 600 && Riser <= 800) {
//		servosec = (map ( RizerDiff, 2, 180, 100, 180) + map ( Riser, 600, 800, 100, 160)) / 2;
//	}
//	if (Riser > 800) {
//		servosec = (map ( RizerDiff, 2, 180, 160, 180) + map ( Riser, 800, 1020, 160, 180)) / 2;
//	}

	servosec = constrain (servosec, 0, 180);
}

/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
