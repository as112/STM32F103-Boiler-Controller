#include "main.h"

#define Relay_Port        	GPIOC
#define Rel_Kol_Pin         GPIO_Pin_14
#define Rel_Kot_Pin         GPIO_Pin_15
#define NasosKol_OFF     	 	GPIO_WriteBit(Relay_Port, Rel_Kol_Pin, Bit_RESET)
#define NasosKol_ON      		GPIO_WriteBit(Relay_Port, Rel_Kol_Pin, Bit_SET)
#define NasosKot_OFF     	 	GPIO_WriteBit(Relay_Port, Rel_Kot_Pin, Bit_RESET)
#define NasosKot_ON      		GPIO_WriteBit(Relay_Port, Rel_Kot_Pin, Bit_SET)
// подключение на нормально замкнутые контакты

float TempKot, TempB, TempKoll, dT;

uint8_t flag = 0;
uint16_t count = 0;

int main(void)
{
	InitEncoder();
	pcf8574_bus_init(I2C1);
	hd44780_init();
	hd44780_backlight_set(1);
	if(!ds18b20_init())  {
		hd44780_goto_xy(0, 0);
		hd44780_printf("NO ds18b20");
		NasosKol_ON;
		NasosKot_ON;
		}
  gpio_PortClockStart(Relay_Port);
  gpio_SetGPIOmode_Out(Relay_Port, Rel_Kol_Pin);
	gpio_SetGPIOmode_Out(Relay_Port, Rel_Kot_Pin);
	GPIO_WriteBit(Relay_Port, GPIO_Pin_13, Bit_SET);
	NasosKol_OFF;
	delay_ms(1000);
	iwdgInit();
		
  while (1)
  {
		Encoder();
		Relay();
		count++;
		if(count >= 7200) NVIC_SystemReset();		// Reset every 4 hours
		IWDG_ReloadCounter();
  }
}

void Encoder (void){
uint8_t LedFlag;
		
/********************************** опрос потенциометра ****************************/
		
//		if (TIM3->CNT > 500)      
//		{
//			SetTemp++;
//			TIM3->CNT = 500;
//		}
//		if (TIM3->CNT < 500)
//		{
//			SetTemp--;
//			TIM3->CNT = 500;
//		}
		
/********************************** опрос кнопки ************************************/
		if ((GPIOA->IDR & GPIO_IDR_IDR0) == 0){
		delay_ms(100);
		
		if ((GPIOC->IDR & GPIO_IDR_IDR13) == 0) {
			LedFlag = 1;
			}
		else {
			LedFlag = 0;
			hd44780_clear();
			}
		hd44780_backlight_set(LedFlag);
		}
	delay_ms(100);

}

void Relay (void){

	GPIOC->ODR ^= GPIO_ODR_ODR13;
		ds18b20_start_convert();
		delay_ms(1000);
    TempKot = ds18b20_get_temp(2);            	//get t from sensor 1
    TempB = ds18b20_get_temp(1);           		  //get t from sensor 2
		TempKoll = ds18b20_get_temp(0);             //get t from sensor 3
		dT = TempKoll - TempB;

/*************************************** вывод на дисплей *********************************/
		hd44780_goto_xy(0, 0);
		hd44780_printf("K= %.1f%s%.1f", TempKot,"  B= ", TempB);
    hd44780_goto_xy(1, 0);
    hd44780_printf("KOLL= %.1f /%.2f", TempKoll, dT); 
		
/********************************** алгоритм насоса коллектора ****************************/
			
		if((dT <= 2) && (flag == 1)) 	{	
			NasosKol_OFF;	
			flag = 0;
		}
		if((dT > 3) && (TempKoll > 30) && (flag == 0))	{
			NasosKol_ON;
			flag = 1;
		}
		delay_ms(500);

/********************************** алгоритм насоса котла *********************************/
		if(TempKot >= 1) {
			if(TempKot <= 60)		NasosKot_OFF;	
			if(TempKot >= 61)		NasosKot_ON;	
			delay_ms(500);
		}
		else {
			NasosKot_ON;
		}
}

//void TIM4_IRQHandler(void)
//{
//	TIM4->SR &= ~TIM_SR_UIF; 					// сбрасываем флаг прерывани¤
//	GPIOC->ODR ^= GPIO_ODR_ODR14;
//}	

void InitEncoder (void) {

	RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;							// Enable clock port A
	RCC->APB2ENR |= RCC_APB2ENR_IOPCEN;							// Enable clock port C
	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;							// Enable clock TIM3

	GPIOA->CRL &= ~(GPIO_CRL_CNF6_0 | GPIO_CRL_CNF7_0);			// Setting PA6 and PA7 input pull-up
	GPIOA->CRL |= GPIO_CRL_CNF6_1 | GPIO_CRL_CNF7_1;
	GPIOA->ODR |= GPIO_ODR_ODR6 | GPIO_ODR_ODR7;
	
	GPIOA->CRL &= ~(GPIO_CRL_CNF0_0);			// Setting button PA0 input pull-up
	GPIOA->CRL |= GPIO_CRL_CNF0_1;
	GPIOA->ODR |= GPIO_ODR_ODR0;

	GPIOC->CRH &= ~GPIO_CRH_CNF13;                              // LED
	GPIOC->CRH |= (GPIO_CRH_MODE13_0 | GPIO_CRH_MODE13_1);			// gpio speed 50 MHz
  	
	TIM3->CCMR1 |= TIM_CCMR1_IC1F | TIM_CCMR1_IC2F;				// Set filter sampling
	TIM3->CCMR1 |= TIM_CCMR1_CC1S_0 | TIM_CCMR1_CC2S_0;			// Enable TI1 and TI2
	TIM3->CCER  &= ~(TIM_CCER_CC1P | TIM_CCER_CC2P); 			// Polarity front signal
	TIM3->SMCR  |= TIM_SMCR_SMS_0 | TIM_SMCR_SMS_1;				// Counter on both channel

	TIM3->ARR = 1000;														// Period
	TIM3->CR1 |= TIM_CR1_CEN; 									// Enable counter

	TIM3->CNT = 500;
	
	/****************************************** TIM4 ********************************************/	

//	RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;
//		
//  TIM4->PSC  = 64000 - 1;						// новая частота = 1.125 kHz
//  TIM4->ARR  = 22500;								// период = 20 sec
//  TIM4->CR1 &= ~TIM_CR1_DIR;				// считаем вверх
//  TIM4->DIER  |= TIM_DIER_UIE; 			// разрешить прерывания по переполнению
//	NVIC_EnableIRQ(TIM4_IRQn);				// включить прерывания

}

void iwdgInit(void)
{
  RCC_LSICmd(ENABLE);
  while (RCC_GetFlagStatus(RCC_FLAG_LSIRDY) == RESET) {}
  IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);
  IWDG_SetPrescaler(IWDG_Prescaler_64); 										// 4, 8, 16 ... 256
  IWDG_SetReload(0x0FFF);																		//This parameter must be a number between 0 and 0x0FFF.
  IWDG_ReloadCounter();
  IWDG_Enable();
}
