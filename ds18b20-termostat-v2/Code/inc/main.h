/*************************************************** Library ******************************************************************/

#include "stm32f10x.h"
#include "delay.h"
#include "gpio.h"
#include "hd44780.h"
#include "pcf8574.h"
#include "ds18b20.h"
#include "onewire.h"
#include "stm32f10x_iwdg.h"

void Encoder (void);
void Relay (void);
void InitEncoder (void);
void iwdgInit(void);
