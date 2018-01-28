#include "time.h"
#include "vape.h"
#include "stm32f1xx.h"

uint32_t time_temp=0;

void Time_set(uint8_t time_set){
	
time_temp=HAL_GetTick();
	if (HAL_GetTick()-time_temp>time_set*1000)
		{
        //HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_5);
				time_temp=HAL_GetTick();
				TIM1->CCR1=0;
		} 
	
}
