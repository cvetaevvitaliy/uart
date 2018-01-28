#ifndef __VAPE_H
#define __VAPE_H
//------------------------------------------------------------
#include "stm32f1xx.h"
#include "stm32f1xx_hal.h"
#include <stdbool.h>

//------------------------------------------------------------

#define ledOn HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET)
#define ledOff HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET)

//------------------------------------------------------------
//void Set_Vape_PWM2(void);
void Set_Vape_PWM(uint8_t num_smell);


#endif /* __VAPE_H */
