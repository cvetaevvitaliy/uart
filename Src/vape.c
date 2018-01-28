#include "vape.h"
#include "time.h"
#include <stdbool.h>
//#include "stdint.h"
//------------------------------------------------------------
//TIM_HandleTypeDef htim_1;
uint16_t pwm = 1200;


void Set_Vape_PWM(uint8_t num_smell)
{

//uint32_t delay_temp=HAL_GetTick();
switch (num_smell) {
	case 1: 
	//----------------
	TIM1->CCR1=pwm;

	break;
	case 2: 
	//-----

	TIM1->CCR2=pwm;
	break;
	case 3: 
	//-----

	TIM1->CCR3=pwm;
	break;
	case 4: 
	//-----

	TIM2->CCR1=pwm;
	break;
	case 5: 
	//-----
		TIM2->CCR2=pwm;
	break;
	case 6: 
	//-----
		TIM2->CCR3=pwm;
	break;
	case 7: 
	//-----
		TIM3->CCR1=pwm;
	break;
	case 8: 
	//-----
		TIM3->CCR2=pwm;
	break;
	case 9: 
	//-----
		TIM3->CCR3=pwm;
	break;

}
	


}

