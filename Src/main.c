/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"

/* USER CODE BEGIN Includes */
#include "time.h"
#include "vape.h"
#include <stdbool.h>
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;
DMA_HandleTypeDef hdma_usart3_rx;
DMA_HandleTypeDef hdma_usart3_tx;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
uint8_t flag_sys;
uint16_t delay_ms;
uint32_t delay_temp;
uint8_t delay_temp2=0;
uint8_t delay_temp3=0;
uint8_t delay_temp4=0;
uint32_t time11=0;
uint8_t rBuffer[32]={0};
//uint8_t i =1;
uint8_t str[]={0};
uint8_t str2[4];
uint8_t buf[4];
uint8_t buf_vape[3];
uint8_t buf_healt[3];
uint8_t buf_coller[3];
uint8_t buf_water[3];
uint8_t buf_vibro[3];
uint8_t i=0;
int temp_t=0;
uint8_t timeoff_vape[10]={0};
uint32_t time_vape1,time_vape2,time_vape3,time_vape4,time_vape5,time_vape6,time_vape7,time_vape8,time_vape9=0;
uint32_t time_vibro=0;
uint32_t time_vibroOff=0;
uint32_t time_healt_left=0;
uint32_t time_haelt_right=0;
uint32_t time_haelt_rightOff[2]={0};
uint32_t time_haelt_leftOff[2]={0};
uint32_t time_cooler_left_right=0;
uint32_t time_cooler_left_rightOff=0;
uint32_t time_water_left=0;
uint32_t time_water_right=0;
uint32_t time_water_Off=0;

bool vibro=false;
bool healt_left=false;
bool healt_right=false;
bool healt_left_right=false;
bool healt_rightON=false;
bool cooler_left_right=false;

bool water_left=false;
bool water_right=false;
bool vapeON=false;
bool vape1,vape2,vape3,vape4,vape5,vape6,vape7,vape8,vape9=false;
bool time_off=false;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART3_UART_Init(void);                                    
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
                                
                                
                                
                                

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USART3_UART_Init();

  /* USER CODE BEGIN 2 */
delay_temp=HAL_GetTick();
time11=HAL_GetTick();
delay_temp3=HAL_GetTick();
HAL_TIM_PWM_Start_IT(&htim1, TIM_CHANNEL_1);
HAL_TIM_PWM_Start_IT(&htim1, TIM_CHANNEL_2); 
HAL_TIM_PWM_Start_IT(&htim1, TIM_CHANNEL_3); 
HAL_TIM_PWM_Start_IT(&htim2, TIM_CHANNEL_1);
HAL_TIM_PWM_Start_IT(&htim2, TIM_CHANNEL_2); 
HAL_TIM_PWM_Start_IT(&htim2, TIM_CHANNEL_3);
HAL_TIM_PWM_Start_IT(&htim3, TIM_CHANNEL_1);
HAL_TIM_PWM_Start_IT(&htim3, TIM_CHANNEL_2); 
HAL_TIM_PWM_Start_IT(&htim3, TIM_CHANNEL_3);
HAL_TIM_PWM_Start_IT(&htim4, TIM_CHANNEL_1);
HAL_TIM_PWM_Start_IT(&htim4, TIM_CHANNEL_2); 
HAL_TIM_PWM_Start_IT(&htim4, TIM_CHANNEL_3);
HAL_TIM_PWM_Start_IT(&htim4, TIM_CHANNEL_4);
//HAL_UART_Receive_IT(&huart2, rBuffer, 32, 0x0d, 0x0a, 1);
//HAL_UART_Receive_IT(&huart2, rBuffer, 32, 0, 0, 0);
TIM1->CCR1=0;
TIM1->CCR2=0;
TIM1->CCR3=0;
TIM2->CCR1=0;
TIM2->CCR2=0;
TIM2->CCR3=0;
TIM3->CCR1=0;
TIM3->CCR2=0;
TIM3->CCR3=0;
TIM4->CCR1=0;
TIM4->CCR2=0;
TIM4->CCR3=0;
TIM4->CCR4=0;

	//Set_Vape_PWM(i);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
		//HAL_Delay(1000);
		time11=HAL_GetTick();
/* if (HAL_GetTick()-delay_temp>3000){
        HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_5);
	 //Set_Vape_PWM(1);
				delay_temp2++;
        delay_temp=HAL_GetTick();} 
 
	*/			
		HAL_UART_Receive_DMA(&huart2, str,1);
		if (str[0]!=0){
			if(str[0]==0x0A){i=0;}else{
			str2[i]=str[0];
			i++;
			str[0]=0;
			if(i>=3)i=0;
				
		}}
		
		if(str2[0]!=0&&str2[1]!=0&&str2[2]!=0){
		if(str2[0]=='5'){buf_vape[0]=str2[0];buf_vape[1]=str2[1];buf_vape[2]=str2[2];str2[0]=0;str2[1]=0;str2[2]=0;}
		if(str2[0]=='1'){buf_healt[0]=str2[0];buf_healt[1]=str2[1];buf_healt[2]=str2[2];str2[0]=0;str2[1]=0;str2[2]=0;}
		if(str2[0]=='2'){buf_healt[0]=str2[0];buf_healt[1]=str2[1];buf_healt[2]=str2[2];str2[0]=0;str2[1]=0;str2[2]=0;}
		if(str2[0]=='3'){buf_healt[0]=str2[0];buf_healt[1]=str2[1];buf_healt[2]=str2[2];str2[0]=0;str2[1]=0;str2[2]=0;}
		if(str2[0]=='4'){buf_coller[0]=str2[0];buf_coller[1]=str2[1];buf_coller[2]=str2[2];str2[0]=0;str2[1]=0;str2[2]=0;}
		if(str2[0]=='6'){buf_water[0]=str2[0];buf_water[1]=str2[1];buf_water[2]=str2[2];str2[0]=0;str2[1]=0;str2[2]=0;}
		if(str2[0]=='7'){buf_vibro[0]=str2[0];buf_vibro[1]=str2[1];buf_vibro[2]=str2[2];str2[0]=0;str2[1]=0;str2[2]=0;}
		else {str2[0]=0;str2[1]=0;str2[2]=0;}				
	}	
		
  if(buf_vape[0]=='5'){vapeON=false;timeoff_vape[0]=buf_vape[2];buf_vape[2]=0;
	if(buf_vape[1]=='1'&&vape1==false)
	{Set_Vape_PWM(1);HAL_GPIO_WritePin(GPIOC,GPIO_PIN_1, GPIO_PIN_SET);vape1=true;time_vape1=HAL_GetTick();buf_vape[1]=0;}
	if(buf_vape[1]=='2'&&vape2==false)
	{Set_Vape_PWM(2);HAL_GPIO_WritePin(GPIOC,GPIO_PIN_1, GPIO_PIN_SET);vape2=true;time_vape2=HAL_GetTick();buf_vape[1]=0;}
	if(buf_vape[1]=='3'&&vape3==false)
	{Set_Vape_PWM(3);vape3=true;time_vape3=HAL_GetTick();buf_vape[1]=0;}
	if(buf_vape[1]=='4'&&vape4==false)
	{Set_Vape_PWM(4);vape4=true;time_vape4=HAL_GetTick();buf_vape[1]=0;}
	if(buf_vape[1]=='5'&&vape5==false)
	{Set_Vape_PWM(5);vape5=true;time_vape5=HAL_GetTick();buf_vape[1]=0;}
	if(buf_vape[1]=='6'&&vape6==false)
	{Set_Vape_PWM(6);vape6=true;time_vape6=HAL_GetTick();buf_vape[1]=0;}
	if(buf_vape[1]=='7'&&vape7==false)
	{Set_Vape_PWM(7);vape7=true;time_vape7=HAL_GetTick();buf_vape[1]=0;}
	if(buf_vape[1]=='8'&&vape8==false)
	{Set_Vape_PWM(8);vape8=true;time_vape8=HAL_GetTick();buf_vape[1]=0;}
	if(buf_vape[1]=='9'&&vape9==false)
	{Set_Vape_PWM(9);vape9=true;time_vape9=HAL_GetTick();buf_vape[1]=0;}
	buf_vape[0]=0;
	
}
if(buf_healt[0]=='1')
{healt_right=true;
TIM4->CCR1=1500;
TIM4->CCR3=1500;
time_haelt_right=HAL_GetTick();
buf_healt[0]=0;
time_haelt_rightOff[0]=buf_healt[2];
buf_healt[2]=0; }	


if(buf_healt[0]=='2')
{healt_left=true;
TIM4->CCR2=1500;
TIM4->CCR4=1500;
time_healt_left=HAL_GetTick();
buf_healt[0]=0;
time_haelt_leftOff[0]=buf_healt[2];
buf_healt[2]=0; }	

if(buf_healt[0]=='3')
{healt_left_right=true;
TIM4->CCR2=1500;
TIM4->CCR4=1500;
TIM4->CCR1=1500;
TIM4->CCR3=1500;
time_healt_left=HAL_GetTick();
buf_healt[0]=0;
time_haelt_leftOff[0]=buf_healt[2];
buf_healt[2]=0; }	



if(buf_coller[0]=='4')
{cooler_left_right=true;
TIM4->CCR3=1500;
TIM4->CCR4=1500;
time_cooler_left_right=HAL_GetTick();
buf_coller[0]=0;
time_cooler_left_rightOff=buf_coller[2];
buf_coller[2]=0; }	


if(buf_water[0]=='6')
{water_left=true;
HAL_GPIO_WritePin(GPIOB,GPIO_PIN_13, GPIO_PIN_SET);
HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12, GPIO_PIN_SET);
time_water_left=HAL_GetTick();
buf_water[0]=0;
time_water_Off=buf_water[2];
buf_water[2]=0; }	

if(buf_vibro[0]=='7')
{vibro=true;
HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14, GPIO_PIN_SET);

time_vibro=HAL_GetTick();
buf_vibro[0]=0;
time_vibroOff = buf_vibro[2];
buf_vibro[2]=0; }	

		
	
		if(vape1==true&&time11-time_vape1>(timeoff_vape[0]- 0x30) * 1000){TIM1->CCR1=0; vape1=false; time_vape1=HAL_GetTick();HAL_GPIO_WritePin(GPIOC,GPIO_PIN_1, GPIO_PIN_RESET);}
		if(vape2==true&&time11-time_vape2>(timeoff_vape[0]- 0x30) * 1000){TIM1->CCR2=0; vape2=false; time_vape2=HAL_GetTick();HAL_GPIO_WritePin(GPIOC,GPIO_PIN_1, GPIO_PIN_RESET);}
		if(vape3==true&&time11-time_vape3>(timeoff_vape[0]- 0x30) * 1000){TIM1->CCR3=0; vape3=false; time_vape3=HAL_GetTick();HAL_GPIO_WritePin(GPIOC,GPIO_PIN_1, GPIO_PIN_RESET);}
		if(vape4==true&&time11-time_vape4>(timeoff_vape[0]- 0x30) * 1000){TIM2->CCR1=0; vape4=false; time_vape4=HAL_GetTick();}
		if(vape5==true&&time11-time_vape5>(timeoff_vape[0]- 0x30) * 1000){TIM2->CCR2=0; vape5=false; time_vape5=HAL_GetTick();}
		if(vape6==true&&time11-time_vape6>(timeoff_vape[0]- 0x30) * 1000){TIM2->CCR3=0; vape6=false; time_vape6=HAL_GetTick();}
		if(vape7==true&&time11-time_vape7>(timeoff_vape[0]- 0x30) * 1000){TIM3->CCR1=0; vape7=false; time_vape7=HAL_GetTick();}
		if(vape8==true&&time11-time_vape8>(timeoff_vape[0]- 0x30) * 1000){TIM3->CCR2=0; vape8=false; time_vape8=HAL_GetTick();}
		if(vape9==true&&time11-time_vape9>(timeoff_vape[0]- 0x30) * 1000){TIM3->CCR3=0; vape9=false; time_vape8=HAL_GetTick();vapeON=false;}

	

if(healt_right==true&&time11-time_haelt_right>(time_haelt_rightOff[0]- 0x30) * 1000){TIM4->CCR1=0;TIM4->CCR3=0;  healt_right=false; time_haelt_right=HAL_GetTick();}

if(healt_left==true&&time11-time_healt_left>(time_haelt_leftOff[0]- 0x30) * 1000){TIM4->CCR2=0;TIM4->CCR4=0;  healt_left=false; time_healt_left=HAL_GetTick();}
if(healt_left_right==true&&time11-time_healt_left>(time_haelt_leftOff[0]- 0x30) * 1000){TIM4->CCR2=0;TIM4->CCR4=0;TIM4->CCR1=0;TIM4->CCR3=0;   healt_left_right=false; time_healt_left=HAL_GetTick();}

if(cooler_left_right==true&&time11-time_cooler_left_right>(time_cooler_left_rightOff- 0x30) * 1000){TIM4->CCR4=0;TIM4->CCR3=0;  cooler_left_right=false; time_cooler_left_right=HAL_GetTick();}

if(water_left==true&&time11-time_water_left>(time_water_Off- 0x30) * 1000){HAL_GPIO_WritePin(GPIOB,GPIO_PIN_13, GPIO_PIN_RESET);HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12, GPIO_PIN_RESET);  water_left=false; time_water_left=HAL_GetTick();}

	if(vibro==true&&time11-time_vibro>(time_vibroOff- 0x30) * 1000){HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14, GPIO_PIN_RESET); vibro=false; time_vibro=HAL_GetTick();}

	
//	str2[0]=0;
//	str2[1]=0;
//	str2[2]=0;
//	str2[3]=0;
	
}
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* TIM1 init function */
static void MX_TIM1_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig;

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 3600;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim1);

}

/* TIM2 init function */
static void MX_TIM2_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 3600;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim2);

}

/* TIM3 init function */
static void MX_TIM3_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 3600;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim3);

}

/* TIM4 init function */
static void MX_TIM4_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 3600;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim4);

}

/* USART2 init function */
static void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART3 init function */
static void MX_USART3_UART_Init(void)
{

  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);
  /* DMA1_Channel6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);
  /* DMA1_Channel7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel7_IRQn);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC1 PC2 PC3 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB12 PB13 PB14 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
