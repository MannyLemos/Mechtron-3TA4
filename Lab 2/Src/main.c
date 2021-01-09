/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/*	

*/
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32l476g_discovery.h"
#include "stm32l476g_discovery_glass_lcd.h"
#include "eeprom.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef enum{DEFAULT, PURGATORY, START} state_Track;		//States for Reaction time tester

/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
RNG_HandleTypeDef hrng;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;

/* USER CODE BEGIN PV */
// the emulated EEPROM can save 3 variables at these three addresses
uint16_t VirtAddVarTab[NB_OF_VAR] = {0x5555, 0x6666, 0x7777}; 
uint16_t counter = 0;
char lcd_buffer[6];
uint32_t rand_Num;
uint16_t time_Elapsed = 0;
uint16_t time_Elapsed_msec = 0;
uint16_t record_Time = 34463;
state_Track state = DEFAULT;

char time_Elapsed_String[6] = "000000";
char rand_Num_String[6] = "000000";
char record_Time_String[6] = "000000";

int sprintf(char *str, const char *format, ...);
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM2_Init(void);
static void MX_RNG_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{ 
	time_Elapsed++;
	if (state == PURGATORY){
		if (time_Elapsed >= rand_Num){
			time_Elapsed = 0;
			BSP_LCD_GLASS_Clear();
			state = START;
		}
	}else if(state == START){
		BSP_LED_On(LED4);
		BSP_LED_On(LED5);
		time_Elapsed_msec = 10*time_Elapsed;
		sprintf(time_Elapsed_String, "%u", time_Elapsed_msec);
		BSP_LCD_GLASS_DisplayString((uint8_t*)time_Elapsed_String);
	}
}

//Handles frequency change of LED4
void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef * htim) 
{
	if (state == DEFAULT){
		BSP_LED_Toggle(LED4);
		BSP_LED_Toggle(LED5);
	}else{
		BSP_LED_Off(LED4);
		BSP_LED_Off(LED5);
	}
	__HAL_TIM_SET_COUNTER(htim, 0x0000);
}
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

  /* Configure the system clock*/
  SystemClock_Config();
	
	HAL_InitTick(0x0000); //set the systick interrupt priority to the highest

	BSP_LED_Init(LED4);
	BSP_LED_Init(LED5);

	BSP_LCD_GLASS_Init();
	
	BSP_JOY_Init(JOY_MODE_EXTI);
  /* USER CODE BEGIN SysInit */
	
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM4_Init();
  MX_TIM2_Init();
  MX_RNG_Init();
  /* USER CODE BEGIN 2 */
	HAL_FLASH_Unlock(); //First unlock flash program erase controller
	
	//EEPROM INIT
	EE_Init();
	EE_WriteVariable(VirtAddVarTab[2],9999);
	EE_ReadVariable(VirtAddVarTab[2], &counter);
	sprintf(lcd_buffer,"%d", counter);
	
	BSP_LCD_GLASS_DisplayString((uint8_t*)"MT3TA4");
  /* USER CODE END 2 */

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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
	//SYSTEM CLOCK set to 4MHz
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RNG;
  PeriphClkInit.RngClockSelection = RCC_RNGCLKSOURCE_PLLSAI1;
  PeriphClkInit.PLLSAI1.PLLSAI1Source = RCC_PLLSOURCE_MSI;
  PeriphClkInit.PLLSAI1.PLLSAI1M = 1;
  PeriphClkInit.PLLSAI1.PLLSAI1N = 16;
  PeriphClkInit.PLLSAI1.PLLSAI1P = RCC_PLLP_DIV7;
  PeriphClkInit.PLLSAI1.PLLSAI1Q = RCC_PLLQ_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1R = RCC_PLLR_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_48M2CLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief RNG Initialization Function
  * @param None
  * @retval None
  */
static void MX_RNG_Init(void)
{

  /* USER CODE BEGIN RNG_Init 0 */

  /* USER CODE END RNG_Init 0 */

  /* USER CODE BEGIN RNG_Init 1 */

  /* USER CODE END RNG_Init 1 */
  hrng.Instance = RNG;
  if (HAL_RNG_Init(&hrng) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RNG_Init 2 */

  /* USER CODE END RNG_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 399;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 99;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */
	if(HAL_TIM_Base_Start_IT(&htim2) != HAL_OK){
		Error_Handler();
	}
  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 399;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 9999;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 9999;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  __HAL_TIM_ENABLE_OCxPRELOAD(&htim4, TIM_CHANNEL_2);
  /* USER CODE BEGIN TIM4_Init 2 */
	if (HAL_TIM_OC_Start_IT(&htim4, TIM_CHANNEL_2) != HAL_OK)
	{
		/*Starting Error*/
		Error_Handler();
	}
  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{	
	//set all joy pin interupts to highest peripheral interupt priority -> 0
	//Ensures Joy Pin interupts will be prioritized as timing is critical
	//in the application they are being used for
	HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
	HAL_NVIC_SetPendingIRQ(EXTI0_IRQn);
	
	HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
	HAL_NVIC_SetPendingIRQ(EXTI1_IRQn);
	
	HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
	HAL_NVIC_SetPendingIRQ(EXTI2_IRQn);
	
	HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
	HAL_NVIC_SetPendingIRQ(EXTI3_IRQn);
	
	HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
	HAL_NVIC_SetPendingIRQ(EXTI9_5_IRQn);
	
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LEDR_GPIO_Port, LEDR_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LEDG_GPIO_Port, LEDG_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LEDR_Pin */
  GPIO_InitStruct.Pin = LEDR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LEDR_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LEDG_Pin */
  GPIO_InitStruct.Pin = LEDG_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LEDG_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	switch (GPIO_Pin){
		//Right Joy Pin forces state default 
		//Restore MT3TA4 to screen
		//Acts like a home button exiting the game no matter the current conditions ie) state
		case (GPIO_PIN_2):
		{
			state = DEFAULT;
			BSP_LCD_GLASS_Clear();
			BSP_LCD_GLASS_DisplayString((uint8_t*)"MT3TA4");
			break;
		}
		//Joy Pins left, select, and down all operate in the same manner
		//all  trigger state = wait on first press
		//all trigger "CHEAT" if pressed during PURGATORY
		//all trigger the time to stop is pressed during START
		case (GPIO_PIN_0):
		case (GPIO_PIN_1):
		case (GPIO_PIN_5):
			{
				switch(state)
					{
					//DEFAULT state
					case (DEFAULT):
						{
						//aquire a true random value using HAL_RNG_GetRandomNumber(&hrng);
						rand_Num = HAL_RNG_GetRandomNumber(&hrng);
						//assign remainder from int division by 4000 to self -> divide by 10 for resolution deficiency
						rand_Num = (rand_Num % 4000)/10;
						BSP_LCD_GLASS_Clear();
						BSP_LCD_GLASS_DisplayString((uint8_t*)"WAIT");
						//turn off LEDS per requirement document
						BSP_LED_Off(LED4);
						BSP_LED_Off(LED5);
						//set time_Elapsed = 0 -> RNG wait time will be compared against time_Elapsed as it counts up 
						//until time_Elapsed >= RNG wait time
						time_Elapsed = 0;
						//State called "PURGATORY" is set so that cheat detection is enabled
						state = PURGATORY;
						break;
						}
					//Waiting Period
					case (PURGATORY):
						{
						//If button is pressed prior to time_Elapsed >= RNG Wait time (rand_Num)
						//message "CHEAT" is displayed and state DEFAULT is restored effectively terminating the turn
						BSP_LCD_GLASS_Clear();
						BSP_LCD_GLASS_DisplayString((uint8_t*)"CHEAT");
						state = DEFAULT;
						break;
						}
					//Game Begins
					case (START):
						{
							//read the current high score and save in record_Time
							EE_ReadVariable(VirtAddVarTab[0], &record_Time);
							//if time just scored is less than  record_Time (whih is preset to 
							if(time_Elapsed_msec < record_Time)
							{
							record_Time = time_Elapsed_msec;
							}
						EE_WriteVariable(VirtAddVarTab[0], record_Time);
						state = DEFAULT;
						break;
						}
				}
				break;
			}
		//UP 
		case (GPIO_PIN_3):
		{
			//states are tested on up joy pin interrupt so that high score is not displayed during a game 
			//also enables up joy pin to be used as a stopping interupt 
			//That is to say it will stop the clock for reaction time or display cheat if pressed during PURGATORY state
				switch(state)
				{
					//DEFAULT
					case (DEFAULT):
					{
						//When up Joy Pin is pressed current high score is read from emulated EEPROM 
						//We store current high score in first memory location [0]
						BSP_LCD_GLASS_Clear();
						EE_ReadVariable(VirtAddVarTab[0], &record_Time);
						sprintf(record_Time_String, "%u", record_Time);
						BSP_LCD_GLASS_Clear();
						BSP_LCD_GLASS_DisplayString((uint8_t*)record_Time_String);						
						break;
					}
					//if up pin is pressed during PURGATORY cheat is displayed
					//prevents high score from displaying during wait
					case (PURGATORY):
					{
						BSP_LCD_GLASS_Clear();
						BSP_LCD_GLASS_DisplayString((uint8_t*)"CHEAT");
						state = DEFAULT;
						break;
					}
					//if up pin is pressed during START timer is stopped and time is compared to highscore 
					//acts same as if left, down, or select was pressed
					//prevents high score from displaying during game
					case (START):
					{
						EE_ReadVariable(VirtAddVarTab[0], &record_Time);
						if(time_Elapsed_msec < record_Time || record_Time == 0x00)
							{
							record_Time = time_Elapsed_msec;
							}
						EE_WriteVariable(VirtAddVarTab[0], record_Time);
						state = DEFAULT;
						break;
					}
				}
				break;
		}
	}
}
	/*	
		//LEFT
		case (GPIO_PIN_1):{
				counter = 100;
				EE_WriteVariable(VirtAddVarTab[2], counter);
				sprintf(lcd_buffer, "%d", counter);
				BSP_LCD_GLASS_Clear();
				BSP_LCD_GLASS_DisplayString((uint8_t*)lcd_buffer);
				break;}
		
		//RIGHT
		case (GPIO_PIN_2):{
				counter = 200;
				EE_WriteVariable(VirtAddVarTab[2], counter);
				sprintf(lcd_buffer, "%d", counter);
				BSP_LCD_GLASS_Clear();
				BSP_LCD_GLASS_DisplayString((uint8_t*)lcd_buffer);
				break;}
		
		//DOWN
		case (GPIO_PIN_5):{
				BSP_LCD_GLASS_Clear();
				break;}
		
		default:{
			//default display 
				BSP_LCD_GLASS_Clear();
				BSP_LCD_GLASS_DisplayString((uint8_t*)"ERROR");
				break
*/
	
/* USER CODE END 4 */

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
