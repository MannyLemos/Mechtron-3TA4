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
Marking Scheme Lab 1
pre) 	[20pts] 	Complete lab 1 tutorial [done]
11)		[5 pts]		Build project to verify it can still compile [done]
19)		[5 pts]		Build the project and download the binaries to the discovery board [done]
24)		[5 pts]		Build the project and download the binaries to the discovery board [done]

1.		[10 pts]	Use functions from BSP package to initiate and blink the 2 LEDS 
								same timers and timer configuration 
2.		[10 pts] 	Use TIM2 instead of TIM3. Use TIM4's channel 2 instead of channel 1
								to blink LED4 and LED5 [done]
3.		[15 pts]	When press the left, right, up and down button of the joystick, 
								display “LEFT”, “RIGHT”, “UP” and “DOWN” on LCD respectively. [done]
4.		[5 pts]		When the Selection button of the joystick is pressed, let  
								“mt3ta4 lab1 project” scroll across the LCD [done]
5.		[25 pts]	Also, when the Selection button of the joystick is pressed, toggle the blinking 
								frequencies for LED4 and LED5. (Pressing Selection once, makes them blink two times 
								slower than in tutorial. Pressing  Selection again makes blink at their original frequencies) 
*/
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

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

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;

/* USER CODE BEGIN PV */
//number of times select has been pressed
__O uint8_t select_Counter = 0;
//Counter which increases for the toggling of LED4
__O uint8_t LED4_Counter = 0;
//Counter which increases for the toggling of LED5
__O uint8_t LED5_Counter = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//handles frequency change of LED5
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{ 
	if ((*htim).Instance==TIM2)
		{
		if (select_Counter %2 == 0){ 
			//HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_8);
			BSP_LED_Toggle(LED5);
		}else{
			if(LED5_Counter %2 == 1)
				{
					BSP_LED_Toggle(LED5);
				}
			LED5_Counter++;
			}
		}
}

//Handles frequency change of LED4
void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef * htim) 
{
	//HAL_GPIO_TogglePin(LEDR_GPIO_Port, LEDR_Pin);
		if (select_Counter %2 == 0){ 
		BSP_LED_Toggle(LED4);
		__HAL_TIM_SET_COUNTER(htim, 0x0000);
		}else{
			if(LED4_Counter %2 == 1)
				{
					BSP_LED_Toggle(LED4);
					__HAL_TIM_SET_COUNTER(htim, 0x0000);
				}else{__HAL_TIM_SET_COUNTER(htim, 0x0000);}
			LED4_Counter++;
			}
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

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM4_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
	BSP_LCD_GLASS_Init();
	BSP_LED_Init(LED4);
	BSP_LED_Init(LED5);
	BSP_JOY_Init(JOY_MODE_EXTI);
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
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
  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
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
  htim2.Init.Period = 4999;
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
  htim4.Init.Period = 65535;
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
/*3.		[15 pts]	When press the left, right, up and down button of the joystick, 
									display “LEFT”, “RIGHT”, “UP” and “DOWN” on LCD respectively. [done]

A switch and several cases are used to detect which pin is providing the input
and then outputs to the LCD accordingly

Moving or pressing any direction of the joystick fires an interrupt. 
The direction pressed on the joystick will then be displayed on the LCD eg) "DOWN"*/
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	switch (GPIO_Pin){
		
		case (GPIO_PIN_0):{
		//pin 0 is triggered  by joy inwards and displays the scrolling message "MT3TA4 lab1 project"
				select_Counter++;
				BSP_LCD_GLASS_Clear();
				BSP_LCD_GLASS_ScrollSentence((uint8_t*)"MT3TA4 lab1 project ",1,300);
				break;}
		
		case (GPIO_PIN_1):{
			//pin 1 is triggered  by joy left and displays "LEFT"
				select_Counter++;
				BSP_LCD_GLASS_Clear();
				BSP_LCD_GLASS_DisplayString((uint8_t*)"LEFT");
				break;}
		
		case (GPIO_PIN_2):{
			//pin 2 is triggered  by joy right and displays "RIGHT"
				BSP_LCD_GLASS_Clear();
				BSP_LCD_GLASS_DisplayString((uint8_t*)"RIGHT");
				break;}
		
		case (GPIO_PIN_3):{
			//pin 3 is triggered  by joy up and displays "UP"
				BSP_LCD_GLASS_Clear();
				BSP_LCD_GLASS_DisplayString((uint8_t*)"UP");
				break;}
		
		case (GPIO_PIN_5):{
			//pin 5 is triggered  by joy down and displays "DOWN"
				BSP_LCD_GLASS_Clear();
				BSP_LCD_GLASS_DisplayString((uint8_t*)"DOWN");
				break;}
		
		default:{
			//default display 
				BSP_LCD_GLASS_Clear();
				BSP_LCD_GLASS_DisplayString((uint8_t*)"ERROR");
				break;}
	}
}


	
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
