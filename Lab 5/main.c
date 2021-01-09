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
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include "stm32l476g_discovery.h"
#include "stm32l476g_discovery_glass_lcd.h"
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
TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */
double factor = 1;	// speed factor
char LCD_BUFFER[6]; // LCD display buffer

uint16_t direction_Setting = 1;		// 1 = CW, 0 = CCW
uint16_t step_Setting = 1;				// 1 = full, 0 = half
uint16_t current_State = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//output to lcd
void displayTags(char *tag)
{
	BSP_LCD_GLASS_Clear();
	BSP_LCD_GLASS_DisplayString((uint8_t*) tag);
}

//joystick callback
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)		//Joy pin interupts
{
  switch (GPIO_Pin) {
			//select
			case SEL_JOY_PIN:
			{				
				//if full
				if (step_Setting == 1){
					//set half
					step_Setting = 0;
					//set period to half settings
					htim3.Instance->ARR =  htim3.Instance->ARR / 2;
					displayTags("HF STP");
				}else{
					//if half set full
					step_Setting = 1;
					//set period to full settings
					htim3.Instance->ARR =  htim3.Instance->ARR * 2;
					displayTags("FL STP");
				}
				break;	
			}
			
			//left
			case LEFT_JOY_PIN:  		
			{
				//if CW
				if (direction_Setting == 1){
					//set CCW
					direction_Setting = 0;
					displayTags("CCW");
				}else{
					//if CCW set CW
					direction_Setting = 1;
					displayTags("CW");
				}
				break;
			}
			
			//right -> does nothing
			case RIGHT_JOY_PIN:     
			{
				break;
			}
			
			//Up
			case UP_JOY_PIN:    			
			{
				BSP_LCD_GLASS_Clear();
				//upper limit of factor (prevent going to fast)
				if (factor < 4.9)
				{
					//increment factor
					factor = factor + 0.1;
				}
				//display factor
				sprintf(LCD_BUFFER, "%.1f", factor);
				BSP_LCD_GLASS_DisplayString((uint8_t*)LCD_BUFFER);
				//test if CCW or CW
				if(step_Setting == 1){
					htim3.Instance->ARR = 13125 / factor;		
				}else{
					htim3.Instance->ARR = 13125 / factor / 2;
				}
				break;
			}
			
			//down
			case DOWN_JOY_PIN:    			
			{
				BSP_LCD_GLASS_Clear();
				//lower limit of factor 
				//spazzes out below 0.3
				if (factor > 0.3)
				{
					//decrement factor
					factor = factor - 0.1;
				}
				//display new factor
				sprintf(LCD_BUFFER, "%.1f", factor);
				BSP_LCD_GLASS_DisplayString((uint8_t*)LCD_BUFFER);
				//test if CW or CCW and set period
				if(step_Setting  == 1){
					htim3.Instance->ARR = 13125 / factor;		
				}else{
					htim3.Instance->ARR = 13125 / factor / 2;
				}
				break;
			
			}
			
			default:
					break;
	  } 
			
}
		
//Tim3 callback
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) 
{
	switch (current_State)
		{
			//first state
			case 0:
			{
				//specify is pin is high or low
				HAL_GPIO_WritePin(GPIOE, GPIO_PIN_12, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOE, GPIO_PIN_14, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_SET);
				break;
			}
			//second state
			case 1:
			{
				//specify is pin is high or low
				HAL_GPIO_WritePin(GPIOE, GPIO_PIN_12, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOE, GPIO_PIN_14, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_RESET);
				break;
			}
			
			//third state
			case 2:
			{
				//specify is pin is high or low
				HAL_GPIO_WritePin(GPIOE, GPIO_PIN_12, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOE, GPIO_PIN_14, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_RESET);
				break;
			}
			
			//fourth state
			case 3:
			{
				//specify is pin is high or low
				HAL_GPIO_WritePin(GPIOE, GPIO_PIN_12, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOE, GPIO_PIN_14, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_RESET);
				break;
			}
			
			//fifth state
			case 4:
			{
				//specify is pin is high or low
				HAL_GPIO_WritePin(GPIOE, GPIO_PIN_12, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOE, GPIO_PIN_14, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_RESET);
				break;
			}
			
			//sixth state
			case 5:
			{
				//specify is pin is high or low
				HAL_GPIO_WritePin(GPIOE, GPIO_PIN_12, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOE, GPIO_PIN_14, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_RESET);
				break;
			}
			
			//seven state
			case 6:
			{
				//specify is pin is high or low
				HAL_GPIO_WritePin(GPIOE, GPIO_PIN_12, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOE, GPIO_PIN_14, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_SET);
				break;
			}
			
			//eigth state
			case 7:
			{
				//specify is pin is high or low
				HAL_GPIO_WritePin(GPIOE, GPIO_PIN_12, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOE, GPIO_PIN_14, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_SET);
				break;
			}
		}
		
		//CW
		if(direction_Setting == 1)
		{
			//CW & half
			if(step_Setting == 0)
			{
				//increment state and % 8
				current_State = (current_State + 1)%8;
			//CW & full
			}else{
				//want current_State to fall on even values
				if(current_State % 2 == 1){current_State++;}
				//increment current_State by 2 and % 8
				current_State = (current_State + 2) % 8;
			}
			
		}
		
		//CCW
		else
		{
			//CCW & half
			if(step_Setting == 0)
			{
				//increment current_State by 7 and %8
				//effectively subtract 1
				current_State = (current_State + 7)%8;
			//CCW & full
			}else{
				//want current_State to fall on even values
				if(current_State % 2 == 1){current_State++;}
				//increment current_State by 6 and % 8
				//effectively subtract 2
				current_State = (current_State + 6) % 8;
			}
		}
			//reqd  for callback
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
	BSP_LCD_GLASS_Init();
	BSP_LED_Init(LED4);
	BSP_LED_Toggle(LED4);
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
	BSP_LCD_GLASS_Clear();
	BSP_LCD_GLASS_DisplayString((uint8_t*)"MOTOR");
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
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 399;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 13125;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  __HAL_TIM_ENABLE_OCxPRELOAD(&htim3, TIM_CHANNEL_1);
  /* USER CODE BEGIN TIM3_Init 2 */
	if(HAL_TIM_Base_Start_IT(&htim3) != HAL_OK){
		Error_Handler();
	}
  /* USER CODE END TIM3_Init 2 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, OUT_1_Pin|OUT_2_Pin|OUT_3_Pin|OUT_4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : JOY_CENTER_Pin */
  GPIO_InitStruct.Pin = JOY_CENTER_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(JOY_CENTER_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : JOY_LEFT_Pin JOY_RIGHT_Pin JOY_UP_Pin JOY_DOWN_Pin */
  GPIO_InitStruct.Pin = JOY_LEFT_Pin|JOY_RIGHT_Pin|JOY_UP_Pin|JOY_DOWN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : OUT_1_Pin OUT_2_Pin OUT_3_Pin OUT_4_Pin */
  GPIO_InitStruct.Pin = OUT_1_Pin|OUT_2_Pin|OUT_3_Pin|OUT_4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

}

/* USER CODE BEGIN 4 */

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
