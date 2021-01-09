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
#include "i2c_at24c64.h"
#include "stm32l476g_discovery.h"
#include "stm32l476g_discovery_glass_lcd.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
//Primary set of states
//provides firstlevel branching to the various tasks which are to be achieved
typedef enum {currentTime, currentDate, timeSettings, saveTime, accessSaves} State;

// Auxillary state for the timeSettings state
//7 states consisting of the 7 variables  associated with date and time 
//which we will be keeping track of and appending
typedef enum {seconds, minutes, hours, weekday, date, month, year} TimeSettingsTraverse;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define EEPROM_ADDRESS 0xA0
//saved time 1 is stored at base address 3
#define EEPROM_data_1 0
//saved time 2 is stored at base address 6
#define EEPROM_data_2 3

//setting base address for EEPROM reference
__IO uint16_t memory_Base = 0x000A;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
RTC_HandleTypeDef hrtc;
/* USER CODE BEGIN PV */


char timestring[10] ={0};  //used to display time in "displayTime()"
char lcd_buffer[64];		// very large buffer used to display several variables sequentially in "displayDate()"

//State Presets
State state =  currentTime;
TimeSettingsTraverse timeSettingsTraverse = year;

//Real Time Clock update variables
//time update
RTC_TimeTypeDef showTime;
//date update
RTC_DateTypeDef showDate;

//Updating RTC in timeSettings
//useful as a proxy between updating and displaying variables above
RTC_TimeTypeDef updateTime;
RTC_DateTypeDef updateDate;

//Flags set by interupts to direct main
int alarmFlag = 0; 					 					//set when HAL_RTC_AlarmAEventCallback is called which occurs every 1 second
int joyFlag = 1; 											//set when a joystick input is detected
int incrementSetting = 0; 						//indicates that the setting should be increased by one
int updateSettingsFlag = 0; 					//indicates that the new date and time setting that were incremented should updated to the RTC

// Stores the initial time from HAL_GetTick() when the Select joy pin is first pressed
//-1 because HAL_GetTick() could be zero which would break functionality if we 
//defaulted this flag to zero
int selectStartTime = -1;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/\

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_RTC_Init(void);

/* USER CODE BEGIN PFP */
void RTC_TimeShow(void); 
void RTC_DateShow(void); 
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


void writeEEPROM(uint16_t ref_Addr, RTC_TimeTypeDef *sTime)
{
	I2C_ByteWrite(&hi2c1, EEPROM_ADDRESS, memory_Base + ref_Addr, sTime->Hours);
	I2C_ByteWrite(&hi2c1, EEPROM_ADDRESS, memory_Base + ref_Addr + 1, sTime -> Minutes);
	I2C_ByteWrite(&hi2c1, EEPROM_ADDRESS, memory_Base + ref_Addr + 2, sTime -> Seconds);
}

RTC_TimeTypeDef readEEPROM(uint16_t ref_Addr)
{
	RTC_TimeTypeDef sTime;
	sTime.Hours = I2C_ByteRead(&hi2c1, EEPROM_ADDRESS, memory_Base + ref_Addr);
	sTime.Minutes = I2C_ByteRead(&hi2c1, EEPROM_ADDRESS, memory_Base + ref_Addr + 1);
	sTime.Seconds = I2C_ByteRead(&hi2c1, EEPROM_ADDRESS, memory_Base + ref_Addr + 2);
	return sTime;
}

// Interrupt is called every second
void HAL_RTC_AlarmAEventCallback(RTC_HandleTypeDef* hrtc)
{
	// Update the current date and time
	HAL_RTC_GetTime(hrtc, &showTime, RTC_FORMAT_BIN);
	HAL_RTC_GetDate(hrtc, &showDate, RTC_FORMAT_BIN);
	BSP_LED_Toggle(LED5);
	alarmFlag = 1;
}

//used to display short tags
//includes a second input to avoid a wait time
void displayTags(char *tag, int bypass)
{
	BSP_LCD_GLASS_Clear();
	BSP_LCD_GLASS_DisplayString((uint8_t*) tag);
	if (bypass == 0){HAL_Delay(1000);}
}

void displayTime(RTC_TimeTypeDef *sTime) //displays time
{
	BSP_LCD_GLASS_Clear(); //want empty lcd so it can be updated
	
	//get current time from sTime structure and save into the RTC_TimeStructure
	sprintf(timestring,"%02d%02d%02d",sTime->Hours,sTime->Minutes,sTime->Seconds); 
	
	BSP_LCD_GLASS_DisplayString((uint8_t*)timestring);
	
	//formatting
	BSP_LCD_GLASS_DisplayChar((uint8_t*) &timestring[1], POINT_OFF, DOUBLEPOINT_ON, LCD_DIGIT_POSITION_2);
	BSP_LCD_GLASS_DisplayChar((uint8_t*) &timestring[3], POINT_OFF, DOUBLEPOINT_ON, LCD_DIGIT_POSITION_4);
	
}

// Displays Date info using scroll sentence 
void displayDate(RTC_DateTypeDef *sDate) 
{
	BSP_LCD_GLASS_Clear();
	//Caps lock the variables -> LCD displays some lowercases badly
	//add a large empty space at beggining to prevent repitition 
	//this is an issue with ScrollSentence but it must be used here 
	sprintf(lcd_buffer, "      WEEKDAY %d  DATE %d  MONTH %d  YEAR %d", sDate->WeekDay, sDate->Date, sDate->Month, sDate->Year);
	
	BSP_LCD_GLASS_ScrollSentence((uint8_t*) lcd_buffer, 1, 300);
}

//Executes every time a joy pin is detected
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	switch (GPIO_Pin)
	{
		case SEL_JOY_PIN:
		{
			if (state == currentTime)
			{
				//Save the time stamp when sel joy pin was pressed down 
				//Enables us to calculate how long it  isheld down for
				selectStartTime = HAL_GetTick(); //get tick is in ms
			}
			else if (state == timeSettings)
			{
				//Increments the setting value
				incrementSetting = 1;
			}
			break;
		}
		//no functionality
		case UP_JOY_PIN:
		{
			break;
		}
		case RIGHT_JOY_PIN:
		{
			if (state == currentTime)
			{
				//ensures that the user has recent RTC info that they can append
				updateDate = showDate;
				updateTime = showTime;
				// Enter into the date and time setting mode, starting with YEAR
				state = timeSettings;
				timeSettingsTraverse = seconds;
				joyFlag = 1;
			}
			else if (state == timeSettings)
			{
				//want to return when right is pressed in timesettings
				state = currentTime;
				joyFlag = 1;
				//enables the recently appended values to be updated to our RTC clock and date
				updateSettingsFlag = 1;
			}
			break;
		}
		//Use downpin to replay tags 
		//good if  you get lost
		case DOWN_JOY_PIN:
		{
			//reset JoyFlag to prevent looping
			joyFlag = 1;
			break;
		}
		case LEFT_JOY_PIN:
		{
			if (state == currentTime)
			{
				//enbles entry to acessSaves Mode which iterates through the 2 saved times
				state = accessSaves;
				//reset JoyFlag to prevent looping
				joyFlag = 1;
			}
			else if (state == accessSaves) 
			{
				//return to the time display mode
				state = currentTime;
				//reset JoyFlag to prevent looping
				joyFlag = 1;
			}
			else if (state == timeSettings)
			{
				//Move to the next value in timeSettingsTraverse
				//enables us to traverse the list
				timeSettingsTraverse = (timeSettingsTraverse + 1) % 7;
				//reset JoyFlag to prevent looping
				joyFlag = 1;
			}
			break;
		}
	}
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main()
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
	BSP_LCD_GLASS_Init();
	BSP_LED_Init(LED4); 
	BSP_LED_Init(LED5);
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
	
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_RTC_Init();
  /* USER CODE BEGIN 2 */

	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	//main while loop begin
  while (1)
  {
		//Runs when the select button has been pressed recently & we are on time screen
		//prevents this from running during timeSetting 
		//We know  HAL_GetTick >= 0, so thus,
		//when select has been pressed recently this holds
		if (selectStartTime >= 0 && state == currentTime)
		{
			//Runs when button is held down for 3 seconds
			if (BSP_JOY_GetState() == JOY_SEL && HAL_GetTick()- selectStartTime > 3000)
			{
				//reset selectStartTime to prevent looping
				selectStartTime = -1;
				
				// Display the current date
				state = currentDate;
				joyFlag = 1;
			}
			else if (BSP_JOY_GetState() != JOY_SEL)
			{
				//reset flag to prevent looping
				selectStartTime = -1;
				
				// Save the time being displayed in CURRENTTIME to EEPROM
				state = saveTime;
				joyFlag = 1;
			}
		}
		
	
		if (joyFlag == 1) 
		{
			//reset flag to prevent looping
			joyFlag = 0;
			switch (state)
			{
				// Display the current time
				case currentTime:
				{
					displayTags("TIME",0);
					displayTime(&showTime);
					break;
				}
				// Display the current date
				case currentDate:
				{
					displayTags("DATE",0);
					displayDate(&showDate);
					state = currentTime;
					break;
				}
				
				//if select was pressed or held for less than 3 seconds this executes
				case saveTime:
				{
					//dont care about oldest EEPROM time -> data_2, move EEPROM data 1 to EEPROM data 2, move just saved time to EEPROM data 1
					RTC_TimeTypeDef shiftTime = readEEPROM(EEPROM_data_1);
					writeEEPROM(EEPROM_data_2, &shiftTime);
					writeEEPROM(EEPROM_data_1, &showTime);
					//shows user that time was saved
					displayTags("SAVED",0);
					//go back to displaying time clock
					state = currentTime;
					break;
				}
				//Display most recent then older time saved in EEPROM
				case accessSaves:
				{
					RTC_TimeTypeDef recent = readEEPROM(EEPROM_data_1);
					displayTags("RECENT",0);
					displayTime(&recent);
					HAL_Delay(1000);
					RTC_TimeTypeDef older = readEEPROM(EEPROM_data_2);
					displayTags("OLDER",0);
					displayTime(&older);
					HAL_Delay(1000);
					break;
				}
				//default to timeSettingsTraverse = seconds
				case timeSettings:
				{
					//display "Option" before each heading for clarity
					displayTags("OPTION",0);
					//display tagline of setting being accessed per requirements
					if (timeSettingsTraverse == year) {
						displayTags("YEAR",0);
						//format using sprintf and store in the large buffer lcd_buffer
						sprintf(lcd_buffer, "%d", updateDate.Year);
					}
					else if (timeSettingsTraverse == date) {
						displayTags("DATE",0);
						sprintf(lcd_buffer, "%d", updateDate.Date);
					}
					else if (timeSettingsTraverse == month) {
						displayTags("MONTH",0);
						sprintf(lcd_buffer, "%d", updateDate.Month);
					}
					else if (timeSettingsTraverse == weekday) {
						displayTags("WKDAY",0);
						sprintf(lcd_buffer, "%d", updateDate.WeekDay);
					}
					else if (timeSettingsTraverse == hours) {
						displayTags("HOUR",0);
						sprintf(lcd_buffer, "%d", updateTime.Hours);
					}
					else if (timeSettingsTraverse == minutes) {
						displayTags("MINUTE",0);
						sprintf(lcd_buffer, "%d", updateTime.Minutes);
					}
					else if (timeSettingsTraverse == seconds) {
						displayTags("SECOND",0);
						sprintf(lcd_buffer, "%d", updateTime.Seconds);
					}
					//output the updated value without delay (the 1 parameter) for rapid increasing
					displayTags(lcd_buffer,1);
					break;
				}
			}
		}
		
		//Runs when left joy pin pressed while in timeSettings state
		//Enables shift to the next setting from list of 7 date and time datapoints
		if ((incrementSetting) && (state == timeSettings))
		{
			//reset our flag to prevent infinite incrementSetting looping
			incrementSetting = 0;
			
			//if else over all datapoints to whichever the next setting is
			if (timeSettingsTraverse == year) {
				//increment the parameter and  store in update(date/time).parameter
				//use % for parameter specific formatting
				updateDate.Year = (updateDate.Year + 1) % 100;
				sprintf(lcd_buffer, "%d", updateDate.Year);
			}
			else if (timeSettingsTraverse == date) {
				updateDate.Date = (updateDate.Date % 31) + 1;
				sprintf(lcd_buffer, "%d", updateDate.Date);
			}
			else if (timeSettingsTraverse == month) {
				updateDate.Month = (updateDate.Month % 12) + 1;
				sprintf(lcd_buffer, "%d", updateDate.Month);
			}
			else if (timeSettingsTraverse == weekday) {
				updateDate.WeekDay = (updateDate.WeekDay % 7) + 1;
				sprintf(lcd_buffer, "%d", updateDate.WeekDay);
			}
			else if (timeSettingsTraverse == hours) {
				updateTime.Hours = (updateTime.Hours+1) % 24;
				sprintf(lcd_buffer, "%d", updateTime.Hours);
			}
			else if (timeSettingsTraverse == minutes) {
				updateTime.Minutes = (updateTime.Minutes+1) % 60;
				sprintf(lcd_buffer, "%d", updateTime.Minutes);
			}
			else if (timeSettingsTraverse == seconds) {
				updateTime.Seconds = (updateTime.Seconds+1) % 60;
				sprintf(lcd_buffer, "%d", updateTime.Seconds);
			}
			//output the updated value without delay (the 1 parameter) for rapid increasing
			displayTags(lcd_buffer,1);
		}
			
		//If timeSettings are updated this flag is raised
		//Thus, the RTC is then updated with these new datapoints
		if (updateSettingsFlag)
		{
			//reset our flag to prevent looping
			updateSettingsFlag = 0;
			
			//updates showTime with the variable stored in updateTime
			//showTime is the proxy variable enabling the update of RTC
			showTime = updateTime;
			
			HAL_RTC_SetTime(&hrtc, &showTime, RTC_FORMAT_BIN);
			
			showDate = updateDate;
			HAL_RTC_SetDate(&hrtc, &showDate, RTC_FORMAT_BIN);
		}
		
		// Executes every second when the HAL_RTC_AlarmAEventCallback interrupt is called
		if (alarmFlag)
		{
			//reset flag to prevent looping
			alarmFlag = 0;
			
			if (state == currentTime)
			{
				// Update the current time on the LCD
				displayTime(&showTime);
			}
		}
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_I2C1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
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
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00000E14;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};
  RTC_AlarmTypeDef sAlarm = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */
  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */

  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 8;
  sTime.Minutes = 30;
  sTime.Seconds = 0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_SATURDAY;
  sDate.Month = RTC_MONTH_NOVEMBER;
  sDate.Date = 7;
  sDate.Year = 20;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }
  /** Enable the Alarm A
  */
  sAlarm.AlarmTime.Hours = 0;
  sAlarm.AlarmTime.Minutes = 0;
  sAlarm.AlarmTime.Seconds = 0;
  sAlarm.AlarmTime.SubSeconds = 0;
  sAlarm.AlarmTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sAlarm.AlarmTime.StoreOperation = RTC_STOREOPERATION_RESET;
  sAlarm.AlarmMask = RTC_ALARMMASK_ALL;
  sAlarm.AlarmSubSecondMask = RTC_ALARMSUBSECONDMASK_ALL;
  sAlarm.AlarmDateWeekDaySel = RTC_ALARMDATEWEEKDAYSEL_DATE;
  sAlarm.AlarmDateWeekDay = 1;
  sAlarm.Alarm = RTC_ALARM_A;
  if (HAL_RTC_SetAlarm_IT(&hrtc, &sAlarm, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

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
  __HAL_RCC_GPIOB_CLK_ENABLE();

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
