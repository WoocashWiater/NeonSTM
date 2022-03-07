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
SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim14;
TIM_HandleTypeDef htim15;
TIM_HandleTypeDef htim16;
TIM_HandleTypeDef htim17;

/* USER CODE BEGIN PV */
volatile uint16_t pulse_count;
volatile uint16_t position;						// position of the encoder, ranging from 0 to 100
volatile uint16_t prev_position=0;
volatile uint8_t state=0;						// the state of the device (0 - working normally, 1 - switching mode, 2 - changing mode's parameter)
static volatile int8_t mode=0;             		// different light modes
static volatile int8_t temp_mode=0;             // different light modes

static volatile uint8_t brightness_level_startup=100;			// startup
static volatile uint8_t startup_end=0;
static volatile uint8_t brightness_level=50;				// mode 0 parameters

static volatile uint8_t brightness_level_letter0_mode1=100;	// mode 1 parameters
static volatile uint8_t brightness_level_letter1_mode1=100;
static volatile uint8_t counter_mode1=0;
static volatile uint8_t threshold_mode1=10;

static volatile uint8_t counter_mode2=0;					// mode 2 parameters
static volatile uint16_t threshold_mode2=100;
static volatile uint8_t phase_mode2=0;
static volatile uint8_t counter_mode3=0;					// mode 3 parameters
static volatile uint16_t threshold_mode3=100;
static volatile uint8_t phase_mode3=0;

const uint16_t LB_ALL[]={0xFFFF,0xFFFF};
const uint16_t LB_OFF[]={0x0000,0x0000};
const uint16_t LB_MODE2[]={0x0000,0x0001,0x0000,0x0003,0x0000,0x0007,0x0000,0x000F,0x0000,0x001F,0x0000,0x005F,0x0000,0x00DF,0x0000,0x00FF,
							0x0001,0x00FF,0x0003,0x00FF,0x0007,0x00FF,0x000F,0x00FF,0x001F,0x00FF,0x009F,0x00FF,0x029F,0x00FF,0x02DF,0x00FF,0x03DF,0x00FF,0x03FF,0x00FF,
							0x03FF,0x00FE,0x03FF,0x00FC,0x03FF,0x00F8,0x03FF,0x00F0,0x03FF,0x00E0,0x03FF,0x00A0,0x03FF,0x0020,0x03FF,0x0000,
							0x03FE,0x0000,0x03FC,0x0000,0x03F8,0x0000,0x03F0,0x0000,0x03E0,0x0000,0x0360,0x0000,0x0160,0x0000,0x0120,0x0000,0x0020,0x0000,0x0000,0x0000};
const uint16_t LB_MODE3[]={0x0021,0x0001,0x0123,0x0003,0x0146,0x0026,0x024C,0x00AC,0x0298,0x00D8,0x0090,0x0050};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM16_Init(void);
static void MX_TIM17_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM14_Init(void);
static void MX_TIM15_Init(void);
/* USER CODE BEGIN PFP */

// SPI Functions --------------------------------------------------------------
static void SPI_SendWord(uint16_t data[], uint8_t num_of_words);
void SPI_LatchData(void);
// LED Functions --------------------------------------------------------------
void LED_TurnOnAll(void);
void LED_TurnOffAll(void);
void LED_TurnOnSingle(uint8_t led_num);
void LED_ToggleSingle(uint8_t led_num);
// Callback Functions ---------------------------------------------------------
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
//void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim);
void LB_Startup(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_SPI1_Init();
  MX_TIM3_Init();
  MX_TIM16_Init();
  MX_TIM17_Init();
  MX_TIM6_Init();
  MX_TIM14_Init();
  MX_TIM15_Init();
  /* USER CODE BEGIN 2 */

  // Turning on all peripherals ---------------------------
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
  HAL_TIM_PWM_Start(&htim16, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim17, TIM_CHANNEL_1);

  // Startup procedure ------------------------------------
  LB_Startup();

  // state = 2 settings -----------------------------------
  if(mode==0)
  {
	  TIM16->CCR1=brightness_level;
	  TIM17->CCR1=brightness_level;
	  SPI_SendWord(LB_ALL,2);
	  SPI_LatchData();
  }

  // state = 1 settings -----------------------------------
  pulse_count=TIM3->CNT;
  position=pulse_count/2;
  prev_position=position;
  // ------------------------------------------------------

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  while(state==0)			// normalna praca neonu
	  {
		  switch(mode)
		  {
		  	  case 0:
		  		  break;
		  	  case 1:
		  		  break;
		  	  case 2:
		  		  break;
		  	  case 3:
		  		  break;
		  }

		  /*
		  HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_SET);
		  HAL_Delay(100);
		  HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_RESET);
		  HAL_Delay(100);
		  */

	  }

	  while(state==1)			// stan wyboru konkretnego trybu
	  {
		  prev_position=position;
		  pulse_count=TIM3->CNT;
		  position=pulse_count/2;

		  if(position!=prev_position)
		  {
			  //HAL_Delay(50);

			  if(position==0&&prev_position==100)
				  --temp_mode;
			  else if(position==100&&prev_position==0)
				  ++temp_mode;
			  else if(position<prev_position)
				  ++temp_mode;
			  else if(position>prev_position)
				  --temp_mode;

			  if(temp_mode>3)
				  temp_mode=0;
			  else if(temp_mode<0)
				  temp_mode=3;

			  LED_TurnOnSingle(temp_mode);
		  }
		  //HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);
	  }

	  while(state==2)			// stan zmiany parametru dotyczacego wybranego wczesniej trybu
	  {
		  prev_position=position;
  		  pulse_count=TIM3->CNT;
  		  position=pulse_count/2;

  		  if(position!=prev_position)
  		  {
			  switch(mode)
			  {
				  case 0:
					  if(position==0&&prev_position==100)
					  {
						  position=100;
						  TIM3->CNT=200;
					  }
					  if(position==100&&prev_position==0)
					  {
						  position=0;
						  TIM3->CNT=0;
					  }

					  TIM16->CCR1=position;
					  TIM17->CCR1=position;
					  break;
				  case 1:
					  if(position==21&&prev_position==20)
					  {
						  position=20;
						  TIM3->CNT=40;
					  }
					  if(position==0&&prev_position==1)
					  {
						  position=1;
						  TIM3->CNT=2;
					  }

					  threshold_mode1=position;
					  break;
				  case 2:
					  if(position==0&&prev_position==100)
					  {
						  position=100;
						  TIM3->CNT=200;
					  }
					  if(position==0&&prev_position==1)
					  {
						  position=1;
						  TIM3->CNT=2;
					  }

					  threshold_mode2=position*10;
					  break;
				  case 3:
					  if(position==0&&prev_position==100)
					  {
						  position=100;
						  TIM3->CNT=200;
					  }
					  if(position==0&&prev_position==1)
					  {
						  position=1;
						  TIM3->CNT=2;
					  }

					  threshold_mode3=position*10;
					  break;
			  }
  		  }

	  }

	  /*
	  uint8_t temp=0;
	  uint16_t data[]={0xFFFF};
	  if (temp==0)
	  {
		  TIM16->CCR1=0;
		  SPI_SendWord(data,1);
		  SPI_LatchData();
		  temp=1;
	  }

	  pulse_count=TIM3->CNT;
	  positions=pulse_count/2;
	  TIM16->CCR1=positions;
	  /*

	  if(HAL_GPIO_ReadPin(ENC_SWITCH_GPIO_Port, ENC_SWITCH_Pin)==0)
	  {
		  if(state)		//state=1
		  {
			  SPI_SendWord(0x0000);
			  SPI_LatchData();
			  state=0;
		  }
		  else			//state=0
		  {
			  SPI_SendWord(0xFFFF);
			  SPI_LatchData();
			  state=1;
		  }
	  }*/

	  //HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_SET);
	  //HAL_Delay(100);
	  //HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_RESET);
	  //HAL_Delay(100);
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

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_1LINE;
  hspi1.Init.DataSize = SPI_DATASIZE_16BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */
  //__HAL_SPI_ENABLE(&hspi1);
  /* USER CODE END SPI1_Init 2 */

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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 201;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 15;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 15;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 9999;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 399;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief TIM14 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM14_Init(void)
{

  /* USER CODE BEGIN TIM14_Init 0 */

  /* USER CODE END TIM14_Init 0 */

  /* USER CODE BEGIN TIM14_Init 1 */

  /* USER CODE END TIM14_Init 1 */
  htim14.Instance = TIM14;
  htim14.Init.Prescaler = 799;
  htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim14.Init.Period = 100;
  htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM14_Init 2 */

  /* USER CODE END TIM14_Init 2 */

}

/**
  * @brief TIM15 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM15_Init(void)
{

  /* USER CODE BEGIN TIM15_Init 0 */

  /* USER CODE END TIM15_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM15_Init 1 */

  /* USER CODE END TIM15_Init 1 */
  htim15.Instance = TIM15;
  htim15.Init.Prescaler = 9999;
  htim15.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim15.Init.Period = 45;
  htim15.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim15.Init.RepetitionCounter = 0;
  htim15.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim15) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim15, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim15, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM15_Init 2 */

  /* USER CODE END TIM15_Init 2 */

}

/**
  * @brief TIM16 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM16_Init(void)
{

  /* USER CODE BEGIN TIM16_Init 0 */

  /* USER CODE END TIM16_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 799;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 99;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim16, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim16, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */

  /* USER CODE END TIM16_Init 2 */
  HAL_TIM_MspPostInit(&htim16);

}

/**
  * @brief TIM17 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM17_Init(void)
{

  /* USER CODE BEGIN TIM17_Init 0 */

  /* USER CODE END TIM17_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM17_Init 1 */

  /* USER CODE END TIM17_Init 1 */
  htim17.Instance = TIM17;
  htim17.Init.Prescaler = 799;
  htim17.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim17.Init.Period = 99;
  htim17.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim17.Init.RepetitionCounter = 0;
  htim17.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim17) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim17) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim17, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim17, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM17_Init 2 */

  /* USER CODE END TIM17_Init 2 */
  HAL_TIM_MspPostInit(&htim17);

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

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LED0_Pin|LED1_Pin|LED2_Pin|LED3_Pin 
                          |LATCH_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI_RESET_GPIO_Port, SPI_RESET_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : ENC_SWITCH_Pin */
  GPIO_InitStruct.Pin = ENC_SWITCH_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ENC_SWITCH_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED0_Pin LED1_Pin LED2_Pin LED3_Pin 
                           LATCH_Pin */
  GPIO_InitStruct.Pin = LED0_Pin|LED1_Pin|LED2_Pin|LED3_Pin 
                          |LATCH_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI_RESET_Pin */
  GPIO_InitStruct.Pin = SPI_RESET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SPI_RESET_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);

}

/* USER CODE BEGIN 4 */
// CALLBACK FUNCTIONS -----------------------------------------------------------------
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	++state;

	if(state==1)
		temp_mode=mode;
		LED_TurnOnSingle(mode);

	if(state==2)
	{
		mode=temp_mode;
		LED_TurnOffAll();
		HAL_TIM_Base_Start_IT(&htim6);
		//if(mode!=1&&mode!=2)
		HAL_TIM_Base_Stop_IT(&htim14);
		if(mode==0)
		{
			TIM3->CNT=brightness_level*2;
			SPI_SendWord(LB_ALL,2);
			SPI_LatchData();
		}
		if(mode==1)
		{
			TIM3->CNT=20;
			SPI_SendWord(LB_ALL,2);
			SPI_LatchData();
			counter_mode1=0;
			brightness_level_letter0_mode1=100;
			brightness_level_letter1_mode1=100;
			HAL_TIM_Base_Start_IT(&htim14);
		}
		if(mode==2)
		{
			TIM3->CNT=20;
			//position=10;
			SPI_SendWord(LB_OFF,2);
			SPI_LatchData();
			counter_mode2=0;
			TIM16->CCR1=brightness_level;
			TIM17->CCR1=brightness_level;
			HAL_TIM_Base_Start_IT(&htim14);
		}
		if(mode==3)
		{
			TIM3->CNT=20;
			//position=10;
			SPI_SendWord(LB_OFF,2);
			SPI_LatchData();
			counter_mode3=0;
			TIM16->CCR1=brightness_level;
			TIM17->CCR1=brightness_level;
			HAL_TIM_Base_Start_IT(&htim14);
		}
	}

	if(state>2) // state = 0
	{
		HAL_TIM_Base_Stop_IT(&htim6);
		LED_TurnOffAll();
		state=0;

		if(mode==0)
		{
			brightness_level=position;
		}
		if(mode==1)
		{
		}
	}

	/*
	switch (state)
	{
	case 0:
		LED
		break;
	case 1:
		LED_TurnOnSingle(mode);
		break;

	}*/

	//state=0; // zakomentuj/usun pozniej to
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance==TIM6)
		LED_ToggleSingle(mode);
	if(htim->Instance==TIM14)
	{
		if(mode==1)
		{
			counter_mode1++;
			if(counter_mode1>=threshold_mode1)
			{
				if((brightness_level_letter0_mode1>=0&&brightness_level_letter0_mode1<50)&&brightness_level_letter1_mode1==0)
				{
					brightness_level_letter0_mode1++;
				}
				else if((brightness_level_letter0_mode1>=50&&brightness_level_letter0_mode1<100)&&(brightness_level_letter1_mode1>=0&&brightness_level_letter1_mode1<50))
				{
					brightness_level_letter0_mode1++;
					brightness_level_letter1_mode1++;
				}
				else if(brightness_level_letter0_mode1==100&&(brightness_level_letter1_mode1>=50&&brightness_level_letter1_mode1<100))
				{
					brightness_level_letter1_mode1++;
				}
				else if((brightness_level_letter0_mode1>50&&brightness_level_letter0_mode1<=100)&&brightness_level_letter1_mode1==100)
				{
					brightness_level_letter0_mode1--;
				}
				else if((brightness_level_letter0_mode1>=0&&brightness_level_letter0_mode1<=50)&&(brightness_level_letter1_mode1>50&&brightness_level_letter1_mode1<=100))
				{
					brightness_level_letter0_mode1--;
					brightness_level_letter1_mode1--;
				}
				else if(brightness_level_letter0_mode1==0&&(brightness_level_letter1_mode1>0&&brightness_level_letter1_mode1<=50))
				{
					brightness_level_letter1_mode1--;
				}

				TIM16->CCR1=brightness_level_letter0_mode1;
				TIM17->CCR1=brightness_level_letter1_mode1;
				counter_mode1=0;
			}
		}

		if(mode==2)
		{
			counter_mode2++;
			if(counter_mode2>=threshold_mode2)
			{
				SPI_SendWord(&LB_MODE2[phase_mode2],2);
				SPI_LatchData();
				counter_mode2=0;
				if(phase_mode2<70)
					phase_mode2=phase_mode2+2;
				else
				{
					SPI_SendWord(LB_OFF,2);
					SPI_LatchData();
					phase_mode2=0;
				}
			}

		}
		if(mode==3)
		{
			counter_mode3++;
			if(counter_mode3>=threshold_mode3)
			{
				SPI_SendWord(&LB_MODE3[phase_mode3],2);
				SPI_LatchData();
				counter_mode3=0;
				if(phase_mode3<12)
					phase_mode3=phase_mode3+2;
				else
				{
					SPI_SendWord(LB_OFF,2);
					SPI_LatchData();
					phase_mode3=0;
				}
			}

		}
	}
	if(htim->Instance==TIM15)
	{
		brightness_level_startup--;
		TIM16->CCR1=brightness_level_startup;
		TIM17->CCR1=brightness_level_startup;

		if(brightness_level_startup==0)
		{
			brightness_level_startup=100;
			startup_end=1;
		}
	}
}
/*
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	//static uint16_t pulse_count1, pulse_count2;

	// detection of the direction of the rotation
	if(htim->Instance==TIM3)
	{
		++lama;
		++occured;
		if(occured>=2)
		{
			occured=0;
			pulse_count2=TIM3->CNT;

			if(pulse_count1==pulse_count2)
				rotation=1;
			else
				rotation=2;
		}
		else
			pulse_count1=TIM3->CNT;

		// changing mode according to the detected direction
		if(rotation)
		{
			if(rotation==1)
				--mode;
			else
				++mode;

			if(mode>3)
				mode=0;
			if(mode<0)
				mode=3;

			LED_TurnOnSingle(mode);
			rotation=0;
		}
	}
}*/
// ------------------------------------------------------------------------------------
void SPI_SendWord(uint16_t data[], uint8_t num_of_words)
{
	HAL_SPI_Transmit(&hspi1, (uint8_t*)(data), num_of_words, HAL_MAX_DELAY);
}

void SPI_LatchData(void)
{
	HAL_GPIO_WritePin(LATCH_GPIO_Port, LATCH_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LATCH_GPIO_Port, LATCH_Pin, GPIO_PIN_RESET);
}
void LED_TurnOffAll(void)
{
	HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_RESET);
}
void LED_TurnOnAll(void)
{
	HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_SET);
}
void LED_TurnOnSingle(uint8_t led_num)
{
	LED_TurnOffAll();
	switch(led_num)
	{
		case 0:
			HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_SET);
			break;
		case 1:
			HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
			break;
		case 2:
			HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);
			break;
		case 3:
			HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_SET);
			break;
	}
}
void LED_ToggleSingle(uint8_t led_num)
{
	switch(led_num)
	{
		case 0:
			HAL_GPIO_TogglePin(LED0_GPIO_Port, LED0_Pin);
			break;
		case 1:
			HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
			break;
		case 2:
			HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
			break;
		case 3:
			HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin);
			break;
	}
}
void LB_Startup(void)
{
	SPI_SendWord(LB_ALL,2);
	SPI_LatchData();
	HAL_TIM_Base_Start_IT(&htim15);
	while(startup_end==0);
	HAL_TIM_Base_Stop_IT(&htim15);
	startup_end=0;
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
