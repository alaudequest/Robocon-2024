/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "DriveMotor.h"
#include "Encoder.h"
#include "PID.h"
#include "stdbool.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define GearRatio 19.2
#define CPR 614

#define SpeedPropotional 0.0003
#define SpeedIntergral 20
#define SpeedDerivative 0
#define SpeedAlpha 0
#define PositionPropotional 1
#define DeltaT 0.001

#define NotBall 0
#define IsBall 1

#define Dropped 1
#define Pending 0

#define Ready 1

#define PutBall_AboveLimit -90000
#define PutBall_BelowLimit -1000
#define PutBall_DropBall -115000

#define RecognizeBallTime 2000
#define SetHomeManualTime 10000
#define DropBallTime 1000
#define ResetEncoderTime 1000

#define PutBall_UpperHighSpeed -400
#define PutBall_UpperLowSpeed -300
#define PutBall_LowerHighSpeed 400
#define PutBall_LowerLowSpeed 200
#define PutBall_StopSpeed 0

#define uI_AboveLimit 1000
#define uI_BelowLimit -1000
#define u_AboveLimit 1000
#define u_BelowLimit -1000
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;

osThreadId LogicTaskHandle;
osThreadId SensorTaskHandle;
/* USER CODE BEGIN PV */
int16_t PWM;
float SpeedTest, PositionTest;
bool __setStart = 0;
bool PutBall = 0;
bool __NotBall = 0, __isBall = 0;
uint16_t cnt = 0;
MotorDrive DC;
EncoderRead ENC_DC;
PID_Param PID_DC_Speed;
PID_Param PID_DC_Pos;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM2_Init(void);
void StartLogicTask(void const * argument);
void StartSensorTask(void const * argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//void PID_Speed()
//{
//	SpeedReadNonReset(&ENC_DC);
//	Pid_Cal(&PID_DC_Speed, SpeedTest, ENC_DC.vel_Real);
//	DC_Drive_BTS(&DC, &htim4, 1000, PID_DC_Speed.u, TIM_CHANNEL_1, TIM_CHANNEL_2);
//}
//
//void PID_Pos()
//{
//	CountRead(&ENC_DC, count_ModeDegree);
//	Pid_Cal(&PID_DC_Pos, PositionTest, CountRead(&ENC_DC, count_ModeDegree));
//	SpeedTest = PID_DC_Pos.u;
//	PID_Speed();
//}

void SetupStartPos()
{
	DC_Drive_BTS(&DC, &htim4, motor_Reserve, PutBall_LowerLowSpeed, TIM_CHANNEL_1, TIM_CHANNEL_2);
	osDelay(SetHomeManualTime);
	DC_Drive_BTS(&DC, &htim4, motor_Reserve, PutBall_StopSpeed, TIM_CHANNEL_1, TIM_CHANNEL_2);
	ResetCount(&ENC_DC, 1);
	__setStart = Ready;
}

void PutBallTask()
{
		  CountRead(&ENC_DC, count_ModeX1);
		  if(!PutBall)
		  {
			  if(__isBall)
			  {
				  if(!__NotBall)
				  {
					  PWM = PutBall_UpperHighSpeed;
					  if(CountRead(&ENC_DC, count_ModeX1) < PutBall_AboveLimit)
					  {
						  PWM = PutBall_UpperLowSpeed;
					  }
					  if(CountRead(&ENC_DC, count_ModeX1)  < PutBall_DropBall)
					  {
						  PWM = PutBall_StopSpeed;
						  PutBall = Dropped;
						  osDelay(DropBallTime);
					  }
				  }
				  else
				  {
					  PWM = PutBall_LowerLowSpeed;
					  if(CountRead(&ENC_DC, count_ModeX1) > PutBall_BelowLimit)
					  {
					  		 PutBall = PutBall_StopSpeed;
					   	     __isBall = NotBall;
					   		 osDelay(10);
					   		 cnt = 0;
					   		 osDelay(ResetEncoderTime);
					   		 ResetCount(&ENC_DC, 1);
					   		 osDelay(10);
					   		 PWM = PutBall_StopSpeed;
					  }
				  }
			  }
		  }
		  else
		  {
			  if(PutBall)
			  {
			  	  PWM = PutBall_LowerLowSpeed;
			  	 if(CountRead(&ENC_DC, count_ModeX1) > PutBall_BelowLimit)
			  	 {
			  		 PutBall = Pending;
			  	     __isBall = NotBall;
			  		 osDelay(10);
			  		 cnt = 0;
			  		 osDelay(ResetEncoderTime);
			  		 ResetCount(&ENC_DC, 1);
			  		 osDelay(10);
			  		 PWM = PutBall_StopSpeed;
			  	 }
			  }
		  }
		  DC_Drive_BTS(&DC, &htim4, motor_Reserve, PWM, TIM_CHANNEL_1, TIM_CHANNEL_2);
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
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);

  HAL_TIM_Encoder_Start_IT(&htim2, TIM_CHANNEL_ALL);
  EncoderSetting(&ENC_DC, &htim2, 11788, DeltaT);

//  Pid_SetParam(&PID_DC_Speed, SpeedPropotional, SpeedIntergral, SpeedDerivative, SpeedAlpha, DeltaT, uI_AboveLimit, uI_BelowLimit, u_AboveLimit, u_BelowLimit);
//  Pid_SetParam(&PID_DC_Pos, PositionPropotional, 0, 0, 0, DeltaT, uI_AboveLimit, uI_BelowLimit, u_AboveLimit, u_BelowLimit);

  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of LogicTask */
  osThreadDef(LogicTask, StartLogicTask, osPriorityIdle, 0, 128);
  LogicTaskHandle = osThreadCreate(osThread(LogicTask), NULL);

  /* definition and creation of SensorTask */
  osThreadDef(SensorTask, StartSensorTask, osPriorityAboveNormal, 0, 128);
  SensorTaskHandle = osThreadCreate(osThread(SensorTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
//	  DC_Drive_BTS(&DC, &htim4, 0, PWM, TIM_CHANNEL_1, TIM_CHANNEL_2);
//	  PID_Speed();
//	  PID_Pos();
//	  HAL_Delay(1);
//	  CountRead(&ENC_DC, count_ModeX1);
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
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
  htim4.Init.Prescaler = 84-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 1000-1;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin : BJ300_Pin */
  GPIO_InitStruct.Pin = BJ300_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BJ300_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartLogicTask */
/**
  * @brief  Function implementing the LogicTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartLogicTask */
void StartLogicTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
	if(!__setStart)
	{
		SetupStartPos();
	}
	if(__isBall)
	{
		PutBallTask();
	}
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartSensorTask */
/**
* @brief Function implementing the SensorTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartSensorTask */
void StartSensorTask(void const * argument)
{
  /* USER CODE BEGIN StartSensorTask */
  /* Infinite loop */
  for(;;)
  {
	 __NotBall = HAL_GPIO_ReadPin(BJ300_GPIO_Port, BJ300_Pin);
	 if(!__NotBall)
	 {
		 if(cnt < RecognizeBallTime)
		 {
			 cnt++;
		 }
		 if(cnt == RecognizeBallTime)
		 {
			 __isBall = IsBall;
		 }
	 }
    osDelay(1);
  }
  /* USER CODE END StartSensorTask */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM14 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM14) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
