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
#include "Encoder.h"
#include "DriveMotor.h"
#include "PID.h"
#include "stdlib.h"
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/*-----------------------------Begin:General Macro----------------------------*/
#define _SecondsPerMin 60
/*-----------------------------End:General Macro------------------------------*/

/*-----------------------------Begin:Initial Macro----------------------------*/
#define AtHome 0
#define NotAtHome 1
#define IntialFindingSpeed 20
#define AccurateFindingSpeed 2
#define FindingDegreeAboveLimit 180
#define FindingDegreeBelowLimit -180
#define AccurateFindingDegreeAboveLimit 10
#define AccurateFindingDegreeBelowLimit -10
#define IntialState 0
#define IntialStopAndResetState 1
#define AccurateFindingState 2
#define	AccurateStopAndResetState 3
#define EndState 4
/*-----------------------------End:Initial Macro------------------------------*/
/*-----------------------------Begin:PID BLDC Macro---------------------------*/
#define BLDCPropotion				0.2
#define BLDCIntergral				10
#define BLDCDerivative				0
#define BLDCAlpha					0
#define BLDCDeltaT					0.001
#define BLDCClockWise				1
#define BLDCCounterClockWise		0
#define BLDCIntergralAboveLimit		1000
#define BLDCIntergralBelowLimit	   -1000
#define BLDCSumAboveLimit			1000
#define BLDCSumBelowLimit		   -1000
#define BLDCEncoderPerRound			200
#define BLDCGearRatio 				2.5

//double BLDCPropotion;
//double BLDCIntergral;
//double BLDCDerivative;
/*-----------------------------End:PID BLDC Macro-----------------------------*/

/*-----------------------------Begin:PID DC Macro(SPEED)----------------------*/
#define DCProportion 			5
#define DCIntegral				300
#define DCDerivatite			0
#define DCAlpha					0
#define DCDeltaT				0.001
#define DCClockWise				1
#define DCCounterClockWise 	   -1
#define DCStop					0
#define DCIntegralAboveLimit	1000
#define DCIntegralBelowLimit	-1000
#define DCSumAboveLimit 		1000
#define DCSumBelowLimit			-1000
#define DCEncoderPerRound		1000
#define DCGearRatio				3.535

/*-----------------------------End:PID DC Macro(SPEED)------------------------*/


/*-----------------------------Begin:PID DC Macro(POS)------------------------*/

#define DCProportionPOS 				10
#define DCIntegralPOS					0
#define DCDerivatitePOS					0
#define DCAlphaPOS						0
#define DCDeltaTPOS						0.001
#define DCIntegralAboveLimitPOS			1000
#define DCIntegralBelowLimitPOS			-1000
#define DCSumAboveLimitPOS				1000
#define DCSumBelowLimitPOS				-1000

/*-----------------------------End:PID DC Macro(POS)--------------------------*/
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;

osThreadId PID_BLDCHandle;
osThreadId PID_DCHandle;
osThreadId LogicTaskHandle;
/* USER CODE BEGIN PV */
EncoderRead ENC_BLDC;
EncoderRead ENC_DC;

MotorDrive BLDC;
MotorDrive DC;

PID_Param PID_BLDC;
PID_Param PID_DC_SPEED;
PID_Param PID_DC_POS;


double SpeedTestBLDC;

double SpeedTest_DC_Speed;
double SpeedTest_DC_POS;

float BLDCSpeed;
float DCPos;

double Degree;

uint8_t HomeStatus, HomeFound, RunStatus;
int8_t dir=1;
int countX1;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM4_Init(void);
void StartPID_BLDC(void const * argument);
void StartPID_DC(void const * argument);
void StartLogicTask(void const * argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void PIDBLDC(void){
	SpeedReadOnly(&ENC_BLDC);

	Pid_Cal(&PID_BLDC, SpeedTestBLDC, ENC_BLDC.vel_Real);

	BLDC_Drive_RedBoard(&BLDC, &htim2, PID_BLDC.u, TIM_CHANNEL_2);
}

void PIDDCSPEED(void){
	SpeedReadNonReset(&ENC_DC);

	Pid_Cal(&PID_DC_SPEED, SpeedTest_DC_Speed, ENC_DC.vel_Real);

	DC_Drive_BTS(&DC, &htim2, motor_Reserve, PID_DC_SPEED.u, TIM_CHANNEL_3, TIM_CHANNEL_4);
}

void PIDDCPOS(void){
	Pid_Cal(&PID_DC_POS, SpeedTest_DC_POS, CountRead(&ENC_DC, count_ModeDegree));

	SpeedTest_DC_Speed = PID_DC_POS.u;

	PIDDCSPEED();
}

void SetAndResetU2parameter(uint8_t command){
	if(!command){
		PID_DC_SPEED.uI_AboveLimit=0;
		PID_DC_SPEED.uI_BelowLimit=0;
		PID_DC_SPEED.u_AboveLimit=0;
		PID_DC_SPEED.u_BelowLimit=0;
		ENC_DC.vel_Pre = 0;
		ENC_DC.vel_Real = 0;
	}else{
		PID_DC_SPEED.uI_AboveLimit = DCIntegralAboveLimit;
		PID_DC_SPEED.uI_BelowLimit = DCIntegralBelowLimit;
		PID_DC_SPEED.u_AboveLimit = DCSumAboveLimit;
		PID_DC_SPEED.u_BelowLimit = DCSumBelowLimit;
	}
}

void HomeFinding()
{
	if(HAL_GPIO_ReadPin(Home_GPIO_Port, Home_Pin) == AtHome)	// if found home successfully
	{
		osDelay(1);
		if(HAL_GPIO_ReadPin(Home_GPIO_Port, Home_Pin) == AtHome)
		{
			HomeStatus = 1;
			if((RunStatus == IntialState) || (RunStatus == AccurateFindingState))
			{
				SpeedTest_DC_POS = CountRead(&ENC_DC, count_ModeDegree);
				RunStatus++;
			}
		}
	}

	if(RunStatus == IntialState)
	{
		if(CountRead(&ENC_DC, count_ModeDegree) > FindingDegreeAboveLimit)
		{
			dir = -1;
		}
		else if(CountRead(&ENC_DC, count_ModeDegree) < FindingDegreeBelowLimit)
		{
			dir = 1;
		}
		SpeedTest_DC_Speed = 30 * dir;
	}

	if((RunStatus == IntialStopAndResetState) || (RunStatus == AccurateStopAndResetState))
	{
		osDelay(500);
		SetAndResetU2parameter(0);
		ResetCount(&ENC_DC, 1);
		SpeedTest_DC_POS = 0;
		osDelay(10);
		ResetCount(&ENC_DC, 0);
		SetAndResetU2parameter(1);
		RunStatus++;
	}

	if((RunStatus == AccurateFindingState) && (HAL_GPIO_ReadPin(Home_GPIO_Port, Home_Pin) == NotAtHome))
	{
		if(CountRead(&ENC_DC, count_ModeDegree) > AccurateFindingDegreeAboveLimit)
		{
			dir = -1;
		}
		else if(CountRead(&ENC_DC, count_ModeDegree) < AccurateFindingDegreeBelowLimit)
		{
			dir = 1;
		}
		SpeedTest_DC_Speed = 2 * dir;
	}

	if(RunStatus == EndState)
	{
		HomeFound = 1;
	}
}

/* ------------------Begin: Receive Data From Main----------------------------*/

uint8_t Rx_Buf[41];

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	if(huart -> Instance == USART1)
	{
		HAL_UARTEx_ReceiveToIdle_DMA(&huart1, Rx_Buf, 41);
		if (Rx_Buf[40]==13)
		{
			DecodeData(Rx_Buf);
			memset(Rx_Buf,0,Size);
		}
		__HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);
	}
}

/* ------------------End: Receive Data From Main------------------------------*/

/* ------------------Begin: Decode Data From Main-----------------------------*/

#define ID 4
uint8_t Mode;
float BLDCSpeed;
float DCPos;
uint8_t *pFloat = NULL;

void DecodeData(uint8_t* dataArr)
{
	if(dataArr[0] == ID)
	{
		pFloat = &Mode;
		*(pFloat)=dataArr[1];
		pFloat = &BLDCSpeed;
		for(uint8_t i = 2; i < 6; i++)
		{
			*(pFloat+i-2)=dataArr[i];
		}
		pFloat = &DCPos;
		for(uint8_t i = 6; i < 10; i++)
		{
			*(pFloat+i-6)=dataArr[i];
		}
	}
	else if(dataArr[10] == ID)
	{
		pFloat = &Mode;
		*(pFloat)=dataArr[11];
		pFloat = &BLDCSpeed;
		for(uint8_t i = 12; i < 16; i++)
		{
			*(pFloat+i-12)=dataArr[i];
		}
		pFloat = &DCPos;
		for(uint8_t i = 16; i < 20; i++)
		{
			*(pFloat+i-16)=dataArr[i];
		}
	}else if(dataArr[20] == ID)
	{
		pFloat = &Mode;
		*(pFloat)=dataArr[21];
		pFloat = &BLDCSpeed;
		for(uint8_t i = 22; i < 26; i++)
		{
			*(pFloat+i-22)=dataArr[i];
		}
		pFloat = &DCPos;
		for(uint8_t i = 26; i < 30; i++)
		{
			*(pFloat+i-26)=dataArr[i];
		}
	}else if(dataArr[30] == ID)
	{
		pFloat = &Mode;
		*(pFloat)=dataArr[31];
		pFloat = &BLDCSpeed;
		for(uint8_t i = 32; i < 36; i++)
		{
			*(pFloat+i-32)=dataArr[i];
		}
		pFloat = &DCPos;
		for(uint8_t i = 36; i < 40; i++)
		{
			*(pFloat+i-36)=dataArr[i];
		}
	}
}

/* ------------------End: Decode Data From Main-------------------------------*/

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
  MX_DMA_Init();
  MX_TIM3_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  /* -------------------Begin: Init PWM CHANNELS------------------------------*/
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
  /* -------------------End: Init PWM CHANNELS--------------------------------*/

  /* -------------------Begin: Init Interrupt Encoder's Channels--------------*/
  HAL_TIM_Encoder_Start_IT(&htim3, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start_IT(&htim4, TIM_CHANNEL_ALL);
  /* -------------------End: Init Interrupt Encoder's Channels----------------*/

  /* -------------------Begin: Setting BLDC-----------------------------------*/
  EncoderSetting(&ENC_BLDC, &htim4, BLDCEncoderPerRound*BLDCGearRatio, BLDCDeltaT);
  Pid_SetParam(&PID_BLDC, BLDCPropotion, BLDCIntergral, BLDCDerivative, BLDCAlpha, BLDCDeltaT, BLDCIntergralAboveLimit, BLDCIntergralBelowLimit, BLDCSumAboveLimit, BLDCSumBelowLimit);
  /* -------------------End: Setting BLDC-----------------------------------*/


  EncoderSetting(&ENC_DC, &htim3, DCEncoderPerRound*DCGearRatio, DCDeltaT);
  Pid_SetParam(&PID_DC_SPEED, DCProportion, DCIntegral, DCDerivatite, DCAlpha, DCDeltaT, DCIntegralAboveLimit, DCIntegralBelowLimit, DCSumAboveLimit, DCSumBelowLimit);
  Pid_SetParam(&PID_DC_POS, DCProportionPOS, DCIntegralPOS, DCDerivatitePOS, DCAlphaPOS, DCDeltaTPOS, DCIntegralAboveLimitPOS, DCIntegralBelowLimitPOS, DCSumAboveLimitPOS, DCSumBelowLimitPOS);

  HAL_UARTEx_ReceiveToIdle_DMA(&huart1, Rx_Buf, 41);
  __HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);
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
  /* definition and creation of PID_BLDC */
  osThreadDef(PID_BLDC, StartPID_BLDC, osPriorityBelowNormal, 0, 128);
  PID_BLDCHandle = osThreadCreate(osThread(PID_BLDC), NULL);

  /* definition and creation of PID_DC */
  osThreadDef(PID_DC, StartPID_DC, osPriorityAboveNormal, 0, 128);
  PID_DCHandle = osThreadCreate(osThread(PID_DC), NULL);

  /* definition and creation of LogicTask */
  osThreadDef(LogicTask, StartLogicTask, osPriorityNormal, 0, 128);
  LogicTaskHandle = osThreadCreate(osThread(LogicTask), NULL);

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 6;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
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
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DirBLDC_GPIO_Port, DirBLDC_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : DirBLDC_Pin */
  GPIO_InitStruct.Pin = DirBLDC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DirBLDC_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Home_Pin */
  GPIO_InitStruct.Pin = Home_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Home_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartPID_BLDC */
/**
  * @brief  Function implementing the PID_BLDC thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartPID_BLDC */
void StartPID_BLDC(void const * argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
//	SpeedReadNonReset(&ENC_BLDC);

	PIDBLDC();
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartPID_DC */
/**
* @brief Function implementing the PID_DC thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartPID_DC */
void StartPID_DC(void const * argument)
{
  /* USER CODE BEGIN StartPID_DC */
  /* Infinite loop */
  for(;;)
  {
//	SpeedReadNonReset(&ENC_DC);
	if((HomeStatus) && (RunStatus != 2))
	{
		PIDDCPOS();
	}
	else
	{
		PIDDCSPEED();
	}
//	PIDDCPOS();
    osDelay(1);
  }
  /* USER CODE END StartPID_DC */
}

/* USER CODE BEGIN Header_StartLogicTask */
/**
* @brief Function implementing the LogicTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartLogicTask */
void StartLogicTask(void const * argument)
{
  /* USER CODE BEGIN StartLogicTask */
  /* Infinite loop */
  for(;;)
  {
	if(HomeFound == 0)
	{
		HomeFinding();
	}
	else
	{
		SpeedTest_DC_POS = DCPos;
		SpeedTest_DC_Speed = BLDCSpeed;
	}
    osDelay(1);
  }
  /* USER CODE END StartLogicTask */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
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
