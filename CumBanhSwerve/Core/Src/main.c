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
#include "stdlib.h"
#include "PID.h"
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

/*---------------------------------------------------------------------------------------------------------------------------------------------*/

/*-----------------------------Begin:PID BLDC Macro---------------------------*/
#define BLDCProportion 0.2
#define BLDCIntegral 10
#define BLDCDerivative	0
#define BLDCAlpha 0
#define BLDCDeltaT 0.001
#define BLDCClockWise 1
#define BLDCCounterClockWise 0
#define BLDCIntegralAboveLimit 1000
#define BLDCIntegralBelowLimit -1000
#define BLDCSumAboveLimit 1000
#define BLDCSumBelowLimit -1000
#define _BLDCEncoderPerRound 2000
#define _BLDCGearRatio 	2.5


double target_BLDC_Speed;
/*-----------------------------End:PID BLDC Macro-----------------------------*/

/*-----------------------------Begin:PID DC Macro(SPEED)----------------------*/
#define DCProportion 20
#define DCIntegral 500
#define DCDerivative 0
#define DCAlpha 0
#define POS 0.001
#define DCDeltaT 0.001
#define DCClockWise -1
#define DCStop 0
#define DCCounterClockWise 1
#define DCIntegralAboveLimit 1000
#define DCIntegralBelowLimit -1000
#define DCSumAboveLimit 1000
#define DCSumBelowLimit -1000
#define _DCEncoderPerRound 68000

double target_DC_SPEED;
/*-----------------------------End:PID DC Macro(SPEED)------------------------*/

/*-----------------------------Begin:PID DC Macro(POS)------------------------*/
#define DCProportionPOS 5
#define DCIntegralPOS 0
#define DCDerivativePOS 0
#define DCAlphaPOS 0
#define DCDeltaTPOS 0.001
#define DCIntegralAboveLimitPOS 1000
#define DCIntegralBelowLimitPOS -1000
#define DCSumAboveLimitPOS 1000
#define DCSumBelowLimitPOS -1000
#define FilterAlpha 0

double target_DC_POS;
/*-----------------------------End:PID DC Macro(POS)--------------------------*/

uint8_t SlaveBuff[10];


/*---------------------------------------------------------------------------------------------------------------------------------------------*/
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;

osThreadId CalPIDDCHandle;
osThreadId CalPIDBLDCHandle;
osThreadId LogicControlHandle;
/* USER CODE BEGIN PV */
/*---------------------------------------------------------------------------------------------------------------------------------------------*/

/*-----------------------------Begin:Home Variables----------------------------*/

uint8_t HomeStatus;
uint8_t RunStatus;
uint8_t IntialSpeed;
uint8_t HomeFound;
uint8_t Testcommand;
int TestComand2;

/*-----------------------------End:Home Variables------------------------------*/

/*-----------------------------Begin:Encoder Read Variables--------------------*/

int8_t RotateStatus = 1;

/*-----------------------------End:Encoder Read Variables----------------------*/

/*---------------------------------------------------------------------------------------------------------------------------------------------*/

PID_Param PID_BLDC;
PID_Param PID_DC_SPEED;
PID_Param PID_DC_POS;

EncoderRead ENC_BLDC;
EncoderRead ENC_DC;

MotorDrive BLDC;
MotorDrive DC;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART1_UART_Init(void);
void StartCalPIDDC(void const * argument);
void StartCalPIDBLDC(void const * argument);
void StartLogicControl(void const * argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//-----------------------------------------------------------------------------------------------------------------//
//----------------------------------------------------BEGIN: PID MOTOR---------------------------------------------//
//-----------------------------------------------------------------------------------------------------------------//
void PIDBLDC(void)
{
	//Calculate the speed of BLD
	SpeedReadOnly(&ENC_BLDC);
	//Calculate PID of BLDC
	Pid_Cal(&PID_BLDC,target_BLDC_Speed,ENC_BLDC.vel_Real);
	//Drive Motor
	BLDC_Drive_RedBoard(&BLDC,&htim2,PID_BLDC.u,TIM_CHANNEL_2);
}

void PIDDCSpeed(void)
{
	//Calculate velocity of DC Servo:
	SpeedReadNonReset(&ENC_DC);
	//Calculate PID of DC speed:
	Pid_Cal(&PID_DC_SPEED,target_DC_SPEED,ENC_DC.vel_Real);
	//Drive Motor
	DC_Drive_BTS(&DC,&htim2,motor_Reserve,PID_DC_SPEED.u,TIM_CHANNEL_3,TIM_CHANNEL_4);

}

void PIDDCPos(){
	//Calculate the Pos of DC Servo:
	Pid_Cal(&PID_DC_POS,target_DC_POS,CountRead(&ENC_DC,count_ModeDegree));
	//Control the Speed PID:
	target_DC_SPEED = PID_DC_POS.u;
	PIDDCSpeed();

}

//-----------------------------------------------------------------------------------------------------------------//
//----------------------------------------------------END: PID MOTOR-----------------------------------------------//
//-----------------------------------------------------------------------------------------------------------------//



/*---------------------------------------------------------------------------------------------------------------------------------------------*/

//-----------------------------------------------------------------------------------------------------------------//
//----------------------------------------------------BEGIN: RESET MOTOR-------------------------------------------//
//-----------------------------------------------------------------------------------------------------------------//


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


//-----------------------------------------------------------------------------------------------------------------//
//----------------------------------------------------END: RESET MOTOR---------------------------------------------//
//-----------------------------------------------------------------------------------------------------------------//

//-----------------------------------------------------------------------------------------------------------------//
//----------------------------------------------------BEGIN: Home Finding------------------------------------------//
//-----------------------------------------------------------------------------------------------------------------//


void HomeFinding(){
//  // Reading home sensor if the  wheel at home or not
//  // if at home the wheel won't run and set the initial Degree to 0
//  // if not the wheel will find home
  if(HAL_GPIO_ReadPin(Home_GPIO_Port, Home_Pin)== AtHome){
	osDelay(1);
	if(HAL_GPIO_ReadPin(Home_GPIO_Port, Home_Pin)== AtHome){
		HomeStatus = 1;
		if((RunStatus == IntialState)||(RunStatus == AccurateFindingState)){
			target_DC_POS = CountRead(&ENC_DC,count_ModeDegree);
			RunStatus++;
		}
	  }
	}

  // At this State the Wheel will find its home at high speed
  // And will reserve when it reach the Degree limits not to break the wires
  if(RunStatus == IntialState){
	if(CountRead(&ENC_DC,count_ModeDegree) > FindingDegreeAboveLimit){
		RotateStatus = DCClockWise;
	}else if(CountRead(&ENC_DC,count_ModeDegree)< FindingDegreeBelowLimit){
		RotateStatus = DCCounterClockWise;
	}
	target_DC_SPEED = IntialFindingSpeed*RotateStatus;
  }

 //At this State the Wheel will stop and Reset to 0 Degree
if((RunStatus == IntialStopAndResetState)||(RunStatus == AccurateStopAndResetState)){
	osDelay(500);
	SetAndResetU2parameter(0);
	ResetCount(&ENC_DC,1);
	target_DC_POS = 0;
	osDelay(10);
	ResetCount(&ENC_DC,0);
	SetAndResetU2parameter(1);
	RunStatus += 1;
}

//At this State the wheel will run at low speed to find its home
//And also reserve when its reach the degree limits
if ((RunStatus == AccurateFindingState)&&(HAL_GPIO_ReadPin(Home_GPIO_Port, Home_Pin) == NotAtHome)){
	if(CountRead(&ENC_DC,count_ModeDegree) < AccurateFindingDegreeBelowLimit){
		RotateStatus = DCCounterClockWise;
	}else if(CountRead(&ENC_DC,count_ModeDegree) > AccurateFindingDegreeAboveLimit){
		RotateStatus = DCClockWise;
	}
	target_DC_SPEED = AccurateFindingSpeed*RotateStatus;
	}
//End Home Finding
if (RunStatus == EndState){
	HomeFound = 1;
}
}

//-----------------------------------------------------------------------------------------------------------------//
//----------------------------------------------------END: Home Finding--------------------------------------------//
//-----------------------------------------------------------------------------------------------------------------//Mode
uint8_t *pByte = NULL;

uint8_t Mode;
float SpeedBLDC;
float DCDegree;
void MergeByteToFloat(uint8_t* dataArray){
	uint8_t *pFloat=NULL;
	pFloat = &Mode;
	*(pFloat)=dataArray[0];
	pFloat = &SpeedBLDC;
	for(uint8_t i = 1; i < 5;i++)
	{
		*(pFloat+i-1)=dataArray[i];
	}
	pFloat = &DCDegree;
	for(uint8_t i = 5; i < 9;i++)
	{
		*(pFloat+i-5)=dataArray[i];
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  HAL_UART_Receive_IT(&huart1, SlaveBuff, 10);
  MergeByteToFloat(SlaveBuff);
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
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_4);
  HAL_TIM_Encoder_Start_IT(&htim4, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start_IT(&htim3, TIM_CHANNEL_ALL);
//  HAL_TIM_Base_Start_IT(&htim1);

  //Set intial Speed of both Motor as 0 RPM

  EncoderSetting(&ENC_BLDC,&htim4,_BLDCEncoderPerRound*2.5,BLDCDeltaT);
  EncoderSetting(&ENC_DC,&htim3,_DCEncoderPerRound,DCDeltaT);

  Pid_SetParam(&PID_BLDC,BLDCProportion,BLDCIntegral,BLDCDerivative,BLDCAlpha,BLDCDeltaT,BLDCIntegralAboveLimit,BLDCIntegralBelowLimit,BLDCSumAboveLimit,BLDCSumBelowLimit);
  Pid_SetParam(&PID_DC_SPEED,DCProportion,DCIntegral,DCDerivative,DCAlpha,DCDeltaT,DCIntegralAboveLimit,DCIntegralBelowLimit,DCSumAboveLimit,DCSumBelowLimit);
  Pid_SetParam(&PID_DC_POS,DCProportionPOS,DCIntegralPOS,DCDerivativePOS,DCAlphaPOS,DCDeltaTPOS,DCIntegralAboveLimitPOS,DCIntegralBelowLimitPOS,DCSumAboveLimitPOS,DCSumBelowLimitPOS);

  HAL_UART_Receive_IT (&huart1, SlaveBuff, 10);
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
  /* definition and creation of CalPIDDC */
  osThreadDef(CalPIDDC, StartCalPIDDC, osPriorityAboveNormal, 0, 128);
  CalPIDDCHandle = osThreadCreate(osThread(CalPIDDC), NULL);

  /* definition and creation of CalPIDBLDC */
  osThreadDef(CalPIDBLDC, StartCalPIDBLDC, osPriorityBelowNormal, 0, 128);
  CalPIDBLDCHandle = osThreadCreate(osThread(CalPIDBLDC), NULL);

  /* definition and creation of LogicControl */
  osThreadDef(LogicControl, StartLogicControl, osPriorityNormal, 0, 128);
  LogicControlHandle = osThreadCreate(osThread(LogicControl), NULL);

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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 7-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1000-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

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
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(Home_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartCalPIDDC */
/**
  * @brief  Function implementing the CalPIDDC thread.
  * @param  argument: Not used
  * @retval None
  */

/* USER CODE END Header_StartCalPIDDC */
void StartCalPIDDC(void const * argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {


	if((HomeStatus)&&(RunStatus != AccurateFindingState)){
		PIDDCPos();
	}
	else{
		PIDDCSpeed();
	}

	osDelay(1);

  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartCalPIDBLDC */
/**
* @brief Function implementing the CalPIDBLDC thread.
* @param argument: Not used
* @retval None
*/
int AntiProtection;
/* USER CODE END Header_StartCalPIDBLDC */
void StartCalPIDBLDC(void const * argument)
{
  /* USER CODE BEGIN StartCalPIDBLDC */
  /* Infinite loop */
  for(;;)
  {
	if(AntiProtection == 1){
		PID_BLDC.uI = 0;
		PID_BLDC.uI_Pre = 0;
		PID_BLDC.u = 0;
		BLDC_Drive_RedBoard(&BLDC,&htim2,0,TIM_CHANNEL_2);
		osDelay(100);
		AntiProtection = 0;
	}else{
		PIDBLDC();
	}
	osDelay(1);
  }
  /* USER CODE END StartCalPIDBLDC */
}

/* USER CODE BEGIN Header_StartLogicControl */
/**
* @brief Function implementing the LogicControl thread.
* @param argument: Not used
* @retval None
*/

int TestDegree[] = {90,180,50,0,60,0,-90,10};
int TestSpeed[] = {100,0,120,200,100,200,100,0};

/* USER CODE END Header_StartLogicControl */
void StartLogicControl(void const * argument)
{
  /* USER CODE BEGIN StartLogicControl */
  /* Infinite loop */
  for(;;)
  {
	if(HomeFound == 0)
	{
		HomeFinding();
	}
	else
	{
		target_BLDC_Speed = SpeedBLDC;
		target_DC_POS = DCDegree;
	}
    osDelay(1);
  }


  /* USER CODE END StartLogicControl */
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
