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
#include "CAN_Control.h"
#include "CAN_FuncHandle.h"
#include "stdbool.h"
#include "PID.h"
#include "Flag.h"
#include "InverseKinematic.h"
#include "SwerveModule.h"
#include "string.h"
#include "Gamepad.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef enum EnableFuncHandle{
	ENABLE_START,
	ENABLE_SETHOME,
	ENABLE_BREAK_PROTECTION,
	ENABLE_ANGLE_SPEED,
	ENABLE_CAN_TX,
	ENABLE_END,
}EnableFuncHandle;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define TARGET_FLAG_GROUP enableFlag
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart3;

osThreadId defaultTaskHandle;
osThreadId TaskInvKineHandle;
uint32_t TaskInvKineBuffer[ 256 ];
osStaticThreadDef_t TaskInvKineControlBlock;
osThreadId TaskCANHandle;
uint32_t TaskCANBuffer[ 128 ];
osStaticThreadDef_t TaskCANControlBlock;
osThreadId TaskActuatorHandle;
osThreadId TaskGamePadHandle;
/* USER CODE BEGIN PV */
bool enableSendSpeedAndAngle = false;
bool enableSendPID = false;
bool enableSendTestMode = false;
bool enableSendBrake = false;

uint8_t testMode = 1;
uint8_t brake = 1;
float bldcSpeed = 50;
float dcAngle = 0;
CAN_DEVICE_ID targetID = CANCTRL_DEVICE_MOTOR_CONTROLLER_4;
PID_Param pid;
PID_type type = PID_BLDC_SPEED;

bool enableSendBreakProtection = false;

uint32_t enableFlag = 0;

EnableFuncHandle mode = ENABLE_ANGLE_SPEED;
CAN_DEVICE_ID Device_ID = CANCTRL_DEVICE_MOTOR_CONTROLLER_4;
CAN_MODE_ID Mode_ID = CANCTRL_MODE_PID_BLDC_SPEED;
PID_Param targetPID = {
	.deltaT = 0.001,
	.kP = 10,
	.kI = 10,
	.kD = 1,
	.alpha = 1,
};
PID_type pidType = PID_BLDC_SPEED;

uint8_t UARTRX3_Buffer[9];
uint8_t DataTayGame[9];

float Xleft,Yleft;
float Xright;


_GamePad GamePad;
uint32_t gamepadRxIsBusy = 0;
float u,v,r;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART3_UART_Init(void);
void StartDefaultTask(void const * argument);
void InverseKinematic(void const * argument);
void CAN_Bus(void const * argument);
void Actuator(void const * argument);
void GamePadHandle(void const * argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


void enfunc_SetFlag(CAN_MODE_ID e){SETFLAG(TARGET_FLAG_GROUP,e);}
bool enfunc_CheckFlag(CAN_MODE_ID e){return CHECKFLAG(TARGET_FLAG_GROUP,e);}
void enfunc_ClearFlag(CAN_MODE_ID e){CLEARFLAG(TARGET_FLAG_GROUP,e);}


void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan){
	 canctrl_Receive(hcan, CAN_RX_FIFO0);
}
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan){
	canctrl_Receive(hcan, CAN_RX_FIFO1);
}


void SendSpeedAndRotation(CAN_DEVICE_ID targetID,float Speed, float Angle)
{
	if(!enfunc_CheckFlag(ENABLE_ANGLE_SPEED)) return;
	CAN_SpeedBLDC_AngleDC speedAngle;
	speedAngle.bldcSpeed = Speed;
	speedAngle.dcAngle = Angle;
	canfunc_MotorPutSpeedAndAngle(speedAngle);
	canctrl_Send(&hcan1, targetID);
	enfunc_ClearFlag(ENABLE_ANGLE_SPEED);
}

void SetTestMode(bool testMode, CAN_DEVICE_ID targetID,bool *enable){
	if(!*enable) return;
	canfunc_SetBoolValue(testMode,CANCTRL_MODE_TEST);
	canctrl_Send(&hcan1, targetID);
	*enable = false;
}

void SetBrake(bool brake, CAN_DEVICE_ID targetID,bool *enable){
	if(!*enable) return;
	canfunc_SetBoolValue(brake,CANCTRL_MODE_MOTOR_BLDC_BRAKE);
	canctrl_Send(&hcan1, targetID);
	*enable = false;
}

void SetHome(CAN_DEVICE_ID targetID)
{
	if(!enfunc_CheckFlag(ENABLE_SETHOME))return;
	canfunc_SetBoolValue(1,CANCTRL_MODE_SET_HOME);
	canctrl_Send(&hcan1, targetID);
	enfunc_ClearFlag(ENABLE_SETHOME);
}

void BreakProtection(CAN_DEVICE_ID targetID)
{
	if(!enfunc_CheckFlag(ENABLE_BREAK_PROTECTION)) return;
	canfunc_SetBoolValue(1, CANCTRL_MODE_PID_BLDC_BREAKPROTECTION);
	canctrl_Send(&hcan1, targetID);
	osDelay(1000);
	canfunc_SetBoolValue(0, CANCTRL_MODE_PID_BLDC_BREAKPROTECTION);
	canctrl_Send(&hcan1, targetID);
	enfunc_ClearFlag(ENABLE_BREAK_PROTECTION);
}

void canTxHandleFunc(CAN_MODE_ID mode,CAN_DEVICE_ID targetID){
	switch(mode){
	case CANCTRL_MODE_SET_HOME :
		SetHome(targetID);
		break;
	case CANCTRL_MODE_LED_BLUE :
		break;
	case CANCTRL_MODE_SHOOT :
		break;
	case CANCTRL_MODE_ENCODER :
		break;
	case CANCTRL_MODE_MOTOR_SPEED_ANGLE :
		SendSpeedAndRotation(Device_ID,bldcSpeed,dcAngle);
		break;
	case CANCTRL_MODE_MOTOR_BLDC_BRAKE :
		break;
	case CANCTRL_MODE_PID_DC_SPEED :
	case CANCTRL_MODE_PID_DC_ANGLE :
	case CANCTRL_MODE_PID_BLDC_SPEED :
		canfunc_PutAndSendParamPID(&hcan1, Device_ID,targetPID,pidType);
		break;
	case CANCTRL_MODE_PID_BLDC_BREAKPROTECTION :
		BreakProtection(targetID);
		break;
	case CANCTRL_MODE_START :
		break;
	case CANCTRL_MODE_TEST :
		break;
	case CANCTRL_MODE_END :
		break;
	}
}




void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	if(huart->Instance == USART3){
		gamepadRxIsBusy = 1;
		int ViTriData = -1;
		for(int i = 0; i <= 8; ++i){
			if(UARTRX3_Buffer[i] == 0xAA){
				ViTriData = i;
			}
		}
		if(ViTriData != -1){
			int cnt = 0;
			while(cnt < 9){
				DataTayGame[cnt] = UARTRX3_Buffer[ViTriData];
				++ViTriData;
				if(ViTriData >= 9){
					ViTriData = 0;
				}
				++cnt;
			}

			GamepPadHandle(&GamePad,DataTayGame);


		}
		else{
			GamePad.Status = 0;
		}
		if(!gamepadRxIsBusy) HAL_UART_Receive_IT(&huart3, (uint8_t*)UARTRX3_Buffer, 9);

	}
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart){
	__HAL_UART_CLEAR_OREFLAG(huart);
	memset(UARTRX3_Buffer,0,sizeof(UARTRX3_Buffer));
	 HAL_UART_Receive_IT(&huart3, (uint8_t*)UARTRX3_Buffer, 9);
	__HAL_UART_DISABLE(huart);
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
  MX_CAN1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Encoder_Start_IT(&htim1, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start_IT(&htim2, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start_IT(&htim3, TIM_CHANNEL_ALL);

  HAL_UART_Receive_IT(&huart3, (uint8_t*)UARTRX3_Buffer, 9);
  HAL_CAN_Start(&hcan1);
  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_RX_FIFO1_MSG_PENDING);
  pid.kP = -0.12;
  pid.kI = 5.32;
  pid.kD = 20.22;
  pid.alpha = 5.31;
  pid.deltaT = 0.001;
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
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityIdle, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of TaskInvKine */
  osThreadStaticDef(TaskInvKine, InverseKinematic, osPriorityLow, 0, 256, TaskInvKineBuffer, &TaskInvKineControlBlock);
  TaskInvKineHandle = osThreadCreate(osThread(TaskInvKine), NULL);

  /* definition and creation of TaskCAN */
  osThreadStaticDef(TaskCAN, CAN_Bus, osPriorityBelowNormal, 0, 128, TaskCANBuffer, &TaskCANControlBlock);
  TaskCANHandle = osThreadCreate(osThread(TaskCAN), NULL);

  /* definition and creation of TaskActuator */
  osThreadDef(TaskActuator, Actuator, osPriorityAboveNormal, 0, 128);
  TaskActuatorHandle = osThreadCreate(osThread(TaskActuator), NULL);

  /* definition and creation of TaskGamePad */
  osThreadDef(TaskGamePad, GamePadHandle, osPriorityLow, 0, 128);
  TaskGamePadHandle = osThreadCreate(osThread(TaskGamePad), NULL);

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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
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
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 21;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_2TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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
  htim2.Init.Period = 65535;
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
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
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
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
bool TestFlag;
void FlagEnable(){
	if (TestFlag){
		enfunc_SetFlag(mode);
		TestFlag = false;
		canTxHandleFunc(Mode_ID, Device_ID);
	}
}

typedef void (*ptnCpltCallback)(ModuleID, float, float);

void InvCpltCallback(ModuleID ID, float speed, float angle)
{
	CAN_SpeedBLDC_AngleDC speedAngle;
	speedAngle.bldcSpeed = speed;
	speedAngle.dcAngle = angle;
	canfunc_MotorPutSpeedAndAngle(speedAngle);
	canctrl_Send(&hcan1, (CAN_DEVICE_ID)ID);
	while(canctrl_Send(&hcan1, ID) != HAL_OK);
}

void InitialSetHome()
{
	CAN_SpeedBLDC_AngleDC speedAngle;
	speedAngle.bldcSpeed = 0;
	speedAngle.dcAngle = 0;
	for (CAN_DEVICE_ID i = CANCTRL_DEVICE_MOTOR_CONTROLLER_1;i<=CANCTRL_DEVICE_MOTOR_CONTROLLER_4;i++){
			canfunc_MotorPutSpeedAndAngle(speedAngle);
			while(canctrl_Send(&hcan1, i) != HAL_OK);
		}
	osDelay(1000);
	for (CAN_DEVICE_ID i = CANCTRL_DEVICE_MOTOR_CONTROLLER_1;i<=CANCTRL_DEVICE_MOTOR_CONTROLLER_4;i++){
			canfunc_SetBoolValue(1, CANCTRL_MODE_SET_HOME);
			while(canctrl_Send(&hcan1, i) != HAL_OK);
		}
}

void TestBreakProtection()
{
	for (CAN_DEVICE_ID i = CANCTRL_DEVICE_MOTOR_CONTROLLER_1;i<=CANCTRL_DEVICE_MOTOR_CONTROLLER_4;i++){
		canfunc_SetBoolValue(1, CANCTRL_MODE_PID_BLDC_BREAKPROTECTION);
		canctrl_Send(&hcan1, i);
		osDelay(1);
	}
	osDelay(1000);
	for (CAN_DEVICE_ID i = CANCTRL_DEVICE_MOTOR_CONTROLLER_1;i<=CANCTRL_DEVICE_MOTOR_CONTROLLER_4;i++){
		canfunc_SetBoolValue(0, CANCTRL_MODE_PID_BLDC_BREAKPROTECTION);
		canctrl_Send(&hcan1, i);
		osDelay(1);
	}
}

void TestSendPID()
{
	for(uint8_t i = 0; i<3;i++)
	{
//		invkine_SetInvCalStep(1);
		canfunc_PutAndSendParamPID(&hcan1, Device_ID,targetPID,pidType);
		osDelay(1);
	}
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
//	swer_Init();
//	TestBreakProtection();
  for(;;)
  {
//	  if(GamePad.Circle == 1)
//	  {
//		  TestBreakProtection();
//	  }else{
//			u = GamePad.YLeftCtr;
//			v = GamePad.XLeftCtr;
//			r = -GamePad.XRightCtr*M_PI/180;
//
//			invkine_Implementation(MODULE_ID_1, u, v, r, &InvCpltCallback);
//			invkine_Implementation(MODULE_ID_2, u, v, r, &InvCpltCallback);
			invkine_Implementation(MODULE_ID_3, u, v, r, &InvCpltCallback);
//			invkine_Implementation(MODULE_ID_4, u, v, r, &InvCpltCallback);
//	  }
//
//
	if(gamepadRxIsBusy) {
		gamepadRxIsBusy = 0;
		HAL_UART_Receive_IT(&huart3, (uint8_t*)UARTRX3_Buffer, 9);
	}
	if((huart3.Instance->CR1 & USART_CR1_UE) == 0){
		__HAL_UART_ENABLE(&huart3);
	}
    osDelay(50);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_InverseKinematic */
/**
* @brief Function implementing the TaskInvKine thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_InverseKinematic */
void InverseKinematic(void const * argument)
{
  /* USER CODE BEGIN InverseKinematic */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END InverseKinematic */
}

/* USER CODE BEGIN Header_CAN_Bus */
/**
* @brief Function implementing the TaskCAN thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_CAN_Bus */
void CAN_Bus(void const * argument)
{
  /* USER CODE BEGIN CAN_Bus */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END CAN_Bus */
}

/* USER CODE BEGIN Header_Actuator */
/**
* @brief Function implementing the TaskActuator thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Actuator */
void Actuator(void const * argument)
{
  /* USER CODE BEGIN Actuator */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Actuator */
}

/* USER CODE BEGIN Header_GamePadHandle */
/**
* @brief Function implementing the TaskGamePad thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_GamePadHandle */
void GamePadHandle(void const * argument)
{
  /* USER CODE BEGIN GamePadHandle */
//	uint32_t gamePadhandle;
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END GamePadHandle */
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
