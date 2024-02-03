/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
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
#include "PID_GunModule.h"
#include "CAN_Control.h"
#include "CAN_FuncHandle.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define FLASH_ADDR_BASE 0x08000000
#define FLASH_ADDR_TARGET_PAGE 64
#define FLASH_ADDR_TARGET (FLASH_ADDR_BASE + 1024*FLASH_ADDR_TARGET_PAGE)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

osThreadId defaultTaskHandle;
osThreadId PIDTaskHandle;
osThreadId CANTaskHandle;
/* USER CODE BEGIN PV */
int count1 = 0, count2 = 0, count3 = 0;
uint16_t pwm = 0;
uint8_t state = 0;
uint8_t fire = 0;
bool IsSetHome = false;
bool IsFirePhoenix = false;
QueueHandle_t qPID, qHome, qShoot;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_CAN_Init(void);
static void MX_TIM1_Init(void);
void StartDefaultTask(void const * argument);
void StartPIDTask(void const * argument);
void StartCANTask(void const * argument);

/* USER CODE BEGIN PFP */
////////////////////////////////////////////////////////
//****************************************************//
//*                     _oo0oo_                      *//
//*                    o8888888o                     *//
//*                    88` . '88                     *//
//*                    (| -_- |)                     *//
//*                     0\ = /0                      *//
//*                  ___/`---'\___                   *//
//*                  .' \\| |// '.                   *//
//*                / \\||| : |||// \                 *//
//*              / _||||| -:- |||||_ \               *//
//*                | | \\\ - /// | |                 *//
//*             |  \_| ''\---/'' |_/ |               *//
//*               \ .-\__ '-' ___/-. /               *//
//*            ___'. .' /--.--\ `. .'___             *//
//*         ."" '< `.___\_<|>_/___.' >' "".          *//
//*        | | : `- \`.;`\ _ /`;.`/ - ` : | |        *//
//*          \ \ `_. \_ __\ /__ _/ .-` / /           *//
//*   =====`-.____`.___ \_____/___.-`___.-'=====     *//
//*                     `=---='                      *//
//****************************************************//
////////////////////////////////////////////////////////
//---------------------ANTI BUG-----------------------//
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
	HAL_CAN_DeactivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
	CAN_MODE_ID modeID = canctrl_Receive_2(hcan, CAN_RX_FIFO0);
	BaseType_t HigherPriorityTaskWoken = pdFALSE;
	xTaskNotifyFromISR(CANTaskHandle, modeID, eSetValueWithOverwrite, &HigherPriorityTaskWoken);
	portYIELD_FROM_ISR(HigherPriorityTaskWoken);
}

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan) {
	HAL_CAN_DeactivateNotification(hcan, CAN_IT_RX_FIFO1_MSG_PENDING);
	CAN_MODE_ID modeID = canctrl_Receive_2(hcan, CAN_RX_FIFO1);
	BaseType_t HigherPriorityTaskWoken = pdFALSE;
	xTaskNotifyFromISR(CANTaskHandle, modeID, eSetValueWithOverwrite, &HigherPriorityTaskWoken);
	portYIELD_FROM_ISR(HigherPriorityTaskWoken);
}

extern BoardParameter_t brdParam;
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	if(GPIO_Pin == ECD3A_Pin){
		if(HAL_GPIO_ReadPin(ECD3B_GPIO_Port, ECD3B_Pin)){
			brdParam.encGun1.count_X1++;
		}
		else
			brdParam.encGun1.count_X1--;
	}
	if(GPIO_Pin == ECD1A_Pin){
		if(HAL_GPIO_ReadPin(ECD1B_GPIO_Port, ECD1B_Pin)){
			brdParam.encGun2.count_X1++;
		}
		else
			brdParam.encGun2.count_X1--;
	}
	BaseType_t HigherPriorityTaskWoken = pdFALSE;
	portYIELD_FROM_ISR(HigherPriorityTaskWoken);
}

void CAN_Init() {
	HAL_CAN_Start(&hcan);
	HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING
			| CAN_IT_RX_FIFO1_MSG_PENDING
			| CAN_IT_RX_FIFO0_FULL);
	uint16_t deviceID = *(__IO uint32_t*) FLASH_ADDR_TARGET << CAN_DEVICE_POS;
	canctrl_Filter_List16(&hcan,
			deviceID | CANCTRL_MODE_SHOOT,
			deviceID | CANCTRL_MODE_MOTOR_GUN_SPEED,
			deviceID | CANCTRL_MODE_MOTOR_GUN_ANGLE,
			deviceID | CANCTRL_MODE_TEST,
			0, CAN_RX_FIFO0);
	canctrl_Filter_List16(&hcan,
			deviceID | CANCTRL_MODE_PID_GUN2_SPEED,
			deviceID | CANCTRL_MODE_PID_GUN1_SPEED,
			deviceID | CANCTRL_MODE_PID_GUN_ANGLE,
			deviceID | CANCTRL_MODE_PID_ROTARY_SPEED,
			1, CAN_RX_FIFO0);
	canctrl_Filter_Mask16(&hcan,
			1 << CAN_RTR_REMOTE,
			0,
			1 << CAN_RTR_REMOTE,
			0,
			2,
			CAN_RX_FIFO0);

}

void can_GetPID_CompleteCallback(CAN_PID canPID, PID_type type) {
	PID_Param pid = brd_GetPID(type);
	canfunc_Convert_CAN_PID_to_PID_Param(canPID, &pid);
	brd_SetPID(pid, type);
}
uint8_t TestMode = 0;
void handleFunctionCAN(CAN_MODE_ID mode) {
	CAN_SpeedGun_Angle nodeSpeedAngle;
	CAN_SpeedGun_Angle speedAngle;
	switch (mode) {
		case CANCTRL_MODE_NODE_REQ_GUN_SPEED:
			nodeSpeedAngle.gunSpeed.gun1Speed = brd_GetCurrentSpeedGun1();
			nodeSpeedAngle.gunSpeed.gun1Speed = brd_GetCurrentSpeedGun2();
			canctrl_SetID(CANCTRL_MODE_NODE_REQ_GUN_SPEED);
			canctrl_PutMessage((void*)&nodeSpeedAngle.gunSpeed, sizeof(nodeSpeedAngle.gunSpeed));
			canctrl_Send(&hcan,*(__IO uint32_t*) FLASH_ADDR_TARGET);
			break;
		case CANCTRL_MODE_NODE_REQ_GUN_ANGLE:
			nodeSpeedAngle.gunAngle = brd_GetCurrentAngle();
			canctrl_SetID(CANCTRL_MODE_NODE_REQ_GUN_ANGLE);
			canctrl_PutMessage((void*)&nodeSpeedAngle.gunAngle, sizeof(nodeSpeedAngle.gunAngle));
			canctrl_Send(&hcan,*(__IO uint32_t*) FLASH_ADDR_TARGET);
			break;
		case CANCTRL_MODE_TEST:
			TestMode = canfunc_GetBoolValue();
			break;
		case CANCTRL_MODE_MOTOR_GUN_ANGLE:
			speedAngle= canfunc_GunGetSpeedAndAngle();
			brd_SetTargetRotaryAngle(speedAngle.gunAngle);
			break;
		case CANCTRL_MODE_MOTOR_GUN_SPEED:
			speedAngle= canfunc_GunGetSpeedAndAngle();
			brd_SetSpeedGun(speedAngle.gunSpeed.gun1Speed, MOTOR_GUN1);
			brd_SetSpeedGun(speedAngle.gunSpeed.gun2Speed, MOTOR_GUN2);
			break;
		case CANCTRL_MODE_PID_GUN_ANGLE:
			case CANCTRL_MODE_PID_GUN2_SPEED:
			case CANCTRL_MODE_PID_GUN1_SPEED:
			canfunc_GetPID(&can_GetPID_CompleteCallback);
		break;
		default:
		break;
	}
}

void handle_CAN_RTR_Response(CAN_HandleTypeDef *can, CAN_MODE_ID modeID) {
	PID_Param pid;
	switch (modeID) {
		case CANCTRL_MODE_SHOOT:
			bool shootValue = 1;
			xQueueSend(qShoot, (void*)&shootValue, 1/portTICK_PERIOD_MS);
		break;
		case CANCTRL_MODE_SET_HOME:
			bool setHomeValue = 1;
			xQueueSend(qHome, (void* )&setHomeValue, 1/portTICK_PERIOD_MS);
		break;
		case CANCTRL_MODE_PID_GUN2_SPEED:
			pid = brd_GetPID(PID_GUN2);
			canfunc_RTR_PID(can, pid, PID_GUN2);
		break;
		case CANCTRL_MODE_PID_GUN1_SPEED:
			pid = brd_GetPID(PID_GUN1);
			canfunc_RTR_PID(can, pid, PID_GUN1);
		break;
		case CANCTRL_MODE_PID_GUN_ANGLE:
			pid = brd_GetPID(PID_ROTARY_ANGLE);
			canfunc_RTR_PID(can, pid, PID_ROTARY_ANGLE);
		break;
		case CANCTRL_MODE_PID_ROTARY_SPEED:
			pid = brd_GetPID(PID_ROTARY_SPEED);
			canfunc_RTR_PID(can, pid, PID_ROTARY_SPEED);
		break;
		default:
		break;
	}
}

void SetHomeCompleteCallback() {
	Encoder_t enc = brd_GetObjEncRotary();
	encoder_ResetCount(&enc);
	brd_SetObjEncRotary(enc);
	canfunc_SetBoolValue(1, CANCTRL_MODE_SET_HOME_GUN);
	canctrl_Send(&hcan, *(__IO uint32_t*) FLASH_ADDR_TARGET);
}

void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan) {
	while (1);
}

void HAL_CAN_RxFifo0FullCallback(CAN_HandleTypeDef *hcan) {
	__NOP();
//	HAL_CAN_DeactivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
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
  MX_CAN_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  qHome  = xQueueCreate(1, sizeof(bool));
  qShoot = xQueueCreate(1, sizeof(bool));
  qPID   = xQueueCreate(2, sizeof(float));
  brd_Init();
//  HAL_FLASH_Unlock();
//  FLASH_EraseInitTypeDef er;
//  er.TypeErase = FLASH_TYPEERASE_PAGES;
//  er.PageAddress = FLASH_ADDR_TARGET;
//  er.NbPages = 1;
//  uint32_t pe = 0;
//  HAL_FLASHEx_Erase(&er, &pe);
//  HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, FLASH_ADDR_TARGET, CANCTRL_DEVICE_ACTUATOR_1);
//  HAL_FLASH_Lock();
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
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of PIDTask */
  osThreadDef(PIDTask, StartPIDTask, osPriorityNormal, 0, 128);
  PIDTaskHandle = osThreadCreate(osThread(PIDTask), NULL);

  /* definition and creation of CANTask */
  osThreadDef(CANTask, StartCANTask, osPriorityAboveNormal, 0, 128);
  CANTaskHandle = osThreadCreate(osThread(CANTask), NULL);

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
  * @brief CAN Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN1;
  hcan.Init.Prescaler = 9;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_1TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */

  /* USER CODE END CAN_Init 2 */

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 72-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 1000-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
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
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
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
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

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
  htim2.Init.Prescaler = 72-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1000-1;
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
  HAL_GPIO_WritePin(GPIOA, RuloBall1_Pin|RuloBall2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : RuloBall1_Pin RuloBall2_Pin */
  GPIO_InitStruct.Pin = RuloBall1_Pin|RuloBall2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : Sensor_Home_Pin */
  GPIO_InitStruct.Pin = Sensor_Home_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Sensor_Home_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : ECD3B_Pin */
  GPIO_InitStruct.Pin = ECD3B_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(ECD3B_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ECD3A_Pin ECD1A_Pin */
  GPIO_InitStruct.Pin = ECD3A_Pin|ECD1A_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : ECD1B_Pin */
  GPIO_InitStruct.Pin = ECD1B_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(ECD1B_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI3_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void SethomeHandle() {
	if (xQueueReceive(qHome, (void*) &IsSetHome, 1 / portTICK_PERIOD_MS) == pdTRUE) {
		brd_SetTargetRotaryAngle(0);
		brd_SetSpeedGun(0, MOTOR_GUN1);
		brd_SetSpeedGun(0, MOTOR_GUN2);
	}
}
void ShootHandle(){
	if (xQueueReceive(qHome, (void*) &IsFirePhoenix, 1 / portTICK_PERIOD_MS) == pdTRUE){
		PID_Rotary_CalPos(brd_GetTargetRotaryAngle());
		PID_Gun_CalSpeed(brd_GetSpeedGun(MOTOR_GUN1), MOTOR_GUN1);
		PID_Gun_CalSpeed(brd_GetSpeedGun(MOTOR_GUN2), MOTOR_GUN2);
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
	SET_HOME_DEFAULT_TASK:
	sethome_Begin();
	while (!sethome_IsComplete()) {
		sethome_Procedure();
		float speed = sethome_GetSpeed();
		xQueueSend(qPID, (const void* )&speed, 10/portTICK_PERIOD_MS);
		osDelay(1);
	}
	brd_SetHomeCompleteCallback();
	IsSetHome = 0;
  /* Infinite loop */
  for(;;)
  {
	  SethomeHandle();
	  if(IsSetHome){
		  osDelay(1);
		  goto SET_HOME_DEFAULT_TASK;
	  }
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartPIDTask */
/**
* @brief Function implementing the PIDTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartPIDTask */
void StartPIDTask(void const * argument)
{
  /* USER CODE BEGIN StartPIDTask */
  SET_HOME_PID_TASK:
  float TargetValue = 0;
  while (!sethome_IsComplete()) {
	  xQueueReceive(qPID, &TargetValue, 0);
	  PID_Rotary_CalSpeed((float) TargetValue);
	  osDelay(5);
  }
  /* Infinite loop */
  for(;;)
  {
	  if(IsSetHome) {
		  goto SET_HOME_PID_TASK;
	  }
	  ShootHandle();
	  osDelay(5);
  }
  /* USER CODE END StartPIDTask */
}

/* USER CODE BEGIN Header_StartCANTask */
/**
* @brief Function implementing the CANTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartCANTask */
void StartCANTask(void const * argument)
{
  /* USER CODE BEGIN StartCANTask */
  CAN_Init();
  uint32_t modeID;
  /* Infinite loop */
  for(;;)
  {
	  if (xTaskNotifyWait(pdFALSE, pdFALSE, &modeID, portMAX_DELAY)) {
	  			CAN_RxHeaderTypeDef rxHeader = canctrl_GetRxHeader();
	  			if (((rxHeader.StdId >> CAN_DEVICE_POS) == *(__IO uint32_t*) FLASH_ADDR_TARGET)) {
	  				if (rxHeader.RTR == CAN_RTR_REMOTE)
	  					handle_CAN_RTR_Response(&hcan, modeID);
	  				if (rxHeader.RTR == CAN_RTR_DATA)
	  					handleFunctionCAN((CAN_MODE_ID) modeID);
	  			}
	  			HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_RX_FIFO1_MSG_PENDING);
	  		}
//	  osDelay(1);
  }
  /* USER CODE END StartCANTask */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM4 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM4) {
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
