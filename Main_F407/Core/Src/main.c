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
#include "ActuatorValve.h"
#include "Odometry.h"
#include "PositionControl.h"
#include "ProcessControl.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum MainEvent
{
	MEVT_GET_NODE_SPEED_ANGLE,
}MainEvent;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim10;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart1_rx;

osThreadId defaultTaskHandle;
osThreadId TaskInvKineHandle;
uint32_t TaskInvKineBuffer[ 256 ];
osStaticThreadDef_t TaskInvKineControlBlock;
osThreadId TaskCANHandle;
uint32_t TaskCANBuffer[ 128 ];
osStaticThreadDef_t TaskCANControlBlock;
osThreadId TaskActuatorHandle;
osThreadId TaskOdometerHandle;
/* USER CODE BEGIN PV */

CAN_DEVICE_ID targetID = CANCTRL_DEVICE_MOTOR_CONTROLLER_1;
PID_Param pid;
PID_type type = PID_BLDC_SPEED;

CAN_MODE_ID Mode_ID = CANCTRL_MODE_MOTOR_SPEED_ANGLE;
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

float Xleft, Yleft;
float Xright;

_GamePad GamePad;
uint32_t gamepadRxIsBusy = 0;
float u, v, r;

float DeltaYR, DeltaYL, DeltaX;
//float TestTargetX = 0, TestTargetY = 0,  = 0;



uint32_t flagMain = 0;
#define MAIN_FLAG_GROUP flagMain
void flagmain_SetFlag(MainEvent e) {
	SETFLAG(MAIN_FLAG_GROUP, e);
}
bool flagmain_CheckFlag(MainEvent e) {
	return CHECKFLAG(MAIN_FLAG_GROUP, e);
}
void flagmain_ClearFlag(MainEvent e) {
	CLEARFLAG(MAIN_FLAG_GROUP, e);
}


uint32_t nodeSwerveSetHomeComplete = 0;
#define SETHOME_FLAG_GROUP nodeSwerveSetHomeComplete
void nodeHome_SetFlag(CAN_DEVICE_ID e) {
	SETFLAG(SETHOME_FLAG_GROUP, e);
}
bool nodeHome_CheckFlag(CAN_DEVICE_ID e) {
	return CHECKFLAG(SETHOME_FLAG_GROUP, e);
}
void nodeHome_ClearFlag(CAN_DEVICE_ID e) {
	CLEARFLAG(SETHOME_FLAG_GROUP, e);
}


uint8_t encEnb,encDis;
CAN_SpeedBLDC_AngleDC nodeSpeedAngle[3] = { 0 };
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_CAN1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM10_Init(void);
static void MX_USART1_UART_Init(void);
void StartDefaultTask(void const * argument);
void InverseKinematic(void const * argument);
void CAN_Bus(void const * argument);
void Actuator(void const * argument);
void OdometerHandle(void const * argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/*=============================== CAN ===============================*/
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
	HAL_CAN_DeactivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
	CAN_MODE_ID modeID = canctrl_Receive_2(hcan, CAN_RX_FIFO0);
	BaseType_t HigherPriorityTaskWoken = pdFALSE;
	xTaskNotifyFromISR(TaskCANHandle, modeID, eSetValueWithOverwrite, &HigherPriorityTaskWoken);
	portYIELD_FROM_ISR(HigherPriorityTaskWoken);
}
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan) {
	HAL_CAN_DeactivateNotification(hcan, CAN_IT_RX_FIFO1_MSG_PENDING);
	CAN_MODE_ID modeID = canctrl_Receive_2(hcan, CAN_RX_FIFO1);
	BaseType_t HigherPriorityTaskWoken = pdFALSE;
	xTaskNotifyFromISR(TaskCANHandle, modeID, eSetValueWithOverwrite, &HigherPriorityTaskWoken);
	portYIELD_FROM_ISR(HigherPriorityTaskWoken);
}
void CAN_Init() {
	HAL_CAN_Start(&hcan1);
	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_RX_FIFO1_MSG_PENDING);
	canctrl_Filter_Mask16(&hcan1,
			CANCTRL_MODE_SET_HOME << 5,
			CANCTRL_MODE_NODE_REQ_SPEED_ANGLE << 5,
			CANCTRL_MODE_SET_HOME << 5,
			CANCTRL_MODE_NODE_REQ_SPEED_ANGLE << 5,
			0,
			CAN_RX_FIFO0);
}

void setHomeComplete()
{

}

void handleFunctionCAN(CAN_MODE_ID mode, CAN_DEVICE_ID targetID) {
	switch (mode) {
		case CANCTRL_MODE_SET_HOME:
			nodeHome_SetFlag(targetID);
			// @formatter:off
//			if(nodeHome_CheckFlag(
//					  1 << CANCTRL_DEVICE_MOTOR_CONTROLLER_1
//					| 1 << CANCTRL_DEVICE_MOTOR_CONTROLLER_2
//					| 1 << CANCTRL_DEVICE_MOTOR_CONTROLLER_3
//					| 1 << CANCTRL_DEVICE_MOTOR_CONTROLLER_4
//					))
			if(nodeSwerveSetHomeComplete == 30)
				setHomeComplete();
																		// @formatter:on
		break;
		case CANCTRL_MODE_NODE_REQ_SPEED_ANGLE:
			nodeSpeedAngle[targetID - 1] = canfunc_MotorGetSpeedAndAngle();
//			flagmain_ClearFlag(MEVT_GET_NODE_SPEED_ANGLE);
		break;
		default:
			break;
	}

}
/*=============================== UART ===============================*/
uint8_t YawHandle;
uint8_t AngleData[5];
int CurrAngle;

char ds[12];
uint8_t uart2_ds[5], ds_ind, ds_cnt, ds_flg;

void Receive(uint8_t *DataArray){
      uint8_t *pInt = NULL;
      if(DataArray[4] == 13){
           pInt = &CurrAngle;
           for(uint8_t i = 0; i < 4; i++) {
               *(pInt + i) = DataArray[i];
            }
      }
      	  memset(DataArray,0,5);
      	YawHandle = 1;
 }

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	if(huart -> Instance == USART1)
	{
		HAL_UARTEx_ReceiveToIdle_DMA(&huart1, uart2_ds, 5);
			Receive(uart2_ds);
		  __HAL_DMA_DISABLE_IT(&hdma_usart1_rx,DMA_IT_HT);
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart->Instance == USART3) {
		gamepadRxIsBusy = 1;
		int ViTriData = -1;
		for (int i = 0; i <= 8; ++i) {
			if (UARTRX3_Buffer[i] == 0xAA) {
				ViTriData = i;
			}
		}
		if (ViTriData != -1) {
			int cnt = 0;
			while (cnt < 9) {
				DataTayGame[cnt] = UARTRX3_Buffer[ViTriData];
				++ViTriData;
				if (ViTriData >= 9) {
					ViTriData = 0;
				}
				++cnt;
			}

			GamepPadHandle(&GamePad, DataTayGame);

		} else {
			GamePad.Status = 0;
		}
		if (!gamepadRxIsBusy)
			HAL_UART_Receive_IT(&huart3, (uint8_t*) UARTRX3_Buffer, 9);

	}
}

void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan) {
	while (1);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
	__HAL_UART_CLEAR_OREFLAG(huart);
	memset(UARTRX3_Buffer, 0, sizeof(UARTRX3_Buffer));
	HAL_UART_Receive_IT(&huart3, (uint8_t*) UARTRX3_Buffer, 9);
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
  MX_DMA_Init();
  MX_CAN1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_USART3_UART_Init();
  MX_TIM1_Init();
  MX_TIM10_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

	HAL_UART_Receive_IT(&huart3, (uint8_t*) UARTRX3_Buffer, 9);
	HAL_UARTEx_ReceiveToIdle_DMA(&huart1, uart2_ds, 5);
	__HAL_DMA_DISABLE_IT(&hdma_usart1_rx,DMA_IT_HT);
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
  osThreadDef(defaultTask, StartDefaultTask, osPriorityIdle, 0, 256);
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

  /* definition and creation of TaskOdometer */
  osThreadDef(TaskOdometer, OdometerHandle, osPriorityLow, 0, 128);
  TaskOdometerHandle = osThreadCreate(osThread(TaskOdometer), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {

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
  RCC_OscInitStruct.PLL.PLLN = 160;
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
  hcan1.Init.Prescaler = 10;
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
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
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
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
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
  * @brief TIM10 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM10_Init(void)
{

  /* USER CODE BEGIN TIM10_Init 0 */

  /* USER CODE END TIM10_Init 0 */

  /* USER CODE BEGIN TIM10_Init 1 */

  /* USER CODE END TIM10_Init 1 */
  htim10.Instance = TIM10;
  htim10.Init.Prescaler = 160-1;
  htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim10.Init.Period = 65000;
  htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim10.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim10) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM10_Init 2 */

  /* USER CODE END TIM10_Init 2 */

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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, HC595_CLK_Pin|HC595_RCLK_Pin|HC595_OE_Pin|HC595_DATA_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : HC595_CLK_Pin HC595_RCLK_Pin HC595_OE_Pin HC595_DATA_Pin */
  GPIO_InitStruct.Pin = HC595_CLK_Pin|HC595_RCLK_Pin|HC595_OE_Pin|HC595_DATA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : SSBall_Pin SSLua1_Pin SSLua2_Pin */
  GPIO_InitStruct.Pin = SSBall_Pin|SSLua1_Pin|SSLua2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
uint8_t shootFlag;
void InvCpltCallback(ModuleID ID, float speed, float angle) {
	CAN_SpeedBLDC_AngleDC speedAngle;
	speedAngle.bldcSpeed = speed;
	speedAngle.dcAngle = angle;
	canfunc_MotorPutSpeedAndAngle(speedAngle);
	while (canctrl_Send(&hcan1, ID) != HAL_OK);
}

uint8_t enableTestMode;
CAN_SpeedBLDC_AngleDC Gun_Actuator;
void setTestModeActuator(){
	if(!enableTestMode) return;
	Gun_Actuator.bldcSpeed = 17.7;
	Gun_Actuator.dcAngle = 44.4;
	canfunc_MotorPutSpeedAndAngle(Gun_Actuator);
	canctrl_Send(&hcan1, CANCTRL_DEVICE_ACTUATOR_1);
	enableTestMode = 0;
}
void TestBreakProtection() {
	for (CAN_DEVICE_ID i = CANCTRL_DEVICE_MOTOR_CONTROLLER_1; i <= CANCTRL_DEVICE_MOTOR_CONTROLLER_3; i++) {
		canfunc_SetBoolValue(1, CANCTRL_MODE_PID_BLDC_BREAKPROTECTION);
		while (canctrl_Send(&hcan1, i) != HAL_OK);
		osDelay(1);
	}
	osDelay(1000);
	for (CAN_DEVICE_ID i = CANCTRL_DEVICE_MOTOR_CONTROLLER_1; i <= CANCTRL_DEVICE_MOTOR_CONTROLLER_3; i++) {
		canfunc_SetBoolValue(0, CANCTRL_MODE_PID_BLDC_BREAKPROTECTION);
		while (canctrl_Send(&hcan1, i) != HAL_OK);
		osDelay(1);
	}
}
void canShoot(){
	shootFlag = 1;
	canctrl_SetID(CANCTRL_MODE_SHOOT);
	canctrl_Send(&hcan1, CANCTRL_DEVICE_ACTUATOR_1);
	while (canctrl_Send(&hcan1, CANCTRL_DEVICE_ACTUATOR_1) != HAL_OK);
	shootFlag = 0;
	osDelay(4000);
//	TestBreakProtection();
}

void StopEnc(bool brake){
	for (CAN_DEVICE_ID i = CANCTRL_DEVICE_MOTOR_CONTROLLER_1; i <= CANCTRL_DEVICE_MOTOR_CONTROLLER_3; i++) {
			canfunc_SetBoolValue(brake,CANCTRL_MODE_MOTOR_BLDC_BRAKE);
			while (canctrl_Send(&hcan1, i) != HAL_OK);
			osDelay(1);
	}
}

bool stopFlag;
void softEmergencyStop() {
	CAN_SpeedBLDC_AngleDC speedAngle;
	speedAngle.bldcSpeed = 0;
	speedAngle.dcAngle = 0;
	for (CAN_DEVICE_ID i = CANCTRL_DEVICE_MOTOR_CONTROLLER_1; i <= CANCTRL_DEVICE_MOTOR_CONTROLLER_3; i++) {
		canfunc_MotorPutSpeedAndAngle(speedAngle);
		while (canctrl_Send(&hcan1, i) != HAL_OK);
	}
	stopFlag = 1;
}
uint8_t ReleaseAll;
void Release(){
	for (CAN_DEVICE_ID i = CANCTRL_DEVICE_MOTOR_CONTROLLER_1; i <= CANCTRL_DEVICE_MOTOR_CONTROLLER_3; i++) {
		canfunc_SetBoolValue(1, CANCTRL_MODE_PID_BLDC_BREAKPROTECTION);
		while (canctrl_Send(&hcan1, i) != HAL_OK);
		osDelay(1);
	}
}
// used for README example
/*
 typedef enum EnableFuncHandle{
 ENABLE_START,
 ENABLE_SETHOME,
 ENABLE_BREAK_PROTECTION,
 ENABLE_ANGLE_SPEED,
 ENABLE_CAN_TX,
 ENABLE_END,
 }EnableFuncHandle;
 #define TARGET_FLAG_GROUP enableFlag
 bool enableSendSpeedAndAngle = false;
 bool enableSendPID = false;
 bool enableSendTestMode = false;
 bool enableSendBrake = false;
 EnableFuncHandle mode = ENABLE_ANGLE_SPEED;
 uint8_t testMode = 1;
 uint8_t brake = 1;
 float bldcSpeed = 50;
 float dcAngle = 0;

 bool enableSendBreakProtection = false;
 uint32_t enableFlag = 0;

 void enfunc_SetFlag(CAN_MODE_ID e){SETFLAG(TARGET_FLAG_GROUP,e);}
 bool enfunc_CheckFlag(CAN_MODE_ID e){return CHECKFLAG(TARGET_FLAG_GROUP,e);}
 void enfunc_ClearFlag(CAN_MODE_ID e){CLEARFLAG(TARGET_FLAG_GROUP,e);}

 bool TestFlag;
 void FlagEnable(){
 if (TestFlag){
 enfunc_SetFlag(mode);
 TestFlag = false;
 canTxHandleFunc(Mode_ID, Device_ID);
 }
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
 */


int count;
int NopeCycle = 3300;

float preGyro;
void RTR_SpeedAngle(){
//	HAL_TIM_Base_Start(&htim10);
//	__HAL_TIM_SET_COUNTER(&htim10,0);
	for (uint8_t i = 0;i < 3;i++){
		canctrl_SetID(CANCTRL_MODE_NODE_REQ_SPEED_ANGLE);
		bool a = 1;
		canctrl_PutMessage((void*)&a, sizeof(bool));
		targetID = i + 1;
		while(canctrl_Send(&hcan1, targetID) != HAL_OK);
		for(uint16_t i = 0; i < NopeCycle; i++) __NOP();
	}
//	count = __HAL_TIM_GET_COUNTER(&htim10);
//	HAL_TIM_Base_Stop(&htim10);
}

#define PulsePerRev 200*2.56

//#define a 0.045
typedef struct SpeedReadSlave{
	float V;
	float Vx;
	float Vy;

	float VxC;
	float VyC;

	float Offset;

	float Vfilt;
	float VfiltPre;
	float filterAlpha;

	int preCount;
}SpeedReadSlave;

typedef struct ForwardKine{
	float uOut;
	float vOut;
	float thetaOut;
}ForwardKine;

typedef struct SwerveOdoHandle{
	float dX;
	float dY;
	float dTheta;

	float poseX;
	float poseY;
	float poseTheta;

	float S;
	float C;

	float OffsetGyro;
	float Suy;
}SwerveOdoHandle;

void omegaToZeta(ForwardKine *kine, float* VxA, float* VyA)
{
	kine->uOut 		= - 0.2357*VxA[0] + 0.2357*VyA[0] - 0.2357*VxA[1] - 0.2357*VyA[1] + 0.3333*VxA[2] + 0.0000*VyA[2] ;
	kine->vOut 		= - 0.2467*VxA[0] - 0.3262*VyA[0] + 0.2467*VxA[1] - 0.3262*VyA[1] - 0.0000*VxA[2] + 0.1898*VyA[2] ;
	kine->thetaOut 	=   0.1417*VxA[0] + 1.1707*VyA[0] - 0.1417*VxA[1] + 1.1707*VyA[1] + 0.0000*VxA[2] + 1.8560*VyA[2] ;
}

SwerveOdoHandle Odo;
ForwardKine	Fkine;
SpeedReadSlave Module[4];
float Vx[4],Vy[4];
float Gyro;

#define UABOVE_X 	0.3
#define UBELOW_X	-0.3

#define UABOVE_Y 	0.3
#define UBELOW_Y	-0.3

#define UABOVE_THETA 	5
#define UBELOW_THETA	-5

#define ROBOT_RADIUS	0.25;

typedef struct PDParam{
	float e;
	float pre;

	float kP;
	float kD;

	float uP;
	float uD;
	float uDf;
	float uDfpre;
	float Alpha;

	float u;
	float uAbove;
	float uBelow;
}PDParam;



void PD_setParam(PDParam *pd,float kP,float kD,float Alpha)
{
	pd->kP = kP;
	pd->kD = kD;
	pd->Alpha = Alpha;
}

void PD_setLimit(PDParam *pd,float uAbove,float uBelow)
{
	pd->uAbove = uAbove;
	pd->uBelow = uBelow;
}

PDParam pDX;
PDParam pDY;
PDParam pDTheta;

float TargetX;
float TargetY;
float TargetTheta;


uint8_t Break;
float absFloat(float num){
	if (num<0.0){
		return num*-1.0;
	}
	else
	return num;
}
float min(float a,float b){
	float min;
	min = a;
	if(b<=min){
		min = b;
	}
	return min;
}

float max(float a,float b){
	float max;
	max = a;
	if(b>=max){
		max = b;
	}
	return max;
}
int Isteady(float e,float thresthold)
{
	if (absFloat(e)<thresthold){
		return 1;
	}

	return 0;
}


uint8_t Run,resetParam,breakProtect;
float kpX = 1 ,kpY = 1,kdX,kdY,kpTheta=0.6,kdTheta,alphaCtrol = 0.2;
int testSpeed,testPos;
float uControlX,uControlY,uControlTheta;
uint8_t stateRun = 0 ,steadycheck;

uint8_t xaDay;

uint8_t ssCheck,stateChange,rst,Gamepad;
#define PFxState10 -0.93
#define PFyState10 -0.75
#define PFxState20 -0.93
#define PFyState20 0.05

char tx_buffer1[] = "rst\n";
char tx_buffer2[] = "red\n";

float yaw;
uint8_t red,rst;

uint8_t StopUseXY,StopUsetheta;
uint8_t StopUsePIDX,StopUsePIDY,StopUsePIDr;
void ReadIMU(){
	HAL_UART_Transmit(&huart1, (uint8_t *)tx_buffer2, strlen(tx_buffer2), 100);
	osDelay(6);
//	yaw = CurrAngle*M_PI/180;
}
void ResetIMU(){
	HAL_UART_Transmit(&huart1, (uint8_t *)tx_buffer1, strlen(tx_buffer1), 100);
	osDelay(6);
//	yaw = CurrAngle*M_PI/180;
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
 * @brief  Function implementing the defaultTask thread.
 * @param  argument: Not used
 * @retval None
 */
uint8_t useEuler;
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
	/* Infinite loop */

	swer_Init();

	for (;;) {

		if(rst == 1){
			ResetIMU();
			rst = 0;
		}

		if(red == 1){
			ReadIMU();
			red = 0;
		}
		if(xaDay == 0)
		{
			invkine_Implementation(MODULE_ID_3, uControlX, uControlY, uControlTheta, &InvCpltCallback);
			invkine_Implementation(MODULE_ID_1, uControlX, uControlY, uControlTheta, &InvCpltCallback);
			invkine_Implementation(MODULE_ID_2, uControlX, uControlY, uControlTheta, &InvCpltCallback);
		}else{
			 InvCpltCallback(MODULE_ID_3, 0, 0);
			 InvCpltCallback(MODULE_ID_1, 0, 0);
			 InvCpltCallback(MODULE_ID_2, 0, 0);
		}

		if(((GamePad.Square == 1)&&(GamePad.Circle == 1))||stateRun == 50)
		{
			xaDay = 1;
		}
		if (gamepadRxIsBusy) {
			gamepadRxIsBusy = 0;
			HAL_UART_Receive_IT(&huart3, (uint8_t*) UARTRX3_Buffer, 9);
		}
		if ((huart3.Instance->CR1 & USART_CR1_UE) == 0) {
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
	for (;;) {


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
	CAN_Init();
	osDelay(500);
	canctrl_RTR_TxRequest(&hcan1, CANCTRL_DEVICE_MOTOR_CONTROLLER_1, CANCTRL_MODE_SET_HOME);
	osDelay(1);
	canctrl_RTR_TxRequest(&hcan1, CANCTRL_DEVICE_MOTOR_CONTROLLER_2, CANCTRL_MODE_SET_HOME);
	osDelay(1);
	canctrl_RTR_TxRequest(&hcan1, CANCTRL_DEVICE_MOTOR_CONTROLLER_3, CANCTRL_MODE_SET_HOME);
	osDelay(1);
	osDelay(500);
//	canctrl_RTR_TxRequest(&hcan1, CANCTRL_DEVICE_MOTOR_CONTROLLER_4, CANCTRL_MODE_SET_HOME);
//	osDelay(1);
	uint32_t modeID;
	/* Infinite loop */
	for (;;) {
		if (xTaskNotifyWait(pdFALSE, pdFALSE, &modeID, portMAX_DELAY)) {
			CAN_RxHeaderTypeDef rxHeader = canctrl_GetRxHeader();
			uint32_t targetID = rxHeader.StdId >> CAN_DEVICE_POS;
			if ((modeID == CANCTRL_MODE_SET_HOME || modeID == CANCTRL_MODE_NODE_REQ_SPEED_ANGLE ) && targetID) {
				handleFunctionCAN(modeID, targetID);
			}
			HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
		}
	}
  /* USER CODE END CAN_Bus */
}

/* USER CODE BEGIN Header_Actuator */
/**
 * @brief Function implementing the TaskActuator thread.
 * @param argument: Not used
 * @retval None
 */
uint8_t BallSS,shoot;
/* USER CODE END Header_Actuator */
void Actuator(void const * argument)
{
  /* USER CODE BEGIN Actuator */
	/* Infinite loop */
	for (;;) {
		process_RunSSAndActuator(&TestBreakProtection);
//		if(ReleaseAll== 1){
//			Release();
//			ReleaseAll = 0;
//		}
//		if(shoot == 1){
//			canShoot();
//			shoot = 0;
//		}
//
//		if(encEnb == 1){
//			StopEnc(0);
//			encEnb = 0;
//		}
//		if(encDis == 1){
//			StopEnc(1);
//			encDis = 0;
//		}
//		BallSS = HAL_GPIO_ReadPin(SSBall_GPIO_Port, SSBall_Pin);
//
//		if((stateRun == 4)||(stateRun == 15)){
//			if (HAL_GPIO_ReadPin(SSLua1_GPIO_Port, SSLua1_Pin)&&HAL_GPIO_ReadPin(SSLua2_GPIO_Port, SSLua2_Pin)){
//				ssCheck ++;
//			}else {
//				ssCheck = 0;
//			}
//
//
//			if(ssCheck > 5){
//				stateRun ++ ;
//				ssCheck = 0;
//			}
//		}
//
//	//MMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMM//
//	//--------------------------------------State 7 ---------------------------------------------------//
//			if(stateRun == 7){
////				ResetIMU();
//				valve_BothCatch();
//				Odo.poseTheta = 0;
//				Odo.poseY = 0;
//				Odo.poseX = 0;
//				stateChange = 0;
//				TestBreakProtection();
//				osDelay(50);
//				stateRun += 1;
//			}
//	//MMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMM//
//	//--------------------------------------State 17 ---------------------------------------------------//
//			if(stateRun == 17){
////				ResetIMU();
//				valve_BothCatch();
//				Odo.poseTheta = 0;
//				Odo.poseY = 0;
//				Odo.poseX = 0;
//				stateChange = 0;
//				TestBreakProtection();
//				osDelay(50);
//				stateRun += 1;
//			}
//
//			if(stateRun == 28){
//				Odo.poseTheta = yaw;
//				if(HAL_GPIO_ReadPin(SSBall_GPIO_Port, SSBall_Pin)){
//					ssCheck++;
//				}else{
//					ssCheck = 0;
//				}
//					if(ssCheck>15){
//						stateRun++;
//						ssCheck = 0;
//				}
//			}
//			if(stateRun == 35){
//				Odo.poseTheta = yaw;
//				if(HAL_GPIO_ReadPin(SSBall_GPIO_Port, SSBall_Pin)){
//					ssCheck++;
//				}else{
//					ssCheck = 0;
//				}
//					if(ssCheck>15){
//						stateRun++;
//						ssCheck = 0;
//				}
//			}
//	//MMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMM//
//	//--------------------------------------State 12 ---------------------------------------------------//
//			if(stateRun == 12){
//
//				StopUseXY = 0;
//				stateRun += 1;
//			}
//	//MMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMM//
//	//--------------------------------------State 22 ---------------------------------------------------//
//			if(stateRun == 22){
//				StopUseXY = 0;
//				stateRun += 1;
//			}
		osDelay(1);
	}
  /* USER CODE END Actuator */
}

/* USER CODE BEGIN Header_OdometerHandle */
/**
 * @brief Function implementing the TaskOdometer thread.
 * @param argument: Not used
 * @retval None
 */
float testX,testY,testTheta;
void odo_SpeedAngleUpdate(){
	for (int i = 0;i<3;i++){
		odo_SetObj_SpAg(i,nodeSpeedAngle[i]);
	}
}
/* USER CODE END Header_OdometerHandle */
void OdometerHandle(void const * argument)
{
  /* USER CODE BEGIN OdometerHandle */
	  /* USER CODE BEGIN OdometerHandle */

		valve_Init();
		osDelay(1000);
		process_Init();
		/* Infinite loop */
		for (;;) {
			if(!shootFlag){
				RTR_SpeedAngle();
			}
			odo_SpeedAngleUpdate();

			process_ReadIMU();
			process_SetYaw(CurrAngle);

			process_Run(Run);


//			for (int i = 0;i<3;i++)
//			{
//				SpeedRead2(&Module[i], nodeSpeedAngle[i].bldcSpeed);
//				Module[i].Vx = Module[i].V * cos(nodeSpeedAngle[i].dcAngle*M_PI/180);
//				Module[i].Vy = Module[i].V * sin(nodeSpeedAngle[i].dcAngle*M_PI/180);
//				Vx[i] = Module[i].Vx;
//				Vy[i] = Module[i].Vy;
//			}
//
//			omegaToZeta(&Fkine, Vx, Vy);
//
//
//			ReadIMU();
//			Odo.poseTheta += Fkine.thetaOut*DeltaT;
//	//		Odo.poseTheta = yaw;
//			Odo.poseX += (Fkine.uOut*cos(Odo.poseTheta)-Fkine.vOut*sin(Odo.poseTheta))*DeltaT;
//			Odo.poseY += (Fkine.uOut*sin(Odo.poseTheta)+Fkine.vOut*cos(Odo.poseTheta))*DeltaT;
//
//			if (breakProtect == 1)
//			{
//				TestBreakProtection();
//				breakProtect = 0;
//			}
//			if(resetParam == 1){
//				Odo.poseX = 0;
//				Odo.poseY = 0;
//				Odo.poseTheta = 0;
//				resetParam = 0;
//			}
//
//			Odo.dTheta = Odo.poseTheta*180/M_PI;
//
//
//	//MMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMM//
//	//------------------------------------State constrain----------------------------------------------//
//	if((stateRun<4)||((stateRun>8)&&(stateRun<15))||(stateRun>18)){
//		if((stateRun != 12)&&(stateRun != 22)&&(stateRun !=7)&&(stateRun !=17)&&(stateRun != 23)){
//			if(StopUseXY == 0)
//			{
//				if(!StopUsePIDX){
//					PD_Controller(&pDX, Xtrajec.xTrajec, Odo.poseX);
//				}
//
//				if(!StopUsePIDY){
//					PD_Controller(&pDY, Ytrajec.xTrajec, Odo.poseY);
//				}
//			}else{
//				pDX.u = 0;
//				pDY.u = 0;
//			}
//
//			if(StopUsetheta == 0){
//				if(!StopUsePIDr){
//					if(!useEuler){
//						PD_Controller(&pDTheta, ThetaTrajec.xTrajec, yaw);
//					}else{
//						PD_Controller(&pDTheta, ThetaTrajec.xTrajec, Odo.poseTheta);
//					}
//				}
//			}else{
//				pDTheta.u = 0;
//			}
//		}
//	}
//
//
//	//MMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMM//
//	//--------------------------------------State 1 ---------------------------------------------------//
//			if((stateRun == 1))
//			{
//
//				if ((absFloat(Odo.poseX-Pfx)<0.02)&&(absFloat(Odo.poseY-Pfy)<0.02)){
//					stateRun += 1;
//				}
//
//			}
//	//MMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMM//
//	//--------------------------------------State 2 ---------------------------------------------------//
//			if(stateRun == 2)
//			{
//				tfx = 1.5;
//				tfy = 1.5;
//				tftheta = 2;
//
//				Xtrajec.t = 0;
//				Ytrajec.t = 0;
//				ThetaTrajec.t = 0;
//
//				v0x = Fkine.uOut;
//				v0y = Fkine.vOut;
//				v0theta = Fkine.thetaOut;
//
//				Pfx = -0.24;
//				P0x = Odo.poseX;
//				Pfy = -0.94;
//				P0y = Odo.poseY;
//				Pftheta = 0*M_PI/180;
//				P0theta = 0;
//
//				vfx = 0.01;
//				vfy = -0.12;
//				vftheta = 0;
//
//				stateRun += 1;
//			}
//	//MMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMM//
//	//--------------------------------------State 3 ---------------------------------------------------//
//			if (stateRun == 3){
//				if ((absFloat(Odo.poseX-Pfx)<0.03)&&(absFloat(Odo.poseY-Pfy)<0.03)){
//					v0x = 0;
//					v0y = 0;
//					v0theta = 0;
//
//					vfx = 0;
//					vfy = 0;
//					vftheta = 0;
//
//					stateRun += 1;
//				}
//			}
//	//MMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMM//
//	//--------------------------------------State 4 ---------------------------------------------------//
//			if (stateRun == 4){
//				u = 0.05;
//				v = -0.12;
//				r = 0;
//				Odo.poseTheta = 0;
//			}
//	//MMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMM//
//	//--------------------------------------State 5 ---------------------------------------------------//
//			if (stateRun == 5){
//				u = 0.05;
//				v = -0.08;
//				r = 0.05;
//				Odo.poseTheta = 0;
//				stateChange++;
//				if(stateChange>3)
//				{
//					u = 0.05;
//					v = 0;
//					r = 0;
//					stateChange = 0;
//					stateRun++;
//				}
//			}
//	//MMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMM//
//	//--------------------------------------State 6 ---------------------------------------------------//
//			if(stateRun == 6){
//				u = 0;
//				v = 0;
//				r = 0;
//				stateRun += 1 ;
//			}
//
//	//MMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMM//
//	//--------------------------------------State 8 ---------------------------------------------------//
//			if(stateRun == 8){
//				stateChange++;
//				StopUsePIDX = 1;
//				StopUsePIDY = 1;
//				StopUsePIDr = 1;
//				u = 0.1;
//				v = 0;
//				r = 0;
//				if(stateChange> 5){
//					ResetIMU();
//					StopUsePIDX = 0;
//					StopUsePIDY = 0;
//					StopUsePIDr = 0;
//					stateChange = 0;
//					tfx = 1.4;
//					tfy = 1;
//					tftheta = 1;
//
//					Xtrajec.t = 0;
//					Ytrajec.t = 0;
//					ThetaTrajec.t = 0;
//
//					Pfx = -0.7;
//					P0x = 0;
//					Pfy = 0;
//					P0y = 0;
//					Pftheta = 0;
//					P0theta = 0;
//
//					Odo.poseTheta = 0;
//					Odo.poseY = 0;
//					Odo.poseX = 0;
//
//					stateRun += 1;
//				}
//			}
//	//MMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMM//
//	//--------------------------------------State 9 ---------------------------------------------------//
//			if (stateRun == 9){
//				if ((absFloat(Odo.poseX-Pfx)<0.03)&&(absFloat(Odo.poseY-Pfy)<0.03)){
//
//				stateRun += 1;
//				}
//			}
//	//MMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMM//
//	//--------------------------------------State 10 ---------------------------------------------------//
//			if(stateRun == 10){
//				tfx = 3;
//				tfy = 3;
//				tftheta = 3;
//
//				Xtrajec.t = 0;
//				Ytrajec.t = 0;
//				ThetaTrajec.t = 0;
//
//				v0x = Fkine.uOut;
//				v0y = Fkine.vOut;
//
//				Pfx = PFxState10;
//				P0x = Odo.poseX;
//				Pfy = PFyState10;
//				P0y = Odo.poseY;
//				Pftheta = -89*M_PI/180;
//				P0theta = 0;
//				stateRun += 1;
//			}
//	//MMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMM//
//	//--------------------------------------State 11 ---------------------------------------------------//
//			if (stateRun == 11){
//				if(stateChange == 0){
//					if ((absFloat(Odo.poseX-Pfx)<0.2)&&(absFloat(Odo.poseY-Pfy)<0.2)){
//						stateChange++;
//						}
//				}
//				if(stateChange == 1){
//					if ((absFloat(Odo.poseX-Pfx)<0.03)&&(absFloat(Odo.poseY-Pfy)<0.03)){
//						Odo.poseTheta = yaw;
//						stateChange = 3;
//						}
//				}
////				if(stateChange == 2)
////				{
////					if ((absFloat(Odo.poseX-Pfx)<0.03)&&(absFloat(Odo.poseY-Pfy)<0.03)){
////						steadycheck ++ ;
////						StopUsetheta = 1;
////						StopUseXY = 1;
////					}
////					else{
////						steadycheck = 0;
////						StopUseXY = 0;
////					}
////
////					if(steadycheck>5){
////
////						useEuler = 1;
////						stateChange ++;
////						steadycheck = 0;
////						StopUsetheta = 0;
////					}
////				}
//				if(stateChange == 3)
//				{
//					StopUseXY = 1;
//					PD_setParam(&pDTheta,1.8,0, alphaCtrol);
//					if ((absFloat(yaw-Pftheta)<=2*M_PI/180)){
//						steadycheck ++ ;
//					}else{
//						StopUsetheta = 0;
//						steadycheck = 0;
//					}
//
//				}
//
//				if(steadycheck > 15)
//				{
//					useEuler = 0;
//					PD_setParam(&pDTheta, 1.2, 0, alphaCtrol);
//					valve_BothHold();
//					osDelay(300);
//					valve_BothRelease();
//					Odo.poseTheta = yaw;
//					stateChange = 0;
//					u = 0;
//					v = 0;
//					r = 0;
//					v0x = 0;
//					v0y = 0;
//					stateRun += 1;
//					steadycheck = 0;
//					StopUseXY = 0;
//					StopUsetheta = 0;
//				}
//			}
//
//
//	//MMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMM//
//	//--------------------------------------State 13 ---------------------------------------------------//
//			if(stateRun == 13){
//				vfx = 0;
//				vfy = 0;
//				vftheta = 0;
//
//				tfx = 3;
//				tfy = 3;
//				tftheta = 3;
//				Xtrajec.t = 0;
//				Ytrajec.t = 0;
//				ThetaTrajec.t = 0;
//				v0x = Fkine.uOut;
//				v0y = Fkine.vOut;
//				Pfx = -0.03;
//				P0x = Odo.poseX;
//				Pfy = 0.03;
//				P0y = Odo.poseY;
//				Pftheta = 0;
//				P0theta = -90*M_PI/180;
//	//			valve_BothRelease();
//				stateRun += 1;
//			}
//	//MMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMM//
//	//--------------------------------------State 14 ---------------------------------------------------//
//			if (stateRun == 14){
//				if ((absFloat(Odo.poseX-Pfx)<0.03)&&(absFloat(Odo.poseY-Pfy)<0.03)){
//					stateRun ++;
//					Odo.poseY = 0;
//					Odo.poseX = 0;
//				}
//			}
//	//MMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMM//
//	//--------------------------------------State 15 ---------------------------------------------------//
//			if (stateRun == 15){
//				u = 0.05;
//				v = -0.12;
//				r = 0;
//			}
//	//MMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMM//
//	//--------------------------------------State 16 ---------------------------------------------------//
//			if (stateRun == 16){
//				u = 0.05;
//				v = -0.08;
//				r = 0.05;
//				stateChange++;
//				if(stateChange>3)
//				{
//					u = 0;
//					v = 0;
//					r = 0;
//					stateChange = 0;
//					stateRun++;
//				}
//			}
//	//MMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMM//
//	//--------------------------------------State 18 ---------------------------------------------------//
//			if(stateRun == 18){
//				stateChange++;
//				StopUsePIDX = 1;
//				StopUsePIDY = 1;
//				StopUsePIDr = 1;
//				u = 0.1;
//				v = 0;
//				r = 0;
//				if(stateChange > 5){
//					StopUsePIDX = 0;
//					StopUsePIDY = 0;
//					StopUsePIDr = 0;
//					ResetIMU();
//					stateChange = 0;
//					tfx = 2;
//					tfy = 1;
//					tftheta = 1;
//
//					Xtrajec.t = 0;
//					Ytrajec.t = 0;
//					ThetaTrajec.t = 0;
//
//					Pfx = -0.5;
//					P0x = Odo.poseX;
//					Pfy = 0;
//					P0y = 0;
//					Pftheta = 0;
//					P0theta = 0;
//
//					Odo.poseY = 0;
//					Odo.poseX = 0;
//
//					stateRun += 1;
//				}
//			}
//	//MMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMM//
//	//--------------------------------------State 19 ---------------------------------------------------//
//			if (stateRun == 19){
//				if ((absFloat(Odo.poseX-Pfx)<0.4)&&(absFloat(Odo.poseY-Pfy)<0.4)){
//				stateRun += 1;
//				}
//			}
//	//MMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMM//
//	//--------------------------------------State 20 ---------------------------------------------------//
//			if(stateRun == 20){
//				vfx = 0;
//				vfy = 0;
//				vftheta = 0;
//
//				tfx = 3;
//				tfy = 3;
//				tftheta = 3;
//				Xtrajec.t = 0;
//				Ytrajec.t = 0;
//				ThetaTrajec.t = 0;
//				v0x = Fkine.uOut;
//				v0y = Fkine.vOut;
//				v0theta = Fkine.thetaOut;
//				Pfx = PFxState20;
//				P0x = Odo.poseX;
//				Pfy = PFyState20;
//				P0y = Odo.poseY;
//				Pftheta = -89*M_PI/180;
//				P0theta = 0;
//				stateRun += 1;
//			}
//	//MMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMM//
//	//--------------------------------------State 21 ---------------------------------------------------//
//			if (stateRun == 21){
//					if(stateChange == 0){
//						if ((absFloat(Odo.poseX-Pfx)<0.2)&&(absFloat(Odo.poseY-Pfy)<0.2)){
////							valve_BothHold();
//							stateChange++;
//							}
//					}
//					if(stateChange == 1){
//						if ((absFloat(Odo.poseX-Pfx)<0.03)&&(absFloat(Odo.poseY-Pfy)<0.03)){
//							stateChange = 3;
////							ReadIMU();
//							Odo.poseTheta = yaw;
////							useEuler = 1;
//							}
//					}
//					if(stateChange == 2)
//					{
//						if ((absFloat(Odo.poseX-Pfx)<0.03)&&(absFloat(Odo.poseY-Pfy)<0.03)){
//							steadycheck ++ ;
//							StopUsetheta = 1;
//							StopUseXY = 1;
//						}
//						else{
//							steadycheck = 0;
//							StopUseXY = 0;
//						}
//
//						if(steadycheck>5){
//							Odo.poseTheta = yaw;
////							useEuler = 1;
//							stateChange ++;
//							steadycheck = 0;
//							StopUsetheta = 0;
//						}
//					}
//					if(stateChange == 3)
//					{
//						StopUseXY = 1;
//						PD_setParam(&pDTheta, 1.8, 0, alphaCtrol);
//						if ((absFloat(yaw-Pftheta)<=2*M_PI/180)){
//							steadycheck ++ ;
//						}else{
//							StopUsetheta = 0;
//							steadycheck = 0;
//						}
//
//					}
//
//					if(steadycheck > 15)
//					{
//						useEuler = 0;
//						PD_setParam(&pDTheta, 1.2, 0, alphaCtrol);
//						valve_BothHold();
//						osDelay(300);
//						valve_BothRelease();
//						Odo.poseTheta = yaw;
//						stateChange = 0;
//						u = 0;
//						v = 0;
//						r = 0;
//						v0x = 0;
//						v0y = 0;
//						stateRun += 1;
//						steadycheck = 0;
//						StopUseXY = 0;
//						StopUsetheta = 0;
//					}
//				}
//
//	////MMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMM//
//	////--------------------------------------State 23 ---------------------------------------------------//
//			if (stateRun == 23){
//
//				if(stateChange == 0){
//				valve_BothRelease();
//	//			TestBreakProtection();
//				}
//				stateChange++;
//				if(stateChange >1){
//					stateChange = 0;
//					Odo.poseTheta = yaw;
//					vfx = 0;
//					vfy = 0;
//					vftheta = 0;
//
//					tfx = 2;
//					tfy = 2;
//					tftheta = 2;
//					Xtrajec.t = 0;
//					Ytrajec.t = 0;
//					ThetaTrajec.t = 0;
//
//					Pfx = -0.22;
//					P0x = Odo.poseX;
//					Pfy = 0.1;
//					P0y = Odo.poseY;
//					v0x = Fkine.uOut;
//					v0y = Fkine.vOut;
//					Pftheta = Odo.poseTheta;
//					P0theta = Odo.poseTheta;
//					stateRun += 1;
//				}
//
//			}
//
//			if (stateRun == 24){
//				Odo.poseTheta = yaw;
//				if ((absFloat(Odo.poseX-Pfx)<0.03)&&(absFloat(Odo.poseY-Pfy)<0.03)){
//				stateRun += 1;
//				}
//			}
//			if (stateRun == 25){
//				vfx = 0;
//				vfy = 0;
//				vftheta = 0;
//
//				tfx = 4;
//				tfy = 4;
//				tftheta = 4;
//
//				Xtrajec.t = 0;
//				Ytrajec.t = 0;
//				ThetaTrajec.t = 0;
//
//				Pfx = Odo.poseX;
//				P0x = Odo.poseX;
//				Pfy = -2.14;
//				P0y = Odo.poseY;
//				v0x = Fkine.uOut;
//				v0y = Fkine.vOut;
//				Pftheta = 0;
//				P0theta = yaw;
//				Odo.poseTheta = yaw;
//				stateRun += 1;
//			}
//			if (stateRun == 26){
//				Odo.poseTheta = yaw;
//				if ((absFloat(Odo.poseX-Pfx)<0.03)&&(absFloat(Odo.poseY-Pfy)<0.03)){
//				stateRun += 1;
//				}
//			}
//
//			if(stateRun == 27){
//				u = -0.3;
//				StopUsePIDX = 1;
//				stateRun ++;
//			}
//			if(stateRun == 29){
//				u = -0.2;
//				stateChange ++;
//				if(stateChange > 22){
//					vfx = 0;
//					vfy = 0;
//					vftheta = 0;
//
//					tfx = 4;
//					tfy = 1;
//					tftheta = 1;
//
//					Xtrajec.t = 0;
//					Ytrajec.t = 0;
//					ThetaTrajec.t = 0;
//
//					Pfx = Odo.poseX;
//					P0x = Odo.poseX;
//					Pfy = Odo.poseY;
//					P0y = Odo.poseY;
//					v0x = Fkine.uOut;
//					v0y = Fkine.vOut;
//					Pftheta = 0;
//					P0theta = yaw;
//					Odo.poseTheta = yaw;
//
//					StopUsePIDX = 0;
//					StopUsePIDY = 1;
//					StopUsePIDr = 0;
//					PD_setParam(&pDTheta, 2, 0, 0);
//					stateChange = 0;
//					stateRun ++;
//				}
//			}
//			if(stateRun == 30){
//				v = 0.2;
//				stateChange++;
//				if(stateChange> 24){
//					u = 0;
//					v = 0;
//					r = 0;
//					StopUseXY = 1;
//					StopUsetheta = 1;
//					stateChange = 0;
//					stateRun++;
//				}
//			}
//			if(stateRun == 31){
//				stateChange++;
//				if(stateChange>1){
//					StopEnc(1);
//					stateChange = 0;
//					stateRun ++;
//					canShoot();
//					StopEnc(0);
//				}
//			}
//			if(stateRun == 32){
//				StopUseXY = 0;
//				StopUsetheta = 0;
//				StopUsePIDX = 0;
//				StopUsePIDY = 0;
//				stateChange = 0;
//
//				vfx = 0;
//				vfy = 0;
//				vftheta = 0;
//
//				tfx = 1;
//				tfy = 1.2;
//				tftheta = 1;
//
//				Xtrajec.t = 0;
//				Ytrajec.t = 0;
//				ThetaTrajec.t = 0;
//
//				Pfx = Odo.poseX;
//				P0x = Odo.poseX;
//				Pfy = Pfy-0.04;
//				P0y = Odo.poseY;
//				v0x = Fkine.uOut;
//				v0y = Fkine.vOut;
//				Pftheta = 0;
//				P0theta = yaw;
//				Odo.poseTheta = yaw;
//				stateRun += 1;
//			}
//			if (stateRun == 33){
//				Odo.poseTheta = yaw;
//				if ((absFloat(Odo.poseX-Pfx)<0.03)&&(absFloat(Odo.poseY-Pfy)<0.03)){
//				stateRun += 1;
//				}
//			}
//			if(stateRun == 34){
//				u = -0.3;
//				StopUsePIDX = 1;
//				stateRun ++;
//			}
//			if(stateRun == 36){
//				u = -0.2;
//				stateChange ++;
//				if(stateChange > 20){
//					vfx = 0;
//					vfy = 0;
//					vftheta = 0;
//
//					tfx = 4;
//					tfy = 1;
//					tftheta = 1;
//
//					Xtrajec.t = 0;
//					Ytrajec.t = 0;
//					ThetaTrajec.t = 0;
//
//					Pfx = Odo.poseX;
//					P0x = Odo.poseX;
//					Pfy = Odo.poseY;
//					P0y = Odo.poseY;
//					v0x = Fkine.uOut;
//					v0y = Fkine.vOut;
//					Pftheta = 0;
//					P0theta = yaw;
//					Odo.poseTheta = yaw;
//
//					StopUsePIDX = 0;
//					StopUsePIDY = 1;
//					StopUsePIDr = 0;
//					stateChange = 0;
//					stateRun ++;
//				}
//			}
//			if(stateRun == 37){
//				v = 0.2;
//				stateChange++;
//				if(stateChange> 24){
//					u = 0;
//					v = 0;
//					r = 0;
//					StopUseXY = 1;
//					StopUsetheta = 1;
//					stateChange = 0;
//					stateRun++;
//				}
//			}
//			if(stateRun == 38){
//				stateChange++;
//				if(stateChange>1){
//					StopEnc(1);
//					stateChange = 0;
//					stateRun ++;
//					canShoot();
//					StopEnc(0);
//				}
//			}
//	//			if(stateChange == 0)
//	//			{
//	//				if ((absFloat(Odo.poseX-Pfx)<0.02)&&(absFloat(Odo.poseY-Pfy)<0.02)){
//	//					shoot = 1;
//	//					stateChange ++;
//	//					StopUseXY = 1;
//	//				}
//	//			}
//	//			if(stateChange == 1)
//	//			{
//	//				StopUseXY = 1;
//	//				PD_setParam(&pDTheta, 2, 0, alphaCtrol);
//	//				if ((absFloat(yaw-Pftheta)<=1*M_PI/180)){
//	//					steadycheck ++ ;
//	//				}else{
//	//					StopUsetheta = 0;
//	//					steadycheck = 0;
//	//				}
//	//
//	//			}
//	//
//	//			if(steadycheck > 10)
//	//			{
//	//				PD_setParam(&pDTheta, 1.2, 0, alphaCtrol);
//	////				valve_BothRelease();
//	//				Odo.poseTheta = yaw;
//	//				stateChange = 0;
//	//				u = 0;
//	//				v = 0;
//	//				r = 0;
//	//				v0x = 0;
//	//				v0y = 0;
//	////				stateRun += 1;
//	//				steadycheck = 0;
//	////				StopUseXY = 0;
//	////				StopUsetheta = 0;
//	//			}
//
//	//		if (stateRun == 27){
//	//			vfx = 0;
//	//			vfy = 0;
//	//			vftheta = 0;
//	//
//	//			tfx = 2;
//	//			tfy = 2;
//	//			tftheta = 2;
//	//			Xtrajec.t = 0;
//	//			Ytrajec.t = 0;
//	//			ThetaTrajec.t = 0;
//	//
//	//			Pfx = -0.7;
//	//			P0x = Odo.poseX;
//	//			Pfy = Odo.poseY;
//	//			P0y = Odo.poseY;
//	//			v0x = Fkine.uOut;
//	//			v0y = Fkine.vOut;
//	//			Pftheta = yaw;
//	//			P0theta = yaw;
//	//			Odo.poseTheta = yaw;
//	//			stateRun += 1;
//	//		}
//	//////MMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMM//
//	//////--------------------------------------State 24 ---------------------------------------------------//
//	//		if (stateRun == 28){
//	//			if(stateChange == 0){
//	//				if ((absFloat(Odo.poseX-Pfx)<0.01)&&(absFloat(Odo.poseY-Pfy)<0.01)){
//	//					stateChange++;
//	//					}
//	//			}
//	//			if(stateChange == 1)
//	//			{
//	//				if ((absFloat(Odo.poseX-Pfx)<0.03)&&(absFloat(Odo.poseY-Pfy)<0.03)){
//	//					steadycheck ++ ;
//	//					StopUsetheta = 1;
//	//					StopUseXY = 1;
//	//				}
//	//				else{
//	//					steadycheck = 0;
//	//					StopUseXY = 0;
//	//				}
//	//
//	//				if(steadycheck>10){
//	//					stateChange ++;
//	//					StopUsetheta = 0;
//	//					steadycheck = 0;
//	//				}
//	//			}
//	//			if(stateChange == 2)
//	//			{
//	//				PD_setParam(&pDTheta, 2, 0, alphaCtrol);
//	//				if ((absFloat(yaw-Pftheta)<=1*M_PI/180)){
//	//					StopUseXY = 1;
//	//					steadycheck ++ ;
//	//				}else{
//	//					StopUsetheta = 0;
//	//					steadycheck = 0;
//	//				}
//	//
//	//			}
//	//
//	//			if(steadycheck > 15)
//	//			{
//	//				PD_setParam(&pDTheta, 1.2, 0, alphaCtrol);
//	//				Odo.poseTheta = yaw;
//	//				stateChange = 0;
//	//				u = 0;
//	//				v = 0;
//	//				r = 0;
//	//				v0x = 0;
//	//				v0y = 0;
//	////				stateRun += 1;
//	//				steadycheck = 0;
//	////				StopUseXY = 0;
//	////				StopUsetheta = 0;
//	//			}
//
//	//		}
//	//MMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMM//
//	//--------------------------------------Main--- ---------------------------------------------------//
//			if(GamePad.Touch== 1){
//				Run = 1;
//			}
//			if(Run == 1)
//			{
//				Xtrajec.t += DeltaT;
//				Ytrajec.t += DeltaT;
//				ThetaTrajec.t += DeltaT;
//				TrajecPlanning(&Xtrajec,P0x, Pfx, tfx, v0x, vfx);
//				TrajecPlanning(&Ytrajec,P0y, Pfy, tfy, v0y, vfy);
//				TrajecPlanning(&ThetaTrajec,P0theta, Pftheta, tftheta, v0theta, vftheta);
//
//	//MMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMM//
//	//------------------------------------State constrain----------------------------------------------//
//	if((stateRun<4)||((stateRun>8)&&(stateRun<15))||(stateRun>18)){
//		if((stateRun != 12)&&(stateRun != 22)&&(stateRun !=7)&&(stateRun !=17)&&(stateRun != 23))
//		{
//			if(StopUseXY == 0)
//			{
//				if(!StopUsePIDX){
//					u = pDX.u;
//				}
//				if(!StopUsePIDY){
//					v = pDY.u ;
//				}
//
//			}else{
//				u = 0;
//				v = 0;
//			}
//
//			if(StopUsetheta == 0)
//			{
//				if(!StopUsePIDr){
//					r = pDTheta.u;
//				}
//			}else{
//				r = 0;
//			}
//		}
//	}
//	//MMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMM//
//	//--------------------------------------State 0 ---------------------------------------------------//
//				if(stateRun == 0){
//					stateChange++;
//					u = 0.1;
//					v = 0;
//					r = 0;
//					Odo.poseX = 0;
//					Odo.poseY = 0;
//					Odo.poseTheta = 0;
//					if(stateChange>5){
//
//						stateChange = 0;
//						breakProtect = 1;
//						resetParam = 1;
//
//						tfx = 0.5;
//						tfy = 0.5;
//						tftheta = 1;
//
//						Xtrajec.t = 0;
//						Ytrajec.t = 0;
//						ThetaTrajec.t = 0;
//
//						Pfx = -0.28;
//						P0x = Odo.poseX;
//						Pfy = -0.25;
//						P0y = Odo.poseY;
//						Pftheta = 0;
//						P0theta = 0;
//
//						vfx = 0;
//						vfy = 0;
//						vftheta = 0;
//
//						stateRun += 1;
//					}
//				}
//			}
//
			if(GamePad.Touch == 1){
				Run = 1;
			}
			if(GamePad.Triangle == 1){
				Gamepad = 1;
			}
			if(Gamepad == 0){
				uControlX = 	process_GetUControl();
				uControlY = 	process_GetVControl();
				uControlTheta = process_GetRControl();
			}else {
				uControlX = GamePad.YLeftCtr;
				uControlY = GamePad.XLeftCtr;
				uControlTheta = GamePad.XRightCtr;
			}

			osDelay(DELTA_T*1000 - IMU_Wait);

		}
  /* USER CODE END OdometerHandle */
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
	while (1) {
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
