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
//#include "LogData.c"
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
#include "ProcessControlRefactor.h"
//#include "LogData.h"
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
UART_HandleTypeDef huart2;
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
QueueHandle_t qControlUVR;
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
static void MX_USART2_UART_Init(void);
void StartDefaultTask(void const * argument);
void InverseKinematic(void const * argument);
void CAN_Bus(void const * argument);
void Actuator(void const * argument);
void OdometerHandle(void const * argument);

/* USER CODE BEGIN PFP */
void setHomeComplete();
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
void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan) {
	while (1);
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
		pInt = (uint8_t*) &CurrAngle;
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



void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
	__HAL_UART_CLEAR_OREFLAG(huart);
	memset(UARTRX3_Buffer, 0, sizeof(UARTRX3_Buffer));
	HAL_UART_Receive_IT(&huart3, (uint8_t*) UARTRX3_Buffer, 9);
	__HAL_UART_DISABLE(huart);
}
//void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
//	log_TransmitCompleteHandle(huart);
//}

/*=============================== Other Functions ===============================*/
void setHomeComplete()
{

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
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
//  	log_Init(&huart2);
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
	qControlUVR = xQueueCreate(5, sizeof(ProcessOutputResult));
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
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOD_CLK_ENABLE();

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
	uint8_t RunStage = 0;
	swer_Init();

	for (;;) {
		if(((GamePad.Square == 1) && (GamePad.Circle == 1)) || RunStage == 50) {
			uint32_t untangleBLDC = 1;
			xTaskNotify(TaskInvKineHandle, untangleBLDC, eSetValueWithOverwrite);
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
	uint32_t untangleBLDC;
	ProcessOutputResult result;
	/* Infinite loop */
	for (;;) {

		if(xTaskNotifyWait(pdFALSE, pdFALSE, &untangleBLDC, 0)) ;
		if(untangleBLDC == 0 && xQueueReceive(qControlUVR, (void*) &result, 10) == pdTRUE) {
			for (ModuleID id = MODULE_ID_1; id <= MODULE_ID_3; id++)
				invkine_Implementation(id, result.uControl, result.vControl, result.rControl, &InvCpltCallback);
		} else {
			InvCpltCallback(MODULE_ID_3, 0, 0);
			InvCpltCallback(MODULE_ID_1, 0, 0);
			InvCpltCallback(MODULE_ID_2, 0, 0);
		}
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
//		process_RunSSAndActuator(&TestBreakProtection);
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


//		float X = odo_GetPoseX();
//		float Y = odo_GetPoseY();
//		float THETA = odo_GetPoseTheta();
//
//		log_AddArgumentToBuffer((void *)&X,TYPE_FLOAT);
//		log_AddArgumentToBuffer((void *)&Y,TYPE_FLOAT);
//		log_AddArgumentToBuffer((void *)&THETA,TYPE_FLOAT);
//
//		log_SendString();



		osDelay(500);
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
		process_Init();
	bool Run = false;
	ProcessOutputResult result;
		/* Infinite loop */
	for (;;) {
			if(!shootFlag){
				RTR_SpeedAngle();
			}
			if(GamePad.Up==1 && GamePad.Triangle==1){
			Run = true;
			}
		odo_SpeedAngleUpdate();
		result = process_Run(Run);
		xQueueSend(qControlUVR, (void* )&result, 10);
		osDelay(DELTA_T * 1000);

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
