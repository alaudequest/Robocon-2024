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
#include "PIDPosition.h"
#include "ActuatorValve.h"
#include "AppInterface.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum MainEvent
{
	MEVT_GET_NODE_SPEED_ANGLE,
} MainEvent;
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

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

osThreadId defaultTaskHandle;
osThreadId TaskInvKineHandle;
uint32_t TaskInvKineBuffer[256];
osStaticThreadDef_t TaskInvKineControlBlock;
osThreadId TaskCANHandle;
uint32_t TaskCANBuffer[128];
osStaticThreadDef_t TaskCANControlBlock;
osThreadId TaskActuatorHandle;
osThreadId TaskOdometerHandle;
/* USER CODE BEGIN PV */

CAN_DEVICE_ID targetID = CANCTRL_DEVICE_MOTOR_CONTROLLER_4;
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
CAN_SpeedBLDC_AngleDC nodeSpeedAngle[3] = {0};

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

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM10_Init(void);
static void MX_USART2_UART_Init(void);
void StartDefaultTask(void const *argument);
void InverseKinematic(void const *argument);
void CAN_Bus(void const *argument);
void Actuator(void const *argument);
void OdometerHandle(void const *argument);

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
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {

	appinf_HandleReceive(huart);
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
	MX_CAN1_Init();
	MX_TIM2_Init();
	MX_TIM3_Init();
	MX_USART3_UART_Init();
	MX_TIM1_Init();
	MX_TIM10_Init();
	MX_USART2_UART_Init();
	/* USER CODE BEGIN 2 */

	HAL_UART_Receive_IT(&huart3, (uint8_t*) UARTRX3_Buffer, 9);
	appinf_Init(&huart2);
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
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
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
	htim10.Init.Prescaler = 160 - 1;
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
	HAL_GPIO_WritePin(GPIOA, HC595_CLK_Pin | HC595_RCLK_Pin | HC595_OE_Pin | HC595_DATA_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pins : HC595_CLK_Pin HC595_RCLK_Pin HC595_OE_Pin HC595_DATA_Pin */
	GPIO_InitStruct.Pin = HC595_CLK_Pin | HC595_RCLK_Pin | HC595_OE_Pin | HC595_DATA_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pins : SSLua1_Pin SSLua2_Pin */
	GPIO_InitStruct.Pin = SSLua1_Pin | SSLua2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

	/* USER CODE BEGIN MX_GPIO_Init_2 */
	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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
int NopeCycle = 3500;

float preGyro;
void RTR_SpeedAngle() {
//	HAL_TIM_Base_Start(&htim10);
//	__HAL_TIM_SET_COUNTER(&htim10,0);
	for (uint8_t i = 0; i < 3; i++) {
		canctrl_SetID(CANCTRL_MODE_NODE_REQ_SPEED_ANGLE);
		bool a = 1;
		canctrl_PutMessage((void*) &a, sizeof(bool));
		targetID = i + 1;
		while (canctrl_Send(&hcan1, targetID) != HAL_OK);
		for (uint16_t i = 0; i < NopeCycle; i++)
			__NOP();
	}
//	count = __HAL_TIM_GET_COUNTER(&htim10);
//	HAL_TIM_Base_Stop(&htim10);
}

#define DeltaT	0.05
#define PulsePerRev 200*2.56

#define dy1	0
#define dx1	0.37545
#define dy2 0.23373/2
#define dx2 -0.07171
#define dy3 -0.23373/2
#define dx3	-0.07171
//#define a 0.045
typedef struct SpeedReadSlave {
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
} SpeedReadSlave;

typedef struct ForwardKine {
	float uOut;
	float vOut;
	float thetaOut;
} ForwardKine;

typedef struct SwerveOdoHandle {
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
} SwerveOdoHandle;

void SpeedRead2(SpeedReadSlave *sp, float count)
{
	sp->V = 0.044 * 2 * M_PI * ((count - sp->preCount) / DeltaT) / (PulsePerRev * 4);
	sp->Vfilt = (1 - sp->filterAlpha) * sp->VfiltPre + sp->filterAlpha * sp->V;
	sp->VfiltPre = sp->Vfilt;

	sp->preCount = count;
}

void omegaToZeta(ForwardKine *kine, float *VxA, float *VyA)
{
	kine->uOut = -0.2357 * VxA[0] + 0.2357 * VyA[0] - 0.2357 * VxA[1] - 0.2357 * VyA[1] + 0.3333 * VxA[2] + 0.0000 * VyA[2];
	kine->vOut = -0.2467 * VxA[0] - 0.3262 * VyA[0] + 0.2467 * VxA[1] - 0.3262 * VyA[1] - 0.0000 * VxA[2] + 0.1898 * VyA[2];
	kine->thetaOut = 0.1417 * VxA[0] + 1.1707 * VyA[0] - 0.1417 * VxA[1] + 1.1707 * VyA[1] + 0.0000 * VxA[2] + 1.8560 * VyA[2];
}

SwerveOdoHandle Odo;
ForwardKine Fkine;
SpeedReadSlave Module[4];
float Vx[4], Vy[4];
float Gyro;

#define UABOVE_X 	0.3
#define UBELOW_X	-0.3

#define UABOVE_Y 	0.3
#define UBELOW_Y	-0.3

#define UABOVE_THETA 	5
#define UBELOW_THETA	-5

#define ROBOT_RADIUS	0.25;

typedef struct PDParam {
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
} PDParam;

void PD_Controller(PDParam *pd, float Target, float Current)
{
	pd->e = Target - Current;
	pd->uP = pd->kP * pd->e;
	pd->uD = pd->kD * (pd->e - pd->pre) / DeltaT;
	pd->uDf = (1 - pd->Alpha) * pd->uDfpre + (pd->Alpha) * pd->uD;
	pd->uDfpre = pd->uDf;
	pd->pre = pd->e;

	pd->u = pd->uP + pd->uD;
	if (pd->u > pd->uAbove)
		pd->u = pd->uAbove;
	else if (pd->u < pd->uBelow) pd->u = pd->uBelow;
}

void PD_setParam(PDParam *pd, float kP, float kD, float Alpha)
{
	pd->kP = kP;
	pd->kD = kD;
	pd->Alpha = Alpha;
}

void PD_setLimit(PDParam *pd, float uAbove, float uBelow)
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
float absFloat(float num) {
	if (num < 0.0) {
		return num * -1.0;
	}
	else
		return num;
}
float min(float a, float b) {
	float min;
	min = a;
	if (b <= min) {
		min = b;
	}
	return min;
}

float max(float a, float b) {
	float max;
	max = a;
	if (b >= max) {
		max = b;
	}
	return max;
}
int Isteady(float e, float thresthold)
{
	if (absFloat(e) < thresthold) {
		return 1;
	}

	return 0;
}
float PointDistances(float *pt1, float *pt2)
{
	float Distance = 0.0;
	Distance = sqrtf(powf((pt2[0] - pt1[0]), 2.0) + powf((pt2[1] - pt1[1]), 2.0));
	return Distance;
}
int equalCompare(float a, float b)
{
	if (absFloat(a - b) < 0.0001) {
		return 1;
	}
	return 0;
}
float path[7][2] = {
		{0, 0},
		{0.35, -0.35},
		{1, -0.35},
		{2.5, -0.35},
		{3, -0.1},
		{3, 0},
		{0, 0},
};

int sgn(float num) {
	if (num >= 0)
		return 1;
	else
		return -1;
}

float solPtn1[2] = {0, 0};
float solPtn2[2] = {0, 0};
float currPtn[2] = {0, 0};
float goalPtn[2] = {0, 0};
float lastgoalPtn[2] = {0, 0};

int lFindex = 0, stIndex = 0;
float lookAheadDis = 0.45;
float X1, Y1, X2, Y2;
float dx, dy, dr, D;
float discriminant;
float minX, minY, maxX, maxY;
float pathlen;
int len;
void Purepursuilt(SwerveOdoHandle *od) {
	currPtn[0] = od->poseX;
	currPtn[1] = od->poseY;
//	pathlen = sizeof(path) / sizeof(path[0]);
	stIndex = lFindex;
	for (int i = stIndex; i < len - 1; i++)
			{
		X1 = path[i][0] - currPtn[0];
		Y1 = path[i][1] - currPtn[1];

		X2 = path[i + 1][0] - currPtn[0];
		Y2 = path[i + 1][1] - currPtn[1];

		dx = X2 - X1;
		dy = Y2 - Y1;

		dr = sqrtf(dx * dx + dy * dy);
		D = X1 * Y2 - X2 * Y1;

		discriminant = (lookAheadDis * lookAheadDis) * (dr * dr) - D * D;

		if (discriminant >= 0) {
			solPtn1[0] = (D * dy + (float) sgn(dy) * dx * sqrtf(discriminant)) / powf(dr, 2);
			solPtn2[0] = (D * dy - (float) sgn(dy) * dx * sqrtf(discriminant)) / powf(dr, 2);
			solPtn1[1] = (-D * dx + absFloat(dy) * sqrtf(discriminant)) / powf(dr, 2);
			solPtn2[1] = (-D * dx - absFloat(dy) * sqrtf(discriminant)) / powf(dr, 2);

			solPtn1[0] += currPtn[0];
			solPtn1[1] += currPtn[1];

			solPtn2[0] += currPtn[0];
			solPtn2[1] += currPtn[1];

			minX = min(path[i][0], path[i + 1][0]);
			minY = min(path[i][1], path[i + 1][1]);
			maxX = max(path[i][0], path[i + 1][0]);
			maxY = max(path[i][1], path[i + 1][1]);

			if (
			(((minX <= solPtn1[0] && solPtn1[0] <= maxX) && (minY <= solPtn1[1] && solPtn1[1] <= maxY))
					|| ((equalCompare(solPtn1[0], minX) == 1) || (equalCompare(solPtn1[0], maxX) == 1) || (equalCompare(solPtn1[1], minY) == 1) || (equalCompare(solPtn1[1], maxY) == 1)))

			||

			(((minX <= solPtn2[0] && solPtn2[0] <= maxX) && (minY <= solPtn2[1] && solPtn2[1] <= maxY))
					|| ((equalCompare(solPtn2[0], minX) == 1) || (equalCompare(solPtn2[0], maxX) == 1) || (equalCompare(solPtn2[1], minY) == 1) || (equalCompare(solPtn2[1], maxY) == 1)))
					)
					{
				if (
				(((minX <= solPtn1[0] && solPtn1[0] <= maxX) && (minY <= solPtn1[1] && solPtn1[1] <= maxY))
						|| ((equalCompare(solPtn1[0], minX) == 1) || (equalCompare(solPtn1[0], maxX) == 1) || (equalCompare(solPtn1[1], minY) == 1) || (equalCompare(solPtn1[1], maxY) == 1)))

				&&

				(((minX <= solPtn2[0] && solPtn2[0] <= maxX) && (minY <= solPtn2[1] && solPtn2[1] <= maxY))
						|| ((equalCompare(solPtn2[0], minX) == 1) || (equalCompare(solPtn2[0], maxX) == 1) || (equalCompare(solPtn2[1], minY) == 1) || (equalCompare(solPtn2[1], maxY) == 1)))
						) {
					if (PointDistances(solPtn1, path[i + 1]) < PointDistances(solPtn2, path[i + 1])) {
						goalPtn[0] = solPtn1[0];
						goalPtn[1] = solPtn1[1];
					} else {
						goalPtn[0] = solPtn2[0];
						goalPtn[1] = solPtn2[1];
					}
				}
				else {
					if
					(((minX <= solPtn1[0] && solPtn1[0] <= maxX) && (minY <= solPtn1[1] && solPtn1[1] <= maxY))
							|| ((equalCompare(solPtn1[0], minX) == 1) || (equalCompare(solPtn1[0], maxX) == 1) || (equalCompare(solPtn1[1], minY) == 1) || (equalCompare(solPtn1[1], maxY) == 1)))
							{
						goalPtn[0] = solPtn1[0];
						goalPtn[1] = solPtn1[1];
					}
					else {
						goalPtn[0] = solPtn2[0];
						goalPtn[1] = solPtn2[1];
					}
				}

				if (PointDistances(goalPtn, path[i + 1]) < PointDistances(currPtn, path[i + 1])) {
					lFindex = i;
					break;
				} else if (lFindex == (len - 2)) {
					goalPtn[0] = path[len - 1][0];
					goalPtn[1] = path[len - 1][1];
					lFindex += 1;
					break;
				} else {

					lFindex += 1;
				}
			}
		} else {
			goalPtn[0] = path[lFindex][0];
			goalPtn[1] = path[lFindex][1];
		}
	}
}

typedef struct Trajecparam {
	float t;
	float a0, a1, a2, a3;
	float xTrajec, xdottraject;
} Trajecparam;

Trajecparam Xtrajec, Ytrajec, ThetaTrajec;
float t;
float a0, a1, a2, a3;
float xTrajec, xdottraject;

float P0x, Pfx, tfx, v0x, vfx;
float P0y, Pfy, tfy, v0y, vfy;
float P0theta, Pftheta, tftheta, v0theta, vftheta;
void TrajecPlanning(Trajecparam *trajec, float P0, float Pf, float tf, float v0, float vf) {
	trajec->a0 = P0;
	trajec->a1 = v0;
	trajec->a2 = (3 / (tf * tf)) * (Pf - P0) - (2 / tf) * v0 - (1 / tf) * vf;
	trajec->a3 = (-2 / (tf * tf * tf)) * (Pf - P0) + (1 / (tf * tf)) * (vf + v0);

	if (trajec->t > tf) trajec->t = tf;

	trajec->xTrajec = trajec->a0 + trajec->a1 * trajec->t + trajec->a2 * trajec->t * trajec->t + trajec->a3 * trajec->t * trajec->t * trajec->t;
	trajec->xdottraject = trajec->a1 + 2 * trajec->a2 * trajec->t + 3 * trajec->a3 * trajec->t * trajec->t;

	if (Pf == P0) {
		trajec->xTrajec = Pf;
		trajec->xdottraject = 0;
	}
}

uint8_t Run, resetParam, breakProtect;
float kpX = 1, kpY = 1, kdX, kdY, kpTheta = 0.8, kdTheta, alphaCtrol = 0.2;
int testSpeed, testPos;
float uControlX, uControlY, uControlTheta;
uint8_t stateRun, steadycheck;
uint8_t ssCheck;
uint8_t stateChange;
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
 * @brief  Function implementing the defaultTask thread.
 * @param  argument: Not used
 * @retval None
 */

/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const *argument)
{
	/* USER CODE BEGIN 5 */
	/* Infinite loop */

	swer_Init();

	for (;;) {
		if (Run == 0) {
			u = GamePad.YLeftCtr;
			v = GamePad.XLeftCtr;
			r = -GamePad.XRightCtr;
		}
		invkine_Implementation(MODULE_ID_3, uControlX, uControlY, uControlTheta, &InvCpltCallback);
		invkine_Implementation(MODULE_ID_1, uControlX, uControlY, uControlTheta, &InvCpltCallback);
		invkine_Implementation(MODULE_ID_2, uControlX, uControlY, uControlTheta, &InvCpltCallback);
//		InvCpltCallback(MODULE_ID_2, testSpeed, testPos) ;
//		if(nodeSwerveSetHomeComplete == 30)
//		{
//			if (!stopFlag) {
//				if ((GamePad.Square == 1)||(Break==1)) {
//					TestBreakProtection();
//					Break = 0;
//					u = v = r = 0;
//				} else {
//					invkine_Implementation(MODULE_ID_1, u, v, r, &InvCpltCallback);
//					invkine_Implementation(MODULE_ID_2, u, v, r, &InvCpltCallback);
//					invkine_Implementation(MODULE_ID_3, u, v, r, &InvCpltCallback);
//				}
//			}
//		}
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
void InverseKinematic(void const *argument)
{
	/* USER CODE BEGIN InverseKinematic */

	/* Infinite loop */
	for (;;) {

		osDelay(20);
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
void CAN_Bus(void const *argument)
{
	/* USER CODE BEGIN CAN_Bus */
	CAN_Init();
	canctrl_RTR_TxRequest(&hcan1, CANCTRL_DEVICE_MOTOR_CONTROLLER_1, CANCTRL_MODE_SET_HOME);
	osDelay(1);
	canctrl_RTR_TxRequest(&hcan1, CANCTRL_DEVICE_MOTOR_CONTROLLER_2, CANCTRL_MODE_SET_HOME);
	osDelay(1);
	canctrl_RTR_TxRequest(&hcan1, CANCTRL_DEVICE_MOTOR_CONTROLLER_3, CANCTRL_MODE_SET_HOME);
	osDelay(1);
//	canctrl_RTR_TxRequest(&hcan1, CANCTRL_DEVICE_MOTOR_CONTROLLER_4, CANCTRL_MODE_SET_HOME);
//	osDelay(1);
	uint32_t modeID;
	/* Infinite loop */
	for (;;) {
		if (xTaskNotifyWait(pdFALSE, pdFALSE, &modeID, portMAX_DELAY)) {
			CAN_RxHeaderTypeDef rxHeader = canctrl_GetRxHeader();
			uint32_t targetID = rxHeader.StdId >> CAN_DEVICE_POS;
			if ((modeID == CANCTRL_MODE_SET_HOME || modeID == CANCTRL_MODE_NODE_REQ_SPEED_ANGLE) && targetID) {
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
/* USER CODE END Header_Actuator */
void Actuator(void const *argument)
{
	/* USER CODE BEGIN Actuator */
	/* Infinite loop */
	for (;;) {
		if (stateRun == 4) {
			if (HAL_GPIO_ReadPin(SSLua1_GPIO_Port, SSLua1_Pin) && HAL_GPIO_ReadPin(SSLua2_GPIO_Port, SSLua2_Pin)) {
				ssCheck++;
			} else {
				ssCheck = 0;
			}

			if (ssCheck > 5) {
				stateRun++;
			}
		}

		//MMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMM//
		//--------------------------------------State 7 ---------------------------------------------------//
		if (stateRun == 7) {
			u = 0;
			v = 0;
			r = 0;
			valve_BothCatch();
			stateRun += 1;
		}
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
/* USER CODE END Header_OdometerHandle */
void OdometerHandle(void const *argument)
{
	/* USER CODE BEGIN OdometerHandle */

	PD_setParam(&pDX, 1, 0, 0.8);
	PD_setParam(&pDY, 1, 0, 0.8);
	PD_setParam(&pDTheta, 1.2, kdTheta, alphaCtrol);

	PD_setLimit(&pDX, 1, -1);
	PD_setLimit(&pDY, 1, -1);
	PD_setLimit(&pDTheta, 1, -1);

	len = sizeof(path) / sizeof(path[0]);
	tfx = 2;
	tfy = 3;
	tftheta = 1;

	valve_Init();
	/* Infinite loop */
	for (;;) {
		RTR_SpeedAngle();
		for (int i = 0; i < 3; i++)
				{
			SpeedRead2(&Module[i], nodeSpeedAngle[i].bldcSpeed);
			Module[i].Vx = Module[i].V * cos(nodeSpeedAngle[i].dcAngle * M_PI / 180);
			Module[i].Vy = Module[i].V * sin(nodeSpeedAngle[i].dcAngle * M_PI / 180);
			Vx[i] = Module[i].Vx;
			Vy[i] = Module[i].Vy;
		}

		omegaToZeta(&Fkine, Vx, Vy);

		Odo.poseTheta += Fkine.thetaOut * DeltaT;
		Odo.poseX += (Fkine.uOut * cos(Odo.poseTheta) - Fkine.vOut * sin(Odo.poseTheta)) * DeltaT;
		Odo.poseY += (Fkine.uOut * sin(Odo.poseTheta) + Fkine.vOut * cos(Odo.poseTheta)) * DeltaT;

		if (breakProtect == 1)
				{
			TestBreakProtection();
			breakProtect = 0;
		}
		if (resetParam == 1) {
			Odo.poseX = 0;
			Odo.poseY = 0;
			Odo.poseTheta = 0;
			resetParam = 0;
		}

		Odo.dTheta = Odo.poseTheta * 180 / M_PI;

//MMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMM//
//------------------------------------State constrain----------------------------------------------//
		if ((stateRun < 4) || (stateRun > 7)) {
			PD_Controller(&pDX, Xtrajec.xTrajec, Odo.poseX);
			PD_Controller(&pDY, Ytrajec.xTrajec, Odo.poseY);
			PD_Controller(&pDTheta, ThetaTrajec.xTrajec, Odo.poseTheta);
		}

//MMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMM//
//--------------------------------------State 1 ---------------------------------------------------//
		if ((stateRun == 1))
		{

			if ((absFloat(Odo.poseX - Pfx) < 0.02) && (absFloat(Odo.poseY - Pfy) < 0.02)) {
				stateRun += 1;
			}

		}
//MMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMM//
//--------------------------------------State 2 ---------------------------------------------------//
		if (stateRun == 2)
				{
			tfx = 2.5;
			tfy = 3;
			tftheta = 1;
			Xtrajec.t = 0;
			Ytrajec.t = 0;
			ThetaTrajec.t = 0;

			Pfx = -0.125;
			P0x = -0.3;
			Pfy = -0.9;
			P0y = 0;
			stateRun = 3;
		}
//MMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMM//
//--------------------------------------State 3 ---------------------------------------------------//
		if (stateRun == 3) {
			if ((absFloat(Odo.poseX - Pfx) < 0.03) && (absFloat(Odo.poseY - Pfy) < 0.03)) {
				stateRun += 1;
			}
//			if ((absFloat(Odo.poseX-Pfx)<0.01)&&(absFloat(Odo.poseY-Pfy)<0.01)){
//				pDX.u = 0;
//				pDY.u = 0;
//				steadycheck ++;
//			}else{
//				steadycheck = 0;
//			}
//
//			if(steadycheck > 5)
//			{
//				stateRun += 1;
//			}

		}
//MMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMM//
//--------------------------------------State 4 ---------------------------------------------------//
		if (stateRun == 4) {
			u = 0.05;
			v = -0.2;
			r = 0;
			Odo.poseTheta = 0;
		}
//MMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMM//
//--------------------------------------State 5 ---------------------------------------------------//
		if (stateRun == 5) {
			u = 0.05;
			v = -0.1;
			r = 0.05;
			Odo.poseTheta = 0;
			stateChange++;
			if (stateChange > 3)
					{
				stateChange = 0;
				stateRun++;
			}
		}
//MMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMM//
//--------------------------------------State 6 ---------------------------------------------------//
		if (stateRun == 6) {
			u = 0;
			v = 0.00001;
			r = 0;

//			breakProtect = 1;
			stateChange++;
			if (stateChange == 1)
					{
				Odo.poseTheta = 0;
				Odo.poseY = 0;
				Odo.poseX = 0;
				u = 0;
				v = 0;
				r = 0;
				TestBreakProtection();
			}
			if (stateChange > 3)
					{
				Odo.poseTheta = 0;
				Odo.poseY = 0;
				Odo.poseX = 0;
				u = 0;
				v = 0;
				r = 0;
				stateChange = 0;
				stateRun++;
			}
//			stateRun += 1 ;
		}

//MMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMM//
//--------------------------------------State 8 ---------------------------------------------------//
		if (stateRun == 8) {
			tfx = 1;
			tfy = 1;
			tftheta = 1;
			Xtrajec.t = 0;
			Ytrajec.t = 0;
			ThetaTrajec.t = 0;

			Pfx = -0.2;
			P0x = 0;
			Pfy = 0;
			P0y = 0;
			stateRun = 9;
		}
//MMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMM//
//--------------------------------------State 9 ---------------------------------------------------//
		if (stateRun == 9) {
			if ((absFloat(Odo.poseX - Pfx) < 0.03) && (absFloat(Odo.poseY - Pfy) < 0.03)) {
				stateRun += 1;
			}
		}
//MMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMM//
//--------------------------------------State 10 ---------------------------------------------------//

		if (stateRun == 10) {
			tfx = 4;
			tfy = 4;
			tftheta = 3;
			Xtrajec.t = 0;
			Ytrajec.t = 0;
			ThetaTrajec.t = 0;

			Pfx = -1;
			P0x = -0.2;
			Pfy = -1;
			P0y = 0;
			Pftheta = -90 * M_PI / 180;
			P0theta = 0;
			stateRun += 1;
		}
//MMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMM//
//--------------------------------------State 11 ---------------------------------------------------//
		if (stateRun == 11) {
			if ((absFloat(Odo.poseX - Pfx) < 0.02) && (absFloat(Odo.poseY - Pfy) < 0.02)) {
				pDX.u = 0;
				pDY.u = 0;
				pDTheta.u = 0;
				steadycheck++;
			} else {
				steadycheck = 0;
			}

			if (steadycheck > 3)
					{
				stateRun += 1;
			}
		}
//MMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMM//
//--------------------------------------State 12 ---------------------------------------------------//
		if (stateRun == 12) {
			TestBreakProtection();
			valve_BothRelease();
			stateRun += 1;
		}

//MMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMM//
//--------------------------------------State 13 ---------------------------------------------------//
		if (stateRun == 13) {
			tfx = 4;
			tfy = 4;
			tftheta = 3;
			Xtrajec.t = 0;
			Ytrajec.t = 0;
			ThetaTrajec.t = 0;

			Pfx = 0;
			P0x = Odo.poseX;
			Pfy = -0.2;
			P0y = Odo.poseY;
			Pftheta = 0 * M_PI / 180;
			P0theta = -90 * M_PI / 180;
			stateRun += 1;
		}
//MMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMM//
//--------------------------------------State 14 ---------------------------------------------------//
		if (stateRun == 14) {
			if ((absFloat(Odo.poseX - Pfx) < 0.01) && (absFloat(Odo.poseY - Pfy) < 0.01)) {
				pDX.u = 0;
				pDY.u = 0;
				steadycheck++;
			} else {
				steadycheck = 0;
			}

			if (steadycheck > 5)
					{
				stateRun += 1;
				TestBreakProtection();
			}
		}
//MMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMM//
//--------------------------------------Main--- ---------------------------------------------------//
		if (Run == 1)
				{
			Xtrajec.t += DeltaT;
			Ytrajec.t += DeltaT;
			ThetaTrajec.t += DeltaT;
			TrajecPlanning(&Xtrajec, P0x, Pfx, tfx, v0x, vfx);
			TrajecPlanning(&Ytrajec, P0y, Pfy, tfy, v0y, vfy);
			TrajecPlanning(&ThetaTrajec, P0theta, Pftheta, tftheta, v0theta, vftheta);
//			t += DeltaT;
//			TrajecPlanning(P0, Pf, tf, v0, vf);

			if (Isteady(pDX.e, 0.01)) {
				pDX.u = 0;
			}

			if (Isteady(pDY.e, 0.01)) {
				pDY.u = 0;
			}

			if (Isteady(pDX.e, 0.05) && Isteady(pDY.e, 0.05))
					{
				pDTheta.u = 0;
			}
//MMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMM//
//------------------------------------State constrain----------------------------------------------//
			if ((stateRun < 4) || (stateRun > 7)) {
				u = pDX.u + Xtrajec.xdottraject;
				v = pDY.u + Ytrajec.xdottraject;
				r = pDTheta.u + ThetaTrajec.xdottraject;
			}
//MMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMM//
//--------------------------------------State 0 ---------------------------------------------------//
			if (stateRun == 0) {
				breakProtect = 1;
				resetParam = 1;
				stateRun = 1;

				Xtrajec.t = 0;
				Ytrajec.t = 0;
				ThetaTrajec.t = 0;

				Pfx = -0.3;
				P0x = 0;
				Pfy = 0;
				P0y = 0;

				Pftheta = 0;
				P0theta = 0;
			}
		}
		uControlX = u * cos(-Odo.poseTheta) - v * sin(-Odo.poseTheta);
		uControlY = u * sin(-Odo.poseTheta) + v * cos(-Odo.poseTheta);
		uControlTheta = r;
//		Purepursuilt(&Odo);
		osDelay(DeltaT * 1000);

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
