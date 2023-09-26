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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "CAN_Control.h"
#include "CAN_FuncHandle.h"
#include "Encoder.h"
#include "BoardParameter.h"
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

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

/* USER CODE BEGIN PV */
PID_Param pid;
bool enableSendPID = 0;
PID_type type;

float bldcSpeed = 500.35;
float dcAngle = 20.24;
bool enableSendSpeedAndAngle = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
void SendPID(PID_Param pid, CAN_DEVICE_ID targetID, PID_type type, bool *enableSendPID);
void SendSpeedAndRotation(CAN_DEVICE_ID targetID);
void SendEncoderX4BLDC(CAN_DEVICE_ID targetID);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan){
	 canctrl_Receive(hcan, CAN_RX_FIFO0);
	 HAL_GPIO_TogglePin(UserLED_GPIO_Port, UserLED_Pin);
}
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan){
	canctrl_Receive(hcan, CAN_RX_FIFO1);
	HAL_GPIO_TogglePin(UserLED_GPIO_Port, UserLED_Pin);
}

void CAN_Init(){

	HAL_CAN_Start(&hcan);
	HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_RX_FIFO1_MSG_PENDING);
//	uint16_t deviceID = *(__IO uint32_t*)FLASH_ADDR_TARGET << CAN_DEVICE_POS;
//	canctrl_Filter_List16(&hcan,
//			deviceID | CANCTRL_MODE_ENCODER,
//			deviceID | CANCTRL_MODE_LED_BLINK,
//			deviceID | CANCTRL_MODE_ENCODER_SEND_ENABLE,
//			deviceID | CANCTRL_MODE_MOTOR_SPEED_ANGLE,
//			0, CAN_RX_FIFO0);
//	canctrl_Filter_List16(&hcan,
//			deviceID | CANCTRL_MODE_PID_BLDC_SPEED,
//			deviceID | CANCTRL_MODE_PID_DC_ANGLE,
//			deviceID | CANCTRL_MODE_PID_DC_SPEED,
//			deviceID | CANCTRL_MODE_SET_HOME,
//			1, CAN_RX_FIFO0);
	canctrl_Filter_Mask16(&hcan, 0, 0, 0, 0, 0, CAN_RX_FIFO0);
}



void ledBlink(uint16_t delay)
{
	static uint32_t tick = 0;
	static uint16_t internalDelay = 100;
	if(delay) internalDelay = delay;
	if(HAL_GetTick() - tick > internalDelay){
		tick = HAL_GetTick();
		HAL_GPIO_TogglePin(UserLED_GPIO_Port, UserLED_Pin);
	}
}
void ledState(bool state){
	HAL_GPIO_WritePin(UserLED_GPIO_Port, UserLED_Pin, state);
}

void handleFunc(CAN_MODE_ID func){

	switch(func){
	case CANCTRL_MODE_ENCODER_SEND_ENABLE:
		break;
	case CANCTRL_MODE_LED_STATE:
		break;
	case CANCTRL_MODE_ENCODER:
		int32_t count_X4 = (int32_t)canfunc_MotorGetEncoderPulseBLDC();
		brd_SetEncX4BLDC(count_X4);
		break;
	case CANCTRL_MODE_LED_BLINK:// only do once
		ledBlink(canctrl_GetIntNum());
		break;
	case CANCTRL_MODE_SHOOT:
		break;
	case CANCTRL_MODE_SET_HOME:
		__NOP();
		break;
	case CANCTRL_MODE_MOTOR_SPEED_ANGLE:
		float bldcSpeed,dcAngle;
		canfunc_MotorGetSpeedAndAngle(&bldcSpeed, &dcAngle);
		brd_SetAngleDC(dcAngle);
		brd_SetSpeedBLDC(bldcSpeed);
		break;
	case CANCTRL_MODE_MOTOR_BLDC_BRAKE:
		bool brake = canfunc_MotorGetBrake();
		MotorBLDC mbldc = brd_GetObjMotorBLDC();
		MotorBLDC_Brake(&mbldc, brake);
		break;
	case CANCTRL_MODE_PID_DC_SPEED:
	case CANCTRL_MODE_PID_DC_ANGLE:
	case CANCTRL_MODE_PID_BLDC_SPEED:
		canfunc_GetPID();
		break;
	case CANCTRL_MODE_START:
	case CANCTRL_MODE_END:
		break;
	}
}


void Flash_Write(CAN_DEVICE_ID ID){
	uint32_t targetAddr = FLASH_ADDR_BASE + 1024*64;
	  FLASH_EraseInitTypeDef fe;
	  fe.TypeErase = FLASH_TYPEERASE_PAGES;
	  fe.PageAddress = targetAddr;
	  fe.NbPages = 1;
	  fe.Banks = FLASH_BANK_1;
	  uint32_t pageErr = 0;
	  HAL_FLASH_Unlock();
	  if(HAL_FLASHEx_Erase(&fe, &pageErr) != HAL_OK){
//		 return HAL_FLASH_GetError();
		  while(1);
	  }
	  if(HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, FLASH_ADDR_TARGET,(uint32_t)ID) != HAL_OK){
//		 return HAL_FLASH_GetError();
		  while(1);
	  }
	  HAL_FLASH_Lock();
}

void testSetParamPID()
{
  pid.kP = 1.02;
  pid.kI = 0.14;
  pid.kD = 20.2;
  pid.alpha = -0.2;
  pid.deltaT = 0.01;
  type = PID_BLDC_SPEED;
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
  MX_CAN_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  brd_Init();
  CAN_Init();
  testSetParamPID();
//  uint8_t data = 10;
//  canctrl_PutMessage((void*)&data, sizeof(data));
//  canctrl_Send(&hcan, CANCTRL_DEVICE_MOTOR_CONTROLLER_1);
//  Flash_Write(CANCTRL_DEVICE_MOTOR_CONTROLLER_1);
//  uint32_t canSlaveAddr = *(__IO uint32_t*)FLASH_ADDR_TARGET;
//  __HAL_DBGMCU_FREEZE_CAN1();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  canfunc_HandleRxEvent(&handleFunc);
	  SendPID(pid, CANCTRL_DEVICE_MOTOR_CONTROLLER_1, type, &enableSendPID);
	  SendSpeedAndRotation(CANCTRL_DEVICE_MOTOR_CONTROLLER_1);
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
  hcan.Init.Prescaler = 18;
  hcan.Init.Mode = CAN_MODE_LOOPBACK;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_2TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_1TQ;
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
  htim2.Init.Prescaler = 6;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 999;
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
  HAL_GPIO_WritePin(GPIOA, BLDC_DIR_Pin|BLDC_BRAKE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(UserLED_GPIO_Port, UserLED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : BLDC_DIR_Pin BLDC_BRAKE_Pin */
  GPIO_InitStruct.Pin = BLDC_DIR_Pin|BLDC_BRAKE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : UserLED_Pin */
  GPIO_InitStruct.Pin = UserLED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(UserLED_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void SendEncoderX4BLDC(CAN_DEVICE_ID targetID)
{
	Encoder_t encBLDC = brd_GetObjEncBLDC();
	canfunc_MotorPutEncoderPulseBLDC(encBLDC.count_X4);
	canctrl_Send(&hcan, targetID);
}

void SendSpeedAndRotation(CAN_DEVICE_ID targetID)
{
	if(!enableSendSpeedAndAngle) return;
	canfunc_MotorPutSpeedAndAngle(bldcSpeed, dcAngle);
	canctrl_Send(&hcan, targetID);
	enableSendSpeedAndAngle = false;
}

void SendPID(PID_Param pid, CAN_DEVICE_ID targetID, PID_type type, bool *enableSendPID)
{
	if(!canfunc_GetStateEnableSendPID() && *enableSendPID) {
			canfunc_EnableSendPID();
			*enableSendPID = 0;
	}
	uint32_t canEvent = canctrl_GetEvent();
	if(canEvent) return;
	canfunc_PutAndSendParamPID(&hcan,targetID,pid,type);
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
