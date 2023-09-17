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
#include <math.h>
#include <stdlib.h>
#include <String.h>
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
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

///* -------------------------Begin: Configure SLAVES---------------------------*/

uint8_t Tx_Buf[41];

void ControlDriver(
uint8_t ID1, uint8_t MODE1, float BLDCSpeed1, float DCPos1,
uint8_t ID2, uint8_t MODE2, float BLDCSpeed2, float DCPos2,
uint8_t ID3, uint8_t MODE3, float BLDCSpeed3, float DCPos3,
uint8_t ID4, uint8_t MODE4, float BLDCSpeed4, float DCPos4)
{
	uint8_t *pByte = NULL;

	/* Begin: Slave 1 */
	pByte = &ID1;
	Tx_Buf[0]=pByte[0];
	pByte = &MODE1;
	Tx_Buf[1]=pByte[0];
	pByte = &BLDCSpeed1;
	for(uint8_t i = 2; i < 6; i++)
	{
		Tx_Buf[i]=pByte[i-2];
	}
	pByte = &DCPos1;
	for(uint8_t i = 6; i < 10; i++)
	{
		Tx_Buf[i]=pByte[i-6];
	}
	/* End: Slave 1 */
	/* Begin: Slave 2 */
	pByte = &ID2;
	Tx_Buf[10]=pByte[0];
	pByte = &MODE2;
	Tx_Buf[11]=pByte[0];
	pByte = &BLDCSpeed2;
	for(uint8_t i = 12; i < 16; i++)
	{
		Tx_Buf[i]=pByte[i-12];
	}
	pByte = &DCPos2;
	for(uint8_t i = 16; i < 20; i++)
	{
		Tx_Buf[i]=pByte[i-16];
	}
	/* End: Slave 2 */
	/* Begin: Slave 3 */
	pByte = &ID3;
	Tx_Buf[20]=pByte[0];
	pByte = &MODE3;
	Tx_Buf[21]=pByte[0];
	pByte = &BLDCSpeed3;
	for(uint8_t i = 22; i < 26; i++)
	{
		Tx_Buf[i]=pByte[i-22];
	}
	pByte = &DCPos3;
	for(uint8_t i = 26; i < 30; i++)
	{
		Tx_Buf[i]=pByte[i-26];
	}
	/* End: Slave 3 */
	/* Begin: Slave 4 */
	pByte = &ID4;
	Tx_Buf[30]=pByte[0];
	pByte = &MODE4;
	Tx_Buf[31]=pByte[0];
	pByte = &BLDCSpeed4;
	for(uint8_t i = 32; i < 36; i++)
	{
		Tx_Buf[i]=pByte[i-32];
	}
	pByte = &DCPos4;
	for(uint8_t i = 36; i < 40; i++)
	{
		Tx_Buf[i]=pByte[i-36];
	}
	/* End: Slave 4 */
	Tx_Buf[40] = 13;

	HAL_UART_Transmit(&huart1, Tx_Buf, 41, HAL_MAX_DELAY);
}

/* -------------------------End: Configure SLAVES-----------------------------*/

/* -------------------------Begin: Testing Command----------------------------*/
float BLDCTestingSpeed = 150.23;
float DCTestingPOS = 45.01;
void commandTask(int testCommand)
{
	switch(testCommand)
	{
//		case 1:
//			ControlDriver(1, 1, BLDCTestingSpeed, DCTestingPOS, 2, 0, 0, 0, 3, 0, 0, 0, 4, 0, 0, 0);
//			break;
//		case 2:
//			ControlDriver(1, 0, 0, 0, 2, 1, BLDCTestingSpeed, DCTestingPOS, 3, 0, 0, 0, 4, 0, 0, 0);
//			break;
//		case 3:
//			ControlDriver(1, 0, 0, 0, 2, 0, 0, 0, 3, 1, BLDCTestingSpeed, DCTestingPOS, 4, 0, 0, 0);
//			break;
//		case 4:
//			ControlDriver(1, 0, 0, 0, 2, 0, 0, 0, 3, 0, 0, 0, 4, 1, BLDCTestingSpeed, DCTestingPOS);
//			break;
//		default:
//			ControlDriver(1, 0, 0, 0, 2, 0, 0, 0, 3, 0, 0, 0, 4, 0, 0, 0);
//			break;
		case 1:
			ControlDriver(1, 1, -BLDCTestingSpeed, DCTestingPOS, 2, 1, -BLDCTestingSpeed, -DCTestingPOS, 3, 0, 0, 0, 4, 0, 0, 0);
			break;
		case 2:
			ControlDriver(1, 1, BLDCTestingSpeed, -DCTestingPOS, 2, 1, -BLDCTestingSpeed, DCTestingPOS, 3, 0, 0, 0, 4, 0, 0, 0);
			break;
		case 3:
			ControlDriver(1, 1, BLDCTestingSpeed, DCTestingPOS, 2, 1, BLDCTestingSpeed, -DCTestingPOS, 3, 0, 0, 0, 4, 0, 0, 0);
			break;
		case 4:
			ControlDriver(1, 1, -BLDCTestingSpeed, -DCTestingPOS, 2, 1, BLDCTestingSpeed, DCTestingPOS, 3, 0, 0, 0, 4, 0, 0, 0);
			break;
		default:
			ControlDriver(1, 0, 0, 0, 2, 0, 0, 0, 3, 0, 0, 0, 4, 0, 0, 0);
			break;
	}
}

/* -------------------------End: Testing Command------------------------------*/


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
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  commandTask(1);
	  HAL_Delay(5000);
	  commandTask(2);
	  HAL_Delay(5000);
	  commandTask(3);
	  HAL_Delay(5000);
	  commandTask(4);
	  HAL_Delay(5000);
	  commandTask(0);
	  HAL_Delay(5000);
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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 168-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 10000-1;
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
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

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
