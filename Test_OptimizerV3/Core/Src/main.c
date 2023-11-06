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
#include "math.h"
#include "stdlib.h"
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

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
float AngleTest;

typedef struct Optimizer {
	int8_t Direc;
	float CurAngle;
	float PreAngle;
	float DeltaAngle;
	float CalInput;
	float CalX;
	float CalY;
	float PreX;
	float PreY;
	float Alpha;
	int8_t X;
	int8_t Y;
}Optimizer;

Optimizer Swerve;

void SwerveInit(Optimizer *Swerve) {
	Swerve->Direc 		= 1;
	Swerve->CurAngle 	= 0;
	Swerve->PreAngle	= 0;
	Swerve->DeltaAngle	= 0;
	Swerve->CalInput 	= 0;
	Swerve->CalX		= 0;
	Swerve->CalY		= 0;
	Swerve->PreX		= 1;
	Swerve->PreY		= 0;
	Swerve->Alpha 		= 0;
	Swerve->X			= 1;
	Swerve->Y			= 0;
}

void OptimizerV3(Optimizer *Swerve, float Input){
	if(Input != Swerve->PreAngle) {
		Swerve->CalInput   = Input*M_PI/180;
		Swerve->CalX 	   = Swerve->X*cos(Swerve->CalInput) - Swerve->Y*sin(Swerve->CalInput);
		Swerve->CalY 	   = Swerve->X*sin(Swerve->CalInput) + Swerve->Y*cos(Swerve->CalInput);
		Swerve->Alpha 	   = acos(((Swerve->PreX*Swerve->CalX)+(Swerve->PreY*Swerve->CalY))/(sqrt((Swerve->CalX*Swerve->CalX)+(Swerve->CalY*Swerve->CalY))*sqrt((Swerve->PreX*Swerve->PreX)+(Swerve->PreY*Swerve->PreY))))*(180/M_PI);
		Swerve->PreX 	   = Swerve->CalX;
		Swerve->PreY 	   = Swerve->CalY;
		Swerve->PreAngle   = Input;
		Swerve->DeltaAngle = abs(Swerve->CurAngle-Swerve->PreAngle);
//		if(Swerve->CurAngle>=(360*3)){
			if(Swerve->Alpha == 0){
				if(Input == (Swerve->CurAngle+360)) Swerve->CurAngle=Input-360;
				else if(Input==(Swerve->CurAngle-360))Swerve->CurAngle=Input+360;
			}
			else if(Swerve->Alpha==180){
				Swerve->Direc *= -1;
				if(Input==(Swerve->CurAngle+180)) Swerve->CurAngle=Input-180;
				else if(Input==(Swerve->CurAngle-180)) Swerve->CurAngle=Input+180;
			}
			else if(Swerve->Alpha<=90) {
				if(Swerve->DeltaAngle<=90){
					Swerve->Direc = 1;
					Swerve->CurAngle=Input;
				}
				else if(Swerve->DeltaAngle<=180 && Swerve->CurAngle>=0){
					Swerve->Direc=-1;
					if(Input>0) Swerve->CurAngle=Input-180;
					else Swerve->CurAngle=Input+180;
				}
				else if(Swerve->DeltaAngle<=180 && Swerve->CurAngle<=0){
					Swerve->Direc=-1;
					if(Input>=0) Swerve->CurAngle=Input-180;
					else Swerve->CurAngle=Input+180;
				}
				else if(Swerve->DeltaAngle<=270 && Swerve->CurAngle>=0){
					Swerve->Direc=-1;
					if(Input>0) Swerve->CurAngle=Input-180;
					else Swerve->CurAngle=Input+180;
				}
				else if(Swerve->DeltaAngle<=270 && Swerve->CurAngle<=0){
					Swerve->Direc=-1;
					if(Input>=0) Swerve->CurAngle=Input-180;
					else Swerve->CurAngle=Input+180;
				}
				else if(Swerve->DeltaAngle>270){
					Swerve->Direc=-1;
					if(Input>=0) Swerve->CurAngle=Input-180*3;
					else Swerve->CurAngle=(180*3+Input);
				}
			}
			else if(Swerve->Alpha>=90) {
				if(Swerve->DeltaAngle<=90) {
					Swerve->CurAngle= Input;
					Swerve->Direc = 1;
				}
				else if(Swerve->CurAngle>=0 && Swerve->DeltaAngle<=180) {
					if(Input>0)Swerve->CurAngle=Input-180;
					else Swerve->CurAngle=Input+180;
					Swerve->Direc = -1;
				}
				else if(Swerve->CurAngle<=0 && Swerve->DeltaAngle<=180) {
					if(Input>=0)Swerve->CurAngle=Input-180;
					else Swerve->CurAngle=Input+180;
					Swerve->Direc = -1;
				}

				else if(Swerve->CurAngle<=0 && Swerve->DeltaAngle<=270) {
					if(Input>=0)Swerve->CurAngle=Input-180;
					else Swerve->CurAngle=Input+180;
					Swerve->Direc = -1;
				}
				else if(Swerve->CurAngle>=0 && Swerve->DeltaAngle<=270) {
					if(Input>=0)Swerve->CurAngle=180-Input;
					else Swerve->CurAngle=Input+180;
					Swerve->Direc = -1;
				}
				else if(Swerve->CurAngle>=0 && Swerve->DeltaAngle>270) {
					if(Input>=0)Swerve->CurAngle=Input-360;
					else Swerve->CurAngle=Input+360;
					Swerve->Direc=1;
				}
				else if(Swerve->CurAngle<=0 && Swerve->DeltaAngle>270) {
					if(Input>=0)Swerve->CurAngle=Input-360;
					else Swerve->CurAngle=Input+360;
					Swerve->Direc=1;
				}
			}
//		}
//		else{
//			Swerve->Direc=1;
//			Swerve->CurAngle=Input;
//		}
	}
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
  /* USER CODE BEGIN 2 */
  SwerveInit(&Swerve);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	OptimizerV3(&Swerve, AngleTest);
	HAL_Delay(10);
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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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
