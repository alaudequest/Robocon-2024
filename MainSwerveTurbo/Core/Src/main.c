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
UART_HandleTypeDef huart4;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_UART4_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

uint8_t TxBuf[21];

void ControlDriver(uint8_t ID1,uint8_t MODE1,float BLDCSpeed1,float DCPos1,uint8_t ID2,uint8_t MODE2,float BLDCSpeed2,float DCPos2)
{
	uint8_t *pByte = NULL;
	pByte = &ID1;
	TxBuf[0]=pByte[0];
	pByte = &MODE1;
	TxBuf[1]=pByte[0];
	pByte = &BLDCSpeed1;
	for(uint8_t i=2;i<6;i++){
		TxBuf[i]=pByte[i-2];
	}
	pByte = &DCPos1;
	for(uint8_t i=6;i<10;i++){
		TxBuf[i]=pByte[i-6];
	}
	pByte = &ID2;
	TxBuf[10]=pByte[0];
	pByte = &MODE2;
	TxBuf[11]=pByte[0];
	pByte = &BLDCSpeed2;
	for(uint8_t i=12;i<16;i++){
		TxBuf[i]=pByte[i-12];
	}
	pByte = &DCPos2;
	for(uint8_t i=16;i<20;i++){
		TxBuf[i]=pByte[i-16];
	}
	TxBuf[20] = 13;

	HAL_UART_Transmit(&huart1, (uint8_t *)TxBuf,sizeof(TxBuf), 1000);

}


uint8_t ID1,Mode1,ID2,Mode2;
float BL1,BL2,DC1,DC2;
void SolveMainTask(uint8_t* dataArray){
	uint8_t *pFloat=NULL;
	pFloat = &ID1;
	*(pFloat)=dataArray[0];
	pFloat = &Mode1;
	*(pFloat)=dataArray[1];
	pFloat = &BL1;
	for(uint8_t i = 2; i < 6;i++)
	{
		*(pFloat+i-2)=dataArray[i];
	}
	pFloat = &DC1;
	for(uint8_t i = 6; i < 10;i++)
	{
		*(pFloat+i-6)=dataArray[i];
	}
	pFloat = &ID2;
	*(pFloat)=dataArray[10];
	pFloat = &Mode2;
	*(pFloat)=dataArray[11];
	pFloat = &BL2;
	for(uint8_t i = 12; i < 16;i++)
	{
		*(pFloat+i-12)=dataArray[i];
	}
	pFloat = &DC2;
	for(uint8_t i = 16; i < 20;i++)
	{
		*(pFloat+i-16)=dataArray[i];
	}

}

typedef struct{
	uint8_t Status;

	uint8_t XLeft;
	uint8_t YLeft;

	uint8_t XRight;
	uint8_t YRight;

	uint8_t Left;
	uint8_t Up;
	uint8_t Right;
	uint8_t Down;

	uint8_t Square;
	uint8_t Triangle;
	uint8_t Circle;
	uint8_t Cross;

	uint8_t L1;
	uint8_t L2;
	uint8_t L3;

	uint8_t R1;
	uint8_t R2;
	uint8_t R3;

	uint8_t Touch;

	uint8_t Charge;
	uint8_t Battery;
} _GamePad;

_GamePad GamePad;

uint8_t UARTRX3_Buffer[9];
uint8_t DataTayGame[9];


float Xleft,Yleft;
float Xright;

float SpeedOutPut,PosOutPut;
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){

	if(huart->Instance == USART3){
		HAL_UART_Receive_IT(&huart3, (uint8_t*)UARTRX3_Buffer, 9);
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
				if(ViTriData == 9){
					ViTriData = 0;
				}
				++cnt;
			}
			GamePad.Status = 1;

			GamePad.XLeft = DataTayGame[1];
			GamePad.YLeft = DataTayGame[2];

			GamePad.XRight = DataTayGame[3];
			GamePad.YRight = DataTayGame[4];

			GamePad.Left = (DataTayGame[5] >> 7) & 1;
			GamePad.Up = (DataTayGame[5] >> 6) & 1;
			GamePad.Right = (DataTayGame[5] >> 5) & 1;
			GamePad.Down = (DataTayGame[5] >> 4) & 1;

			GamePad.Square = (DataTayGame[5] >> 3) & 1;
			GamePad.Triangle = (DataTayGame[5] >> 2) & 1;
			GamePad.Circle = (DataTayGame[5] >> 1) & 1;
			GamePad.Cross = DataTayGame[5] & 1;

			GamePad.L1 = (DataTayGame[6] >> 7) & 1;
			GamePad.L2 = (DataTayGame[6] >> 6) & 1;
			GamePad.R1 = (DataTayGame[6] >> 5) & 1;
			GamePad.R2 = (DataTayGame[6] >> 4) & 1;

			GamePad.Touch = (DataTayGame[6] >> 3) & 1;
			GamePad.Charge = (DataTayGame[6] >> 2) & 1;

			GamePad.L3 = (DataTayGame[6] >> 1) & 1;
			GamePad.R3 = DataTayGame[6] & 1;

			GamePad.Battery = DataTayGame[7];



			  Xleft = ((GamePad.XLeft-125)/10)*0.3/12;
			  Yleft = ((GamePad.YLeft-125)/10)*0.3/12;
			  Xright =((GamePad.XRight-120)/10)*30/12;
		}
		else{
			GamePad.Status = 0;
		}
	}
}


#define robot_Lenght 0.3
#define robot_WheelR 0.09

float wheel_Vel_X1,wheel_Vel_Y1,wheel_Angle1,wheel_AngleVel1;
float wheel_Vel_X2,wheel_Vel_Y2,wheel_Angle2,wheel_AngleVel2;

int8_t quadrantCheck(float X, float Y)
{
	if ((X>0)&&(Y>=0))return 1;
	else if ((X>0)&&(Y<=0))return 4;
	else if ((X<0)&&(Y<=0))return 3;
	else if ((X<0)&&(Y>=0))return 2;
	else if ((X==0)&&(Y>0))return -1;
	else if ((X==0)&&(Y<0))return -2;
	else return 0;
}

float u,v,rad,c;

int Direc = 1;
int prevState1,prevState2;

typedef struct Optimizer{
	int Direc;
	float CurrentAngle;
	float DeltaAngle;
	float OutPutAngle;
	float PreAngle;
	float CalInput;
	float preCal;
	float DeltaCal;
	float preDir;
	float dirFlag;
	float actualPos;
}Optimizer;

void SwerveInit(Optimizer *Swerve){
	Swerve -> Direc = 1;
	Swerve -> preDir = 1;
	Swerve -> CurrentAngle = 0;
	Swerve -> DeltaAngle = 0;
	Swerve -> OutPutAngle = 0;
	//Swerve -> PreAngle =0;
	Swerve -> preCal =0;
	Swerve -> DeltaCal = 0;
	Swerve -> dirFlag = 0;
	Swerve->CalInput = 0;
	Swerve-> actualPos=0;
}


float modulo360(float Angle)
{
	int Result = (int)Angle/360;
	return Angle-Result*360;
}

int signum(int num){
	if (num>0)
		return 1;
	else if(num<0)
		return -1;
}
float TestAngle;
Optimizer Swerve1;
Optimizer Swerve2;
void OptimizeAngle (Optimizer *Swerve,float InPut)
{
	if (InPut != Swerve->PreAngle){

//		if (Swerve->CurrentAngle>180)Swerve->CurrentAngle-=360;
//		else if (Swerve->CurrentAngle<-180)Swerve->CurrentAngle+=360;
//
//		if (Swerve->CurrentAngle>180)Swerve->CurrentAngle-=360;
//		else if (Swerve->CurrentAngle<-180)Swerve->CurrentAngle+=360;
//
//
//		Swerve->CalInput = modulo360(Swerve->CurrentAngle);
//		if (Swerve->CurrentAngle>180)Swerve->CalInput-=360;
//		else if (Swerve->CurrentAngle<-180)Swerve->CalInput+=360;
//
//		Swerve->DeltaAngle = InPut-(Swerve->CalInput);
//
//		if (Swerve->DeltaAngle >180)
//			{
//				Swerve->DeltaAngle-=360;
//			}
//		else if (Swerve->DeltaAngle <-180)
//		{
//			Swerve->DeltaAngle+=360;
//
//		}

//		if((Swerve->DeltaAngle<=90)&&(Swerve->DeltaAngle>=-90))
//			{
//				Swerve->DeltaAngle=Swerve->DeltaAngle;
//			}
//		else if ((Swerve->DeltaAngle>90)&&(Swerve->DeltaAngle<=180))
//		{
//			Swerve->DeltaAngle += -180;
//			Swerve->Direc *=-1;
//		}
//		else if ((Swerve->DeltaAngle<-90)&&(Swerve->DeltaAngle>=-180))
//		{
//			Swerve->DeltaAngle += 180;
//			Swerve->Direc *=-1;
//		}
//
//		Swerve->OutPutAngle += Swerve->DeltaAngle;
//		Swerve->CurrentAngle += Swerve->DeltaAngle;
//

//		Swerve->CalInput = modulo360(Swerve->CurrentAngle);
//		if (Swerve->CalInput>180)Swerve->CalInput-=360;
//		else if (Swerve->CalInput<-180)Swerve->CalInput+=360;
//
//		Swerve->DeltaAngle = InPut - Swerve->CalInput;
//
//		if(abs(Swerve->DeltaAngle)>180)
//		{
//			if(abs(Swerve->DeltaAngle)>=270)Swerve->Direc*=1;
//			else Swerve->Direc*=1;
//
//
//			Swerve->DeltaAngle = -(signum(Swerve->DeltaAngle)*360)+Swerve->DeltaAngle;
//		}
//		Swerve->PreAngle = InPut;
//		Swerve->CurrentAngle+= Swerve->DeltaAngle;


		Swerve->CalInput = InPut;
		if ((Swerve->CurrentAngle>=0)&&Swerve->CalInput<0)Swerve->CalInput+=360;
		else if ((Swerve->CurrentAngle<0)&&Swerve->CalInput>0) Swerve->CalInput-=360;

		Swerve->DeltaCal = Swerve->CalInput-Swerve->preCal;
		if(Swerve->DeltaCal>180)Swerve->DeltaCal+=-360;
		else if(Swerve->DeltaCal<-180)Swerve->DeltaCal+=360;


		if ((Swerve->DeltaCal>90)&&(Swerve->DeltaCal<180))
		{
			//Swerve->DeltaAngle += -180;
			Swerve->Direc *= -1;
			Swerve->dirFlag = 1;
		}else if ((Swerve->DeltaCal<-90)&&(Swerve->DeltaCal>-180))
		{
			//Swerve->DeltaAngle += 180;
			Swerve->Direc *= -1;
			Swerve->dirFlag = 1;
		}else if(abs(Swerve->DeltaCal)==180)
		{
			Swerve->Direc *= -1;
			Swerve->dirFlag = -1;
		}else {
			Swerve->dirFlag = 1;
		}

//		if((abs(Swerve->CalInput-Swerve->preCal)>=270)||(abs(Swerve->CalInput-Swerve->preCal)<180))Swerve->Direc *=1;
//		else Swerve->Direc *=-1;
		Swerve->DeltaAngle = Swerve->CalInput-modulo360(Swerve->CurrentAngle);

		if(Swerve->DeltaAngle>180)Swerve->DeltaAngle+=-360;
		else if(Swerve->DeltaAngle<-180)Swerve->DeltaAngle+=360;

		if((Swerve->DeltaAngle<=90)&&(Swerve->DeltaAngle>=-90))
		{
			if(Swerve->dirFlag==1)
				Swerve->DeltaAngle = Swerve->DeltaAngle;
			else Swerve->DeltaAngle = Swerve->DeltaAngle;
//			Swerve->Direc *=1;
		}else if ((Swerve->DeltaAngle>90)&&(Swerve->DeltaAngle<=180))
		{
			Swerve->DeltaAngle += -180;
//			Swerve->dirFlag = 0;
//			Swerve->Direc *= -1;
		}else if ((Swerve->DeltaAngle<-90)&&(Swerve->DeltaAngle>=-180))
		{
			Swerve->DeltaAngle += 180;
//			Swerve->dirFlag = 0;
//			Swerve->Direc *= -1;
		}

		Swerve->PreAngle = InPut;
		Swerve->preCal = Swerve->CalInput;
		Swerve->CurrentAngle+= Swerve->DeltaAngle;


		if(Swerve->CalInput==modulo360(Swerve->CurrentAngle)){
			Swerve->Direc = 1;
		}
//		Swerve->preDir=Swerve->Direc;
//		if(Swerve->CurrentAngle>0)
//		{
//			if(Swerve->Direc*modulo360(Swerve->CurrentAngle)<0){
//				Swerve->actualPos = (Swerve->Direc*Swerve->CurrentAngle)+360;
//			}
//				if(Swerve->actualPos!=Swerve->CalInput)Swerve->Direc*=-1;
//		}else{
//			if(Swerve->Direc*Swerve->CurrentAngle>0){
//				Swerve->actualPos = (Swerve->Direc*modulo360(Swerve->CurrentAngle))-360;
//			}
//		if(Swerve->actualPos!=Swerve->CalInput)Swerve->Direc*=-1;
//		}

		//Từ ma trận động học nghịch vận tốc ta suy được
	}
}
float showAngle;
int preQuad1,currQuad1;
int preQuad2,currQuad2;
void InverseKine(float u,float v ,float r)
{

	wheel_Vel_X1 = v;
	wheel_Vel_Y1 = u-robot_Lenght*r;

	wheel_Vel_X2 = v;
	wheel_Vel_Y2 = u+robot_Lenght*r;

	currQuad1 = quadrantCheck(wheel_Vel_X1,wheel_Vel_Y1);
	currQuad2 = quadrantCheck(wheel_Vel_X2,wheel_Vel_Y2);
	if(currQuad1 == 0)
		{
			OptimizeAngle(&Swerve1, preQuad1);
		}
	else{
		OptimizeAngle(&Swerve1, atan2(wheel_Vel_Y1,wheel_Vel_X1)*180/M_PI);
		preQuad1 = atan2(wheel_Vel_Y1,wheel_Vel_X1)*180/M_PI;
	}
	wheel_AngleVel1 = Swerve1.Direc*(1/robot_WheelR)*(sqrt(pow(wheel_Vel_X1,2)+pow(wheel_Vel_Y1,2)))/ 0.1047198;
	//OptimizeAngle(&Swerve1, TestAngle);

	if(currQuad2 == 0)
	{
		OptimizeAngle(&Swerve2, preQuad2);
	}
	else{
		OptimizeAngle(&Swerve2, atan2(wheel_Vel_Y2,wheel_Vel_X2)*180/M_PI);
		preQuad2 = atan2(wheel_Vel_Y2,wheel_Vel_X2)*180/M_PI;
	}

	wheel_AngleVel2 = -Swerve2.Direc*(1/robot_WheelR)*(sqrt(pow(wheel_Vel_X2,2)+pow(wheel_Vel_Y2,2)))/0.1047198;



}

/* USER CODE END 0 */
#define Speed 20

int Angle;
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
  MX_UART4_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
//  ControlDriver(1,10,11);
//  ControlDriver(1,1,100,90,2,1,-100,95);
//  SolveMainTask(TxBuf);
  SwerveInit(&Swerve1);
  SwerveInit(&Swerve2);
  HAL_UART_Receive_IT(&huart3, (uint8_t*)UARTRX3_Buffer, 9);
  HAL_Delay(1000);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  //ControlDriver(1,1,SpeedOutPut,PosOutPut,2,1,0-SpeedOutPut,PosOutPut);

	  HAL_Delay(100);

	  if (GamePad.Touch == 1)
	  {
		  ControlDriver(1,2,0,0,2,2,0,0);
	  }else if (GamePad.R2 == 1){
		  SwerveInit(&Swerve1);
		  SwerveInit(&Swerve2);
	  }
	  else {
	  rad = Angle*M_PI/180;
	  	  InverseKine(Yleft,-Xleft,-Xright*M_PI/180);
	  	  ControlDriver(1,1,wheel_AngleVel1,Swerve1.CurrentAngle,2,1,wheel_AngleVel2,Swerve2.CurrentAngle);

	  }





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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin : PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

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
