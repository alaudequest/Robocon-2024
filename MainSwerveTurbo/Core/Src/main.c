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
#include <PID.h>
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

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
/*-----------------------------Begin:kinematic Macro--------------------------*/
#define robot_Lenght 0.3
#define robot_WheelR 0.09
/*-----------------------------End:kinematic Macro----------------------------*/

/*-----------------------------Begin:Optimizer Var----------------------------*/

/*-----------------------------End:Optimizer Var------------------------------*/
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_UART4_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
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

char ds[12];
uint8_t uart2_ds, ds_ind, ds_cnt, ds_flg;
int GocRobot;

void GetDataCompass(){
	GocRobot = ds[1] - 48;
	int x = 2;
	while((ds[x] >= 48) && (ds[x] <= 57)){
		GocRobot = GocRobot * 10;
		GocRobot += ds[x] -48;
		++x;
	}

	if(ds[0] == '-'){
		GocRobot = -GocRobot;
	}
}

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
	if(huart->Instance == USART2){
		if(uart2_ds != '\n')
				ds[ds_ind++] = uart2_ds;
		else{
				GetDataCompass();
				ds_cnt = ds_ind;
				ds_flg = 1;
				ds_ind = 0;
		}
		HAL_UART_Receive_IT(&huart2, &uart2_ds, 1);
	}
}




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
	uint8_t OdorFlag;
}Optimizer;

void SwerveInit(Optimizer *Swerve){
	Swerve -> Direc = 1;
	Swerve -> CurrentAngle = 0;
	Swerve -> DeltaAngle = 0;
	Swerve -> OutPutAngle = 0;
	Swerve -> PreAngle =0;
	Swerve -> preCal =0;
	Swerve -> DeltaCal = 0;
	Swerve->  CalInput = 0;
	Swerve-> OdorFlag = 1;
}


float modulo360(float Angle)
{
	int Result = (int)Angle/360;
	return Angle-Result*360;
}
float modulo180(float Angle)
{
	int Result = (int)Angle/180;
	return Angle-Result*180;
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

int OdoPos,OdoFlag1,OdoFlag2,OdorCnt;;
double CurrPos;

void OptimizeAngle (Optimizer *Swerve,float InPut)
{
	if (InPut != Swerve->PreAngle){

		Swerve->CalInput = InPut;
		if ((Swerve->CurrentAngle>=0)&&Swerve->CalInput<0)Swerve->CalInput+=360;
		else if ((Swerve->CurrentAngle<0)&&Swerve->CalInput>0) Swerve->CalInput-=360;

		Swerve->DeltaCal = Swerve->CalInput-Swerve->preCal;
		if(Swerve->DeltaCal>180)Swerve->DeltaCal+=-360;
		else if(Swerve->DeltaCal<-180)Swerve->DeltaCal+=360;


		if ((Swerve->DeltaCal>90)&&(Swerve->DeltaCal<180))
		{
			Swerve->Direc *= -1;
		}else if ((Swerve->DeltaCal<-90)&&(Swerve->DeltaCal>-180))
		{
			Swerve->Direc *= -1;
		}else if(abs(Swerve->DeltaCal)==180)
		{
			Swerve->Direc *= -1;
		}

		Swerve->DeltaAngle = Swerve->CalInput-modulo360(Swerve->CurrentAngle);

		if(Swerve->DeltaAngle>180)Swerve->DeltaAngle+=-360;
		else if(Swerve->DeltaAngle<-180)Swerve->DeltaAngle+=360;

		if((Swerve->DeltaAngle<=90)&&(Swerve->DeltaAngle>=-90))
		{
			Swerve->DeltaAngle = Swerve->DeltaAngle;
		}else if ((Swerve->DeltaAngle>90)&&(Swerve->DeltaAngle<=180))
		{
			Swerve->DeltaAngle += -180;
//			if (modulo360(abs(Swerve->CurrentAngle)>90)){
//				Swerve->OdorFlag ^= 1;
//			}
		}else if ((Swerve->DeltaAngle<-90)&&(Swerve->DeltaAngle>=-180))
		{
			Swerve->DeltaAngle += 180;
//			if (modulo360(abs(Swerve->CurrentAngle)>90)&&(Swerve->)){
//				Swerve->OdorFlag ^= 1;
//			}
		}

		Swerve->PreAngle = InPut;
		Swerve->preCal = Swerve->CalInput;
		Swerve->CurrentAngle+= Swerve->DeltaAngle;


		if(Swerve->CalInput==modulo360(Swerve->CurrentAngle)){
			Swerve->Direc = 1;
		}
	}
}
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

	if(currQuad2 == 0)
	{
		OptimizeAngle(&Swerve2, preQuad2);
	}
	else{
		OptimizeAngle(&Swerve2, atan2(wheel_Vel_Y2,wheel_Vel_X2)*180/M_PI);
		preQuad2 = atan2(wheel_Vel_Y2,wheel_Vel_X2)*180/M_PI;
	}

	wheel_AngleVel2 = -Swerve2.Direc*(1/robot_WheelR)*(sqrt(pow(wheel_Vel_X2,2)+pow(wheel_Vel_Y2,2)))/0.1047198;

	if ((v>0)&&(Swerve1.Direc>0))Swerve1.OdorFlag = 0;
	else if ((v<0)&&(Swerve1.Direc>0))Swerve1.OdorFlag = 1;
	else if ((v<0)&&(Swerve1.Direc<0))Swerve1.OdorFlag = 0;
	else if ((v>0)&&(Swerve1.Direc<0))Swerve1.OdorFlag = 1;
	else {
		OdorCnt = 0;
		Swerve1.OdorFlag = 2;
	}
}
Optimizer Base;
PID_Param PIDAngle;
PID_Param PIDOdoU;
PID_Param PIDOdoV;

double h,xnow,ynow,preh;
void PositionCal(float x,float y,float Count)
{
	h = sqrt(pow(x,2)+pow(y,2));
	xnow = sqrt(pow(Count,2)-pow((y)*(Count/h),2));
	ynow = sqrt(pow(Count,2)-pow((x)*(Count/h),2));
}



#define P_Term 0
#define I_Term 0
#define D_Term 0
#define DeltaT 0.01
#define D_alpha 0
#define I_Above 40
#define I_Below -40
#define U_Above 40
#define U_Below -40


double kP=1.5,kI,kD=0.1,alpha=0.8,T = 0.05,Ia,Ib,Ua = 40,Ub =-40;
double kP1=0,kI1,kD1=0,alpha1=0,T1 = 0.05,Ia1,Ib1,Ua1 = 0.05,Ub1 =-0.05;
double kP2=1.5,kI2,kD2=0.1,alpha2=0.8,T2 =0.05,Ia2,Ib2,Ua2 = 0.1,Ub2 =-0.1;
double TargetAngle = 0;
int zamn = 0;


double x,y;
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (GPIO_Pin == Odo_Extio_Pin)
	{
		if(HAL_GPIO_ReadPin(Odo_Input_GPIO_Port, Odo_Input_Pin)==0)OdorCnt++;
		else OdorCnt--;
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
  MX_USART1_UART_Init();
  MX_UART4_Init();
  MX_USART3_UART_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  SwerveInit(&Swerve1);
  SwerveInit(&Swerve2);
  HAL_UART_Receive_IT(&huart3, (uint8_t*)UARTRX3_Buffer, 9);
  HAL_TIM_Base_Start_IT(&htim2);
  HAL_Delay(5000);
  HAL_UART_Receive_IT(&huart2, &uart2_ds, 1);
//  TargetAngle=GocRobot;


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
//	  ControlDriver(1,1,SpeedOutPut,PosOutPut,2,1,0-SpeedOutPut,PosOutPut);
	  HAL_Delay(T*1000);
	  if (GamePad.Touch == 1)
	  {
		  ControlDriver(1,2,0,Swerve1.CurrentAngle,2,2,0,Swerve2.CurrentAngle);
	  }
	  else {
//	  	if(zamn == 1){
//	  		  Pid_Cal(&PIDAngle,TargetAngle,GocRobot);
//	  		  Pid_SetParam(&PIDAngle,kP,kI,kD,alpha,T,Ia,Ib,Ua,Ub);
//	  		  InverseKine(Yleft,-Xleft,-PIDAngle.u*M_PI/180);
//	  		  ControlDriver(1,1,wheel_AngleVel1,Swerve1.CurrentAngle,2,1,wheel_AngleVel2,Swerve2.CurrentAngle);
//	  	}

		 if(GamePad.Square == 1){
			 OdoFlag1 = 1;
			 OdoFlag2 = 0;
		 }else if(GamePad.Circle == 1)
		 {
			 OdoFlag2 = 1;
			 OdoFlag1 = 0;
		 }
	  	 }
	  if (OdoFlag1 == 1){

		if(Swerve1.OdorFlag == 0)
		{
			OdoPos-= OdorCnt;
		}else {
			OdoPos+= OdorCnt;
		}

		OdorCnt = 0;

//		  Pid_SetParam(&PIDAngle,kP,kI,kD,alpha,T,Ia,Ib,Ua,Ub);
		  Pid_SetParam(&PIDOdoU,kP1,kI1,kD1,alpha1,T1,Ia1,Ib1,Ua1,Ub1);
//		  Pid_SetParam(&PIDOdoV,kP2,kI2,kD2,alpha2,T2,Ia2,Ib2,Ua2,Ub2);
		  CurrPos = (0.07*M_PI/1000)*OdoPos;
//		  PositionCal(x,y,OdoCount);
		  Pid_Cal(&PIDOdoU,x,CurrPos);
//		  Pid_Cal(&PIDOdoV,y,ynow);
//		  Pid_Cal(&PIDAngle,TargetAngle,GocRobot);

		  InverseKine(0,-PIDOdoU.u,0);
				  //-PIDAngle.u*M_PI/180);
		  ControlDriver(1,1,wheel_AngleVel1,Swerve1.CurrentAngle,2,1,wheel_AngleVel2,Swerve2.CurrentAngle);
	  }
////	  else if (GamePad.R2 == 1){
////		  SwerveInit(&Swerve1);
////		  SwerveInit(&Swerve2);
////	  }
//	  else {
//	  	 InverseKine(Yleft,-Xleft,-Xright*M_PI/180);
//	  	 ControlDriver(1,1,wheel_AngleVel1,Swerve1.CurrentAngle,2,1,wheel_AngleVel2,Swerve2.CurrentAngle);
//	  }

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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin : Odo_Extio_Pin */
  GPIO_InitStruct.Pin = Odo_Extio_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(Odo_Extio_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Odo_Input_Pin */
  GPIO_InitStruct.Pin = Odo_Input_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(Odo_Input_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

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
