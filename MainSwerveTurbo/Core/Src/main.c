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
#include <math.h>
#include <stdlib.h>
#include <PID.h>
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

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart2_rx;

osThreadId TaskPathTrackHandle;
osThreadId TaskPIDCalHandle;
osThreadId KinematicTaskHandle;
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
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_UART4_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
void PathTracking(void const * argument);
void StartTask02(void const * argument);
void KinematicCal(void const * argument);

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
uint8_t uart2_ds[5], ds_ind, ds_cnt, ds_flg;
//int GocRobot;
//
//void GetDataCompass(){
//	GocRobot = ds[1] - 48;
//	int x = 2;
//	while((ds[x] >= 48) && (ds[x] <= 57)){
//		GocRobot = GocRobot * 10;
//		GocRobot += ds[x] -48;
//		++x;
//	}
//
//	if(ds[0] == '-'){
//		GocRobot = -GocRobot;
//	}
//}

uint8_t AngleData[5];
int CurrAngle;
void Receive(uint8_t *DataArray){
      uint8_t *pInt = NULL;
      if(DataArray[4] == 13){
           pInt = &CurrAngle;
           for(uint8_t i = 0; i < 4; i++) {
               *(pInt + i) = DataArray[i];
            }
      }
      	  memset(DataArray,0,5);
 }

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	if(huart -> Instance == USART2)
	{
		HAL_UARTEx_ReceiveToIdle_DMA(&huart2, uart2_ds, 5);
			Receive(uart2_ds);
		  __HAL_DMA_DISABLE_IT(&hdma_usart2_rx,DMA_IT_HT);
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
			  Xright =0.785/33*((GamePad.XRight-120)/10)*30/12;
		}
		else{
			GamePad.Status = 0;
		}
	}
//	if(huart->Instance == USART2){
////		if(uart2_ds != '\n')
////				ds[ds_ind++] = uart2_ds;
////		else{
////				GetDataCompass();
////				ds_cnt = ds_ind;
////				ds_flg = 1;
////				ds_ind = 0;
////		}
//		HAL_UART_Receive_IT(&huart2, (uint8_t*)uart2_ds, 5);
//		int Vitridata2 = -1;
//		for (int i2 =0;i2<=4;++i2)
//		{
//			if(uart2_ds[i2]==149){
//				Vitridata2 = i2;
//			}
//		}
//
//		if(Vitridata2 != -1){
//			int cnt2 = 0;
//			while (cnt2<5){
//				AngleData[cnt2] = AngleData[Vitridata2];
//				++Vitridata2;
//				if(Vitridata2 == 5)
//				{
//					Vitridata2 = 0;
//				}
//				++cnt2;
//			}
//			CurrAngle = AngleData[2]<<8 | AngleData[3];
//		}
//
//	}
}

//MMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMM//
//----------------------------------------Begin:Optimizer--------------------------------------//


float wheel_Vel_X1,wheel_Vel_Y1,wheel_Angle1,wheel_AngleVel1;
float wheel_Vel_X2,wheel_Vel_Y2,wheel_Angle2,wheel_AngleVel2;

float u,v,rad,c;
int Direc = 1;

float TestAngle;



//De Tim Ra 4 goc phan tu + cac Vecto giua cac goc phan tu
int8_t quadrantCheck(float X, float Y)
{
	if((X>0)&&(Y>0))return 1;
	else if ((X<0)&&(Y>0))return 2;
	else if ((X<0)&&(Y<0))return 3;
	else if ((X>0)&&(Y<0))return 4;
	else if ((X>0)&&(Y==0))return -1;
	else if ((X<0)&&(Y==0))return -2;
	else if ((X==0)&&(Y>0))return -3;
	else if ((X==0)&&(Y<0))return -4;
	else return 0;
}



//Struct giu chi so cua Ham toi uu goc xoay
typedef struct Optimizer{
	int Direc;
	float CurrentAngle;
	float DeltaAngle;
	float OutPutAngle;
	float PreAngle;
	float CalInput;
	float preCal;
	float DeltaCal;
	uint8_t OdoFlagX;
	uint8_t OdoFlagY;
	double Xcoef,Ycoef;
	double XCurrPos;
	double YCurrPos;
}Optimizer;



//Khoi tao chi so cho moi cum Swerve
void SwerveInit(Optimizer *Swerve){
	Swerve -> Direc = 1;
	Swerve -> CurrentAngle = 0;
	Swerve -> DeltaAngle = 0;
	Swerve -> OutPutAngle = 0;
	Swerve -> PreAngle =0;
	Swerve -> preCal =0;
	Swerve -> DeltaCal = 0;
	Swerve->  CalInput = 0;
	Swerve-> OdoFlagX = 1;
	Swerve-> OdoFlagY = 1;
	Swerve-> Xcoef = 0;
	Swerve-> Ycoef = 0;
}



//Ham tinh toan chia het cho 360
float modulo360(float Angle)
{
	int Result = (int)Angle/360;
	return Angle-Result*360;
}



//Ham tinh toan chia het cho 180
float modulo180(float Angle)
{
	int Result = (int)Angle/180;
	return Angle-Result*180;
}



//Ham Tim Goc Xoay Toi Uu nhat cho cum Swerve
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
			Swerve->DeltaAngle += -180.0;
		}else if ((Swerve->DeltaAngle<-90)&&(Swerve->DeltaAngle>=-180))
		{
			Swerve->DeltaAngle += 180.0;
		}

		Swerve->PreAngle = InPut;
		Swerve->preCal = Swerve->CalInput;
		Swerve->CurrentAngle+= Swerve->DeltaAngle;


		if(Swerve->CalInput==modulo360(Swerve->CurrentAngle)){
			Swerve->Direc = 1;
		}
	}
}


//----------------------------------------End:Optimizer----------------------------------------//
//MMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMM//


float H ;
float Lenght = 58;
float Width = 0;
float Gamma ;
float gamma1,gamma2 ;
float gamma01 = 0*M_PI/180,gamma02 = 180*M_PI/180;

float Rcp,Phicl = 0.785;
float R1,R2;
float Phi1,Phi2;
float velFactor1,velFactor2;

double max(double a,double b)
{
	double max;
	max = a;
	if (b>=max)
	{
		max = b;
	}
	return max;
}
//
//float uSnake = 0,v = 1;

void SnakeTurn(double u,double v)
{
	if(Phicl!=0){
		 H = sqrt(pow(Lenght,2)+pow(Width,2))/2;
		    Gamma = atan2(u,v) ;
			gamma1 = Gamma + gamma01;
			gamma2 = Gamma + gamma02;


			Rcp = H/tan(Phicl);

			R1 = sqrt(pow(H,2)*pow(cos(gamma1),2)+pow((Rcp-H*sin(gamma1)),2));
			R2 = sqrt(pow(H,2)*pow(cos(gamma2),2)+pow((Rcp-H*sin(gamma2)),2));

			Phi1 = ((Phicl)*asin(H*cos(gamma1)/R1));
			Phi2 = ((Phicl)*asin(H*cos(gamma2)/R2));

			velFactor1 = R1/max(R1, R2);
			velFactor2 = R2/max(R1, R2);
	}else{
		Phi1 = 0;
		Phi2 = 0;
		velFactor1 = 1;
		velFactor2 = 1;
	}

}



//MMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMM//
//----------------------------------------Begin:kinematic--------------------------------------//
int preQuad1,currQuad1;
int preQuad2,currQuad2;
int OdoPos,OdoFlag1,OdoFlag2,OdorCnt;

Optimizer Swerve1;
Optimizer Swerve2;

//Ham tinh dong hoc nghich cho robot dua vao van toc chuyen vi theo cac phuong
void InverseKine(float u,float v ,float r)
{
	Phicl = r;
	wheel_Vel_X1 = v;
	wheel_Vel_Y1 = u;

	SnakeTurn(u,v);
//			-robot_Lenght*r;

	wheel_Vel_X2 = v;
	wheel_Vel_Y2 = u;
//			+robot_Lenght*r;

	currQuad1 = quadrantCheck(wheel_Vel_X1,wheel_Vel_Y1);
	currQuad2 = quadrantCheck(wheel_Vel_X2,wheel_Vel_Y2);



	if(currQuad1 == 0)
		{
			OptimizeAngle(&Swerve1, preQuad1);
		}
	else{
		if ((Phicl == 0)){
			OptimizeAngle(&Swerve1, (atan2(wheel_Vel_Y1,wheel_Vel_X1))*180/M_PI);
			preQuad1 = (atan2(wheel_Vel_Y1,wheel_Vel_X1))*180/M_PI;
		}else {
			if(v==0){
				OptimizeAngle(&Swerve1, (atan2(wheel_Vel_Y1,wheel_Vel_X1))*180/M_PI);
				preQuad1 = (atan2(wheel_Vel_Y1,wheel_Vel_X1))*180/M_PI;
			}else {
				OptimizeAngle(&Swerve1, Phi1*180/M_PI);
				preQuad1 = Phi1*180/M_PI;
			}
		}
		}
	wheel_AngleVel1 = velFactor1*Swerve1.Direc*(1/robot_WheelR)*(sqrt(pow(wheel_Vel_X1,2)+pow(wheel_Vel_Y1,2)))/ 0.1047198;

	if(currQuad2 == 0)
	{
		OptimizeAngle(&Swerve2, preQuad2);
	}
	else{
		if((Phicl == 0)){
			OptimizeAngle(&Swerve2, (atan2(wheel_Vel_Y2,wheel_Vel_X2))*180/M_PI);
			preQuad2 = (atan2(wheel_Vel_Y2,wheel_Vel_X2))*180/M_PI;
		}else{
			if(v == 0){
				OptimizeAngle(&Swerve2, (atan2(wheel_Vel_Y2,wheel_Vel_X2))*180/M_PI);
				preQuad2 = (atan2(wheel_Vel_Y2,wheel_Vel_X2))*180/M_PI;
			}
			else{
				OptimizeAngle(&Swerve2, Phi2*180/M_PI);
				preQuad2 = Phi2*180/M_PI;
			}
		}
	}

	wheel_AngleVel2 = -velFactor2*Swerve2.Direc*(1/robot_WheelR)*(sqrt(pow(wheel_Vel_X2,2)+pow(wheel_Vel_Y2,2)))/0.1047198;


	//Góc phần tư đầu tiên :
	if (currQuad1 == 1){
		if(Swerve1.Direc>0)
			{
				Swerve1.OdoFlagX = 0;
				Swerve1.OdoFlagY = 0;
			}
		else if (Swerve1.Direc<0)
			{
				Swerve1.OdoFlagX = 1;
				Swerve1.OdoFlagY = 1;
			}
		Swerve1.Xcoef =  cos(atan(wheel_Vel_Y1/wheel_Vel_X1));
		Swerve1.Ycoef =  sin(atan(wheel_Vel_Y1/wheel_Vel_X1));
	//Goc phan tu thu 2 :
	}
	else if (currQuad1 == 2){
		if(Swerve1.Direc>0)
			{
				Swerve1.OdoFlagX = 1;
				Swerve1.OdoFlagY = 0;
			}
		else if (Swerve1.Direc<0)
			{
				Swerve1.OdoFlagX = 0;
				Swerve1.OdoFlagY = 1;
			}
		Swerve1.Xcoef =  cos(atan(wheel_Vel_Y1/wheel_Vel_X1));
		Swerve1.Ycoef =  sin(atan(wheel_Vel_Y1/wheel_Vel_X1));
	}
	//Goc phan tu thu 3 :
	else if (currQuad1 == 3)
	{
		if(Swerve1.Direc>0)
			{
				Swerve1.OdoFlagX = 1;
				Swerve1.OdoFlagY = 1;
			}
		else if (Swerve1.Direc<0)
			{
				Swerve1.OdoFlagX = 0;
				Swerve1.OdoFlagY = 0;
			}
		Swerve1.Xcoef =  cos(atan(wheel_Vel_Y1/wheel_Vel_X1));
		Swerve1.Ycoef =  sin(atan(wheel_Vel_Y1/wheel_Vel_X1));
	}
	//Goc phan tu thu 4 :
	else if (currQuad1 == 4){
		if(Swerve1.Direc>0)
			{
				Swerve1.OdoFlagX = 0;
				Swerve1.OdoFlagY = 1;
			}
		else if (Swerve1.Direc<0)
			{
				Swerve1.OdoFlagX = 1;
				Swerve1.OdoFlagY = 0;
			}
		Swerve1.Xcoef =  cos(atan(wheel_Vel_Y1/wheel_Vel_X1));
		Swerve1.Ycoef =  sin(atan(wheel_Vel_Y1/wheel_Vel_X1));
	}
	//Vector chi huong 0 do :
	else if (currQuad1 == -1)
	{
		if(Swerve1.Direc>0)
			{
				Swerve1.OdoFlagX = 0;
				Swerve1.OdoFlagY = 2;
			}
		else if (Swerve1.Direc<0)
			{
				Swerve1.OdoFlagX = 1;
				Swerve1.OdoFlagY = 2;
			}
		Swerve1.Xcoef = 1;
		Swerve1.Ycoef = 0;
	}
	//Vector chi huong 180 do :
	else if (currQuad1 == -2)
	{
		if(Swerve1.Direc>0)
			{
				Swerve1.OdoFlagX = 1;
				Swerve1.OdoFlagY = 2;
			}
		else if (Swerve1.Direc<0)
			{
				Swerve1.OdoFlagX = 0;
				Swerve1.OdoFlagY = 2;
			}
		Swerve1.Xcoef = 1;
		Swerve1.Ycoef = 0;
	}
	//Vecto chi huong 90 do :
	else if (currQuad1 == -3)
	{
		if(Swerve1.Direc>0)
			{
				Swerve1.OdoFlagX = 2;
				Swerve1.OdoFlagY = 0;
			}
		else if (Swerve1.Direc<0)
			{
				Swerve1.OdoFlagX = 2;
				Swerve1.OdoFlagY = 1;
			}
		Swerve1.Xcoef = 0;
		Swerve1.Ycoef = 1;
	}
	//vecto chi huong 270 do :
	else if (currQuad1 == -4)
	{
		if(Swerve1.Direc>0)
			{
				Swerve1.OdoFlagX = 2;
				Swerve1.OdoFlagY = 1;
			}
		else if (Swerve1.Direc>0)
			{
				Swerve1.OdoFlagX = 2;
				Swerve1.OdoFlagY = 0;
			}
		Swerve1.Xcoef = 0;
		Swerve1.Ycoef = 1;
	}
	else{
		Swerve1.Xcoef = 0;
		Swerve1.Ycoef = 0;
	}

if (Swerve1.Ycoef<0)Swerve1.Ycoef=Swerve1.Ycoef*-1;
}

//----------------------------------------End:Kinematics---------------------------------------//
//MMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMM//

//MMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMM//
//----------------------------------------Begin:Position Track---------------------------------//
Optimizer Base;
PID_Param PIDAngle;
PID_Param PIDOdoU;
PID_Param PIDOdoV;

double u1,v1;
double Prex1,Prey1;
int TrajecFlag;
double TargetAngle = 0;

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(OdoFlag2!=0){
		if (GPIO_Pin == Odo_Extio_Pin)
		{
			if(HAL_GPIO_ReadPin(Odo_Input_GPIO_Port, Odo_Input_Pin)==0)OdorCnt++;
			else OdorCnt--;
		}
	}
}




void OdoCountLogic(void){
	//Xac dinh chuyen vi theo phuong X:
	if (Swerve1.OdoFlagX == 0)
	{
		Swerve1.XCurrPos += (0.054*M_PI/1000)*OdorCnt*Swerve1.Xcoef;
	}
	else if (Swerve1.OdoFlagX == 1)
	{
		Swerve1.XCurrPos -= (0.054*M_PI/1000)*OdorCnt*Swerve1.Xcoef;
	}
	else if (Swerve1.OdoFlagX == 2)
	{
		Swerve1.XCurrPos += 0;
	}
	//Xac dinh chuyen vi theo phuong Y:
	if (Swerve1.OdoFlagY == 0)
	{
		Swerve1.YCurrPos += (0.054*M_PI/1000)*OdorCnt*Swerve1.Ycoef;
	}
	else if (Swerve1.OdoFlagY == 1)
	{
		Swerve1.YCurrPos -= (0.054*M_PI/1000)*OdorCnt*Swerve1.Ycoef;
	}
	else if (Swerve1.OdoFlagY == 2)
	{
		Swerve1.YCurrPos += 0;
	}
	OdorCnt = 0;
}

//----------------------------------------End:Position Track-----------------------------------//
//MMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMM//




//MMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMM//
//----------------------------------------Begin:PurePursuilt-----------------------------------//

double Points[5][2] = {{0,0},{0,0.5},{1,1},{0,2},{0,0}};
double CurrPosition[2] = {0,0};
double LookAheadDis = 0.3;

uint16_t LFIndex;
uint8_t IntersectionFound=0;
uint8_t STIndex;

double X1,X2,Y1,Y2;
double Solx1,Solx2,Soly1,Soly2;
double CurrX = 0,CurrY = 1;
double dx,dy,dr,D,discriminant;

double Solptn1[2],Solptn2[2],GoalPtn[2];

double minX,minY,maxX,maxY;

uint8_t EndOfPath;

int pathlen;
//Tim Khoang cach giua hai diem :
double PointDistances(double* pt1,double* pt2)
{
	double Distance = 0;
	Distance = sqrt(pow((pt2[0]-pt1[0]),2)+pow((pt2[1]-pt1[1]),2));
	return Distance;
}

//Ham kiem tra dau :
int sgn(double num)
{
	if(num >=0)return 1;
	else return 0;
}


double min(double a,double b)
{
	double min;
	min = a;
	if (b<=min)
	{
		min = b;
	}
	return min;
}



double absDouble(double num)
{
	if (num >=0)return num;
	else return num*-1;
}


double TargetPoint[2];
double PrevTarget[2];
uint8_t NextIndex;



void PurePursuilt(void)
{
	pathlen = sizeof(Points)/sizeof(Points[0]);
	STIndex = LFIndex;
	IntersectionFound = 0;
	CurrPosition[0]=Swerve1.XCurrPos;
	CurrPosition[1]=Swerve1.YCurrPos;
	for (int i = STIndex;i<pathlen-1;i++)
	{
		EndOfPath = 0;

		X1 = Points[i][0]-Swerve1.XCurrPos;
		Y1 = Points[i][1]-Swerve1.YCurrPos;


		X2 = Points[i+1][0]-Swerve1.XCurrPos;
		Y2 = Points[i+1][1]-Swerve1.YCurrPos;

		dx = X2-X1;
		dy = Y2-Y1;

		dr = sqrt(pow(dx,2)+pow(dy,2));

		D = X1*Y2-X2*Y1;

		discriminant = pow(LookAheadDis,2)*pow(dr,2)-pow(D,2);

		if (discriminant>=0)
		{
			Solx1 = (D*dy+sgn(dy)*dx*sqrt(discriminant))/pow(dr,2);
			Solx2 = (D*dy-sgn(dy)*dx*sqrt(discriminant))/pow(dr,2);
			Soly1 = (-D*dx+absDouble(dy)*sqrt(discriminant))/pow(dr,2);
			Soly2 = (-D*dx-absDouble(dy)*sqrt(discriminant))/pow(dr,2);

			Solptn1[0]=Solx1+Swerve1.XCurrPos;
			Solptn1[1]=Soly1+Swerve1.YCurrPos;

			Solptn2[0]=Solx2+Swerve1.XCurrPos;
			Solptn2[1]=Soly2+Swerve1.YCurrPos;

			minX = min(Points[i][0],Points[i+1][0]);
			minY = min(Points[i][1],Points[i+1][1]);
			maxX = max(Points[i][0],Points[i+1][0]);
			maxY = max(Points[i][1],Points[i+1][1]);

			if(( ((Solptn1[0]>=minX)&&(Solptn1[0]<=maxX))
			&& ((Solptn1[1]>=minY)&&(Solptn1[1]<=maxY)) )
			|| ( ((Solptn2[0]>=minX)&&(Solptn2[0]<=maxX))
			&& ((Solptn2[1]>=minY)&&(Solptn2[1]<=maxY)) ))
			{
				IntersectionFound = 1;

				if (( ((Solptn1[0]>=minX)&&(Solptn1[0]<=maxX))
					&& ((Solptn1[1]>=minY)&&(Solptn1[1]<=maxY)) )
					&& ( ((Solptn2[0]>=minX)&&(Solptn2[0]<=maxX))
					&& ((Solptn2[1]>=minY)&&(Solptn2[1]<=maxY)) ))
				{
					if((PointDistances(Solptn1,Points[i+1])) < (PointDistances(Solptn2,Points[i+1])))
					{
						GoalPtn[0] = Solptn1[0];
						GoalPtn[1] = Solptn1[1];
					}else{
						GoalPtn[0] = Solptn2[0];
						GoalPtn[1] = Solptn2[1];
					}
				}else {
					if( ((Solptn1[0]>=minX)&&(Solptn1[0]<=maxX))
					&& ((Solptn1[1]>=minY)&&(Solptn1[1]<=maxY)) )
					{
						GoalPtn[0] = Solptn1[0];
						GoalPtn[1] = Solptn1[1];
					}else{
						GoalPtn[0] = Solptn2[0];
						GoalPtn[1] = Solptn2[1];
					}
				}

            if (PointDistances(GoalPtn, Points[i+1])<PointDistances(CurrPosition, Points[i+1])){
                LFIndex = i;
                break;
            }
            else if (LFIndex==pathlen-2)
				{
					EndOfPath = 1 ;
					GoalPtn[0] = Points[i+1][0];
					GoalPtn[1] = Points[i+1][1];
					LFIndex = i+1;
                }
            else{
            	LFIndex = i+1;
            }
			}
			else{
				IntersectionFound = 0;
				GoalPtn[0] = PrevTarget[0];
				GoalPtn[1] = PrevTarget[1];
			}
		}else{
			IntersectionFound = 0;
			NextIndex = LFIndex+1;
			if(NextIndex>pathlen-1)NextIndex=pathlen-1;
			GoalPtn[0] = Points[LFIndex][0];
			GoalPtn[1] = Points[LFIndex][1];
		}
	}
	PrevTarget[0] = GoalPtn[0];
	PrevTarget[1] = GoalPtn[1];

	TargetPoint[0]=GoalPtn[0];
	TargetPoint[1]=GoalPtn[1];
}

//----------------------------------------End:PurePursuilt-------------------------------------//
//MMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMM//

//MMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMM//
//----------------------------------------Begin:PIDController----------------------------------//


double kP=1.5,kI,kD=0.1,alpha=0.8,T = 0.08,Ia,Ib,Ua = 40,Ub =-40;
double kP1=1.2,kI1,kD1=0.05,alpha1=0.1,T1 = 0.08,Ia1,Ib1,Ua1 = 0.3,Ub1 =-0.2;
double kP2=1.2,kI2,kD2=0.05,alpha2=0.1,T2 = 0.08,Ia2,Ib2,Ua2 = 0.3,Ub2 =-0.2;

uint8_t esTablished,establishCheck;
void SteadyStateCheck(void)
{

	if ((abs(e_CalOnly(&PIDOdoU,Solptn1[0],Swerve1.XCurrPos))<0.01) && (abs(e_CalOnly(&PIDOdoV,Solptn1[1],Swerve1.YCurrPos))<0.01))
	{
		establishCheck+=1;
		if(establishCheck>3)
		{
			esTablished = 1;
			establishCheck = 3;
		}
		else {
			esTablished = 0;
		}
	}else{
		establishCheck = 0;
	}
}

void ResetCheckPara(void)
{
	esTablished = 0;
	establishCheck = 0;
}




void PIDPathFollow(void)
{
	  Pid_SetParam(&PIDAngle,kP,kI,kD,alpha,T,Ia,Ib,Ua,Ub);
	  Pid_SetParam(&PIDOdoU,kP1,kI1,kD1,alpha1,T1,Ia1,Ib1,Ua1,Ub1);
	  Pid_SetParam(&PIDOdoV,kP2,kI2,kD2,alpha2,T2,Ia2,Ib2,Ua2,Ub2);

	  if(EndOfPath == 1)
	  {
		  SteadyStateCheck();
	  }
	  Pid_Cal(&PIDOdoV,TargetPoint[0],Swerve1.XCurrPos);
	  Pid_Cal(&PIDOdoU,TargetPoint[1],Swerve1.YCurrPos);
	  if (esTablished!= 1)
	  {
		  Pid_Cal(&PIDAngle,TargetAngle,CurrAngle);
	  }else {
		  PIDAngle.u = 0;
	  }
}

//----------------------------------------End:PIDController------------------------------------//
//MMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMM//


void TrajectoryCtr1(double x,double y,double t ,double Phi)
{
	if(x!= Prex1){
		u1=(x-Prex1)/t;
		v1=(y-Prey1)/t;
		TargetAngle = Phi;

		if ((x>Prex1)&&(y>Prey1))
		{
			TrajecFlag = 1;
		}else if ((x>Prex1)&&(y<Prey1))
		{
			TrajecFlag = 2;
		}else if ((x<Prex1)&&(y>Prey1))
		{
			TrajecFlag = 3;
		}else if ((x<Prex1)&&(y<Prey1))
		{
			TrajecFlag = 4;
		}else if ((x>Prex1)&&(y==Prey1))
		{
			TrajecFlag = 5;
		}else if ((x<Prex1)&&(y==Prey1))
		{
			TrajecFlag = 6;
		}else if ((x==Prex1)&&(y<Prey1))
		{
			TrajecFlag = 7;
		}else if ((x==Prex1)&&(y<Prey1))
		{
			TrajecFlag = 8;
		}else{
			TrajecFlag = 0;
		}
		Prex1 = x;
		Prey1 = y;
	}
}

double uT,vT,rT;

uint8_t PIDCalFlag;

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
//  HAL_UART_Receive_IT(&huart2, (uint8_t *)uart2_ds, 5);

  HAL_UARTEx_ReceiveToIdle_DMA(&huart2, uart2_ds, 5);
  __HAL_DMA_DISABLE_IT(&hdma_usart2_rx,DMA_IT_HT);
//  TargetAngle=GocRobot;


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
  /* definition and creation of TaskPathTrack */
  osThreadDef(TaskPathTrack, PathTracking, osPriorityNormal, 0, 128);
  TaskPathTrackHandle = osThreadCreate(osThread(TaskPathTrack), NULL);

  /* definition and creation of TaskPIDCal */
  osThreadDef(TaskPIDCal, StartTask02, osPriorityNormal, 0, 128);
  TaskPIDCalHandle = osThreadCreate(osThread(TaskPIDCal), NULL);

  /* definition and creation of KinematicTask */
  osThreadDef(KinematicTask, KinematicCal, osPriorityBelowNormal, 0, 128);
  KinematicTaskHandle = osThreadCreate(osThread(KinematicTask), NULL);

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

//	  if (GamePad.Touch == 1)
//	  {
//		  ControlDriver(1,2,0,Swerve1.CurrentAngle,2,2,0,Swerve2.CurrentAngle);
//	  }
//	  else {
//			 if(GamePad.Square == 1){
//				 OdoFlag2 = 0;
//			 }else if(GamePad.Circle == 1)
//			 {
//				 OdoFlag2 = 1;
//			 }
//	  	 }
//
//	  OdoCountLogic();
//	  if(OdoFlag2==1){
//		  PurePursuilt();
//		  PIDPathFollow();
//	  }
//	  InverseKine(uT,vT,rT);
//	  ControlDriver(1,1,wheel_AngleVel1,Swerve1.CurrentAngle,2,1,wheel_AngleVel2,Swerve2.CurrentAngle);
//	  HAL_Delay(T*1000);
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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);

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
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

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
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_PathTracking */
/**
  * @brief  Function implementing the TaskPathTrack thread.
  * @param  argument: Not used
  * @retval None
  */

/* USER CODE END Header_PathTracking */
void PathTracking(void const * argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
//	 if(GamePad.Triangle == 1){
//		 OdoFlag2 = 0;
//	 }else if(GamePad.Circle == 1)
//	 {
//		 OdoFlag2 = 1;
//	 }else if(GamePad.Square == 1){
//		 OdoFlag2 = 2;
//	 }
//
//
//	if (OdoFlag2 == 2)
//	{
	  	  if (GamePad.Touch == 1)
	  	  {
	  		  ControlDriver(1,2,0,Swerve1.CurrentAngle,2,2,0,Swerve2.CurrentAngle);
	  	  }else {
	  		  InverseKine(Yleft,Xleft,Xright);
			ControlDriver(1,1,wheel_AngleVel1,Swerve1.CurrentAngle,2,1,wheel_AngleVel2,Swerve2.CurrentAngle);
//	}
	  	  }
//	  SnakeTurn();
    osDelay(T*1000);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartTask02 */
/**
* @brief Function implementing the TaskPIDCal thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask02 */
void StartTask02(void const * argument)
{
  /* USER CODE BEGIN StartTask02 */
  /* Infinite loop */
  for(;;)
  {

    osDelay(1);
  }
  /* USER CODE END StartTask02 */
}

/* USER CODE BEGIN Header_KinematicCal */
/**
* @brief Function implementing the KinematicTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_KinematicCal */
void KinematicCal(void const * argument)
{
  /* USER CODE BEGIN KinematicCal */
  /* Infinite loop */
  for(;;)
  {
//	 if(OdoFlag2 == 1)
//	 {
//		osDelay(T*1000);
//	 }
	  osDelay(1);
  }
  /* USER CODE END KinematicCal */
}

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
