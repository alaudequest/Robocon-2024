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
#include "Encoder.h"
#include "PositionControl.h"
#include "ActuatorValve.h"
//#include "AppInterface.h"
#include "PutBall.h"
#include "LogData.h"
#include "BlueTeamProcess.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum MainEvent {
	MEVT_GET_NODE_SPEED_ANGLE,
} MainEvent;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define REDBALL 0
#define BLUEBALL 1
#define PURPLEBALL 2
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;

CAN_HandleTypeDef hcan1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim10;

UART_HandleTypeDef huart5;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart2_tx;

osThreadId defaultTaskHandle;
osThreadId TaskInvKineHandle;
uint32_t TaskInvKineBuffer[ 256 ];
osStaticThreadDef_t TaskInvKineControlBlock;
osThreadId TaskCANHandle;
uint32_t TaskCANBuffer[ 128 ];
osStaticThreadDef_t TaskCANControlBlock;
osThreadId TaskOdometerHandle;
osThreadId taskFlashHandle;
/* USER CODE BEGIN PV */

//CAN_DEVICE_ID targetID = CANCTRL_DEVICE_MOTOR_CONTROLLER_1;
PID_Param pid;
PID_type type = PID_BLDC_SPEED;

CAN_MODE_ID Mode_ID = CANCTRL_MODE_MOTOR_SPEED_ANGLE;
PID_Param targetPID =
		{ .deltaT = 0.001, .kP = 10, .kI = 10, .kD = 1, .alpha = 1, };
PID_type pidType = PID_BLDC_SPEED;




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

uint8_t UARTRX3_Buffer[9];
uint8_t DataTayGame[9];

float Xleft, Yleft;
float Xright;

_GamePad GamePad;
uint32_t gamepadRxIsBusy = 0;

float DeltaYR, DeltaYL, DeltaX;
uint8_t Run,resetParam,breakProtect;
float uControlX,uControlY,uControlTheta;
uint8_t stateRun = 0 ,steadycheck;
uint8_t xaDay;
bool startToFlash = false;
bool resetAllDataFlash = false;
uint8_t ssCheck,stateChange,rst,Gamepad;

//////////////////////////////////////////////////////////////////////////////////////////////////////////

uint8_t step = 0;
uint8_t angle_PID_Cycle;
////////////MPU//////////////
uint8_t mpu[10];
uint8_t send_mpu;
int16_t angle;
float a_Now;
float angle_Rad;
uint8_t reset_MPU;

///////////PID MPU///////////////
PID_Param pid_Angle;
uint8_t   use_pidTheta;
float 	  TargetTheta,TargetTheta_Degree;
//////////Floating Enc///////
uint8_t reset_ENC;
Encoder_t FloatingEnc;
int floatingEncCount;

//////////Control Signal/////
float u,v,r;

//////////Process ///////////
uint8_t RunProcess;
float chasis_Vector_TargetSpeed;
uint8_t angle_Accel_Flag;
float PreTargetAngle;
float process_AutoChose;
float process_AutoChose_Count;
uint8_t process_SubState;
uint8_t process_GetBall_State;
uint8_t process_SSCheck;
uint8_t process_Count;

//////////LineFollow////////
uint8_t LineSensor[8];
uint8_t DataLineX;
uint8_t DataLineY;
int8_t LineX;
int8_t LineY;
int KtLineX=0;
int KtLineY=0;
uint8_t SendLine='a';
int v_line=30;
int v_Line;
//////////////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////CAM BIEN DO KHOANG CACH///////////////////////////////////////////
float count,adc_val_Fil,sum;
float adc_val_Fil2,sum2;
float adc_val = 0;
float adc_val2 = 0;
float distance = 0;
float distance2 = 0;

///////////////////////////////////////Quy Hoach Quy Dao///////////////////////////////////////////
trajec_Param trajecTheta;

Encoder_t PutBall_Enc;
PID_Param PutBall_PID;

MotorDC putBall_DC;
MotorDC GetBall_DC;
///////////////////////////////////////Truyen Thong Xu Ly Anh///////////////////////////////////////
uint8_t UARTRX5_Buffer[4], Raspberry[4];
uint8_t UARTTX5_OK_Buffer[3] = "OK\n";
uint8_t UARTTX5_Start_Buffer[2] = "a\n";
bool UART5_IsReceived = false;
bool Raspberry_Enable = false;


uint8_t Safety_Mode;
int column,row;
float distance_Delta, distance_Pre;
uint8_t distanceFlag,GetballFlag;

uint8_t Team,Start,Retry;
///////////////////////////////////////Xu ly silo///////////////////////////////////////

#define SILO_12_DISTANCE 4.256
#define SILO_23_DISTANCE 3.203
#define SILO_34_DISTANCE 2.155
#define SILO_45_DISTANCE 1.083

#define SILO_1_DISTANCE 3.25
#define SILO_2_DISTANCE 2.55
#define SILO_3_DISTANCE 1.8
#define SILO_4_DISTANCE 1.08
#define SILO_5_DISTANCE 0.35

#define SILO_MAX_BALL 1
////
#define RED   1
#define BLUE  2
#define RETRY_ENABLE	1
#define RETRY_DISABLE	0
#define START	1
#define IDLE	0


uint8_t TargetSilo,lastRow;
//////////////////////////////////////////////////////////////////////////////////////////////////////////

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
static void MX_ADC1_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM5_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_UART5_Init(void);
static void MX_ADC2_Init(void);
void StartDefaultTask(void const * argument);
void InverseKinematic(void const * argument);
void CAN_Bus(void const * argument);
void OdometerHandle(void const * argument);
void TaskFlash(void const * argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/*=============================== CAN ===============================*/
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
	HAL_CAN_DeactivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
	CAN_MODE_ID modeID = canctrl_Receive_2(hcan, CAN_RX_FIFO0);
	BaseType_t HigherPriorityTaskWoken = pdFALSE;
	xTaskNotifyFromISR(TaskCANHandle, modeID, eSetValueWithOverwrite,
			&HigherPriorityTaskWoken);
	portYIELD_FROM_ISR(HigherPriorityTaskWoken);
}
void CAN_Init() {
	HAL_CAN_Start(&hcan1);
	HAL_CAN_ActivateNotification(&hcan1,
			CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_RX_FIFO1_MSG_PENDING);
	canctrl_Filter_Mask16(&hcan1,
			CANCTRL_MODE_SET_HOME << 5, //mask bit Set Home không trùng bit với bit Robot Error thì mới l�?c được
			CANCTRL_MODE_ROBOT_ERROR << 5,
			CANCTRL_MODE_SET_HOME << 5,
			CANCTRL_MODE_ROBOT_ERROR << 5,
			0,
			CAN_RX_FIFO0);
}

void setHomeComplete()
{
	HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, 1);
	osDelay(50);
	HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, 0);
}

void robotErrorHandler(CAN_DEVICE_ID targetID)
{

	while(1){
		Safety_Mode = 1;
		if(targetID == CANCTRL_DEVICE_MOTOR_CONTROLLER_1){
			HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, 1);
			osDelay(50);
			HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, 0);
			osDelay(1000);
		}
		else if(targetID == CANCTRL_DEVICE_MOTOR_CONTROLLER_2){
			for(uint8_t i = 0; i < 2; i++){
				HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, 1);
				osDelay(100);
				HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, 0);
			}
			osDelay(1000);
		}
		else if(targetID == CANCTRL_DEVICE_MOTOR_CONTROLLER_3){
			for(uint8_t i = 0; i < 3; i++){
				HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, 1);
				osDelay(100);
				HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, 0);
			}
			osDelay(1000);
		}
	}
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
	case CANCTRL_MODE_UNTANGLE_WIRE:

		break;
	case CANCTRL_MODE_ROBOT_ERROR:
		robotErrorHandler(targetID);
		break;
	default:
		break;
	}

}
/*=============================== UART ===============================*/
//uint8_t YawHandle;
//uint8_t AngleData[5];
//int CurrAngle;
//
//char ds[12];
//uint8_t uart2_ds[5], ds_ind, ds_cnt, ds_flg;
//
//void Receive(uint8_t *DataArray){
//      uint8_t *pInt = NULL;
//      if(DataArray[4] == 13){
//           pInt = &CurrAngle;
//           for(uint8_t i = 0; i < 4; i++) {
//               *(pInt + i) = DataArray[i];
//            }
//      }
//      	  memset(DataArray,0,5);
//      	YawHandle = 1;
// }
//void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
//{
//	if(huart -> Instance == USART1)
//	{
//		HAL_UARTEx_ReceiveToIdle_DMA(&huart1, uart2_ds, 5);
//			Receive(uart2_ds);
//		  __HAL_DMA_DISABLE_IT(&hdma_usart1_rx,DMA_IT_HT);
//	}
//}
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
			CustomGamepad_t customGamepad;
			for (uint8_t i = 0; i < 6; i++) {
				customGamepad.ballMatrix[i] = DataTayGame[i + 1];
			}
			customGamepad.siloNum = DataTayGame[7];
			BrdParam_SetCustomGamepad(customGamepad);
		}
		if (!gamepadRxIsBusy)
			HAL_UART_Receive_IT(&huart3, (uint8_t*) UARTRX3_Buffer, 9);

	}

	if (huart->Instance == UART5) {
		memcpy(Raspberry, UARTRX5_Buffer, 4);
		memset(UARTRX5_Buffer, 0, 4);
		UART5_IsReceived = true;
		HAL_UART_Receive_IT(&huart5, (uint8_t*) UARTRX5_Buffer, 1);
	}
}


void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan) {
	while (1)
		;
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
	__HAL_UART_CLEAR_OREFLAG(huart);
	memset(UARTRX3_Buffer, 0, sizeof(UARTRX3_Buffer));
	HAL_UART_Receive_IT(&huart3, (uint8_t*) UARTRX3_Buffer, 9);
	__HAL_UART_DISABLE(huart);
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
	log_TransmitCompleteHandle(huart);
}

void Send_Data(){

	log_SendString();
}

void Send_Header(){
	log_AddHeaderArgumentToBuffer("PoseX");
	log_AddHeaderArgumentToBuffer("PoseY");
	log_AddHeaderArgumentToBuffer("PoseTheta");

	log_AddHeaderArgumentToBuffer("TrajecPlanX");
	log_AddHeaderArgumentToBuffer("TrajecPlanY");
	log_AddHeaderArgumentToBuffer("TrajecPlanTheta");

	log_SendString();
}


float absf(float num)
{
	if (num>=0)return num;
	else return num*-1;
}
///////////////////////////MPU//////////////////////////
#define DELTA_T 0.05
int16_t xxx,yyy,sumxy;

void Get_MPU_Angle()
{
    send_mpu ='z';
    HAL_UART_Transmit(&huart1,&send_mpu,1,1);
    angle=mpu[0]<<8|mpu[1];
    a_Now=angle;
    xxx=mpu[2]<<8|mpu[3];
    yyy=mpu[4]<<8|mpu[5];
    sumxy = xxx+yyy;
}

void Reset_MPU_Angle()
{
    send_mpu ='a';
    HAL_UART_Transmit(&huart1,&send_mpu,1,1);

}
/////////////////////Process///////////////////



void process_Init()
{
	////////PID/////////
	pid_Angle.kP = 1.2;
	pid_Angle.kI = 0;
	pid_Angle.kD = 0;
	pid_Angle.alpha = 0;
	pid_Angle.deltaT = DELTA_T;
	pid_Angle.u_AboveLimit = 5;
	pid_Angle.u_BelowLimit = -5;
	pid_Angle.kB = 1/DELTA_T;

	process_AutoChose = 0;

	PutBall_PID.kP = 0;
	PutBall_PID.kI = 0;
	PutBall_PID.kD = 0;
	PutBall_PID.alpha = 0;
	PutBall_PID.deltaT = DELTA_T;
	PutBall_PID.u_AboveLimit = 0;
	PutBall_PID.u_BelowLimit = 0;
	PutBall_PID.kB = 1/DELTA_T;

	encoder_Init(&FloatingEnc, &htim1, 200, DELTA_T);
	encoder_Init(&PutBall_Enc,&htim2, 7000/4, DELTA_T);
	MotorDC_Init(&putBall_DC, &htim3, MOTOR_PWM_INVERSE, TIM_CHANNEL_1, TIM_CHANNEL_2);
	MotorDC_Init(&GetBall_DC, &htim5, MOTOR_PWM_NORMAL, TIM_CHANNEL_1, TIM_CHANNEL_2);
}

void process_PD_OnStrainghtPath()
{
	pid_Angle.kP = 2;
	pid_Angle.kI = 0;
	pid_Angle.kD = 0;
	pid_Angle.alpha = 0;
	pid_Angle.deltaT = DELTA_T;
	pid_Angle.u_AboveLimit = 5;
	pid_Angle.u_BelowLimit = -5;
	pid_Angle.kB = 1/DELTA_T;
}

void process_PD_OnTrajecPath()
{
	pid_Angle.kP = 1.5;
	pid_Angle.kI = 0;
	pid_Angle.kD = 0;
	pid_Angle.alpha = 0;
	pid_Angle.deltaT = DELTA_T;
	pid_Angle.u_AboveLimit = 5;
	pid_Angle.u_BelowLimit = -5;
	pid_Angle.kB = 1/DELTA_T;
}

void process_PD_Critical()
{
	pid_Angle.kP = 3;
	pid_Angle.kI = 0;
	pid_Angle.kD = 0.3;
	pid_Angle.alpha = 0.1;
	pid_Angle.deltaT = DELTA_T;
	pid_Angle.u_AboveLimit = 5;
	pid_Angle.u_BelowLimit = -5;
	pid_Angle.kB = 1/DELTA_T;
}

void process_PD_Auto_Chose(float Target,float Current)
{
	if(process_AutoChose == 0)
	{
		if(absf(Target - Current)<5*M_PI/180)
		{
			process_AutoChose_Count++;
		}else{
			process_AutoChose_Count = 0;
		}
	}

	if (process_AutoChose_Count>15)
	{
		process_AutoChose_Count = 0;
		process_AutoChose = 1;
	}

	if (process_AutoChose == 1)
	{
		process_PD_OnStrainghtPath();
	}else{
		process_PD_OnTrajecPath();
	}
}
void process_SetFloatingEnc()
{
	floatingEncCount = encoder_GetFloatingDis(&FloatingEnc);
}

void process_ResetFloatingEnc()
{
	encoder_ResetCount(&FloatingEnc);
	floatingEncCount = 0;
}
void process_Accel_FloatingEnc(float Angle,float maxSpeed,float s,float accel,float TargetAngle,float accelAngle)
{
	if (TargetAngle != PreTargetAngle)
	{
		if (TargetAngle > PreTargetAngle)
		{
			angle_Accel_Flag = 0;
		}
		if (TargetAngle < PreTargetAngle)
		{
			angle_Accel_Flag = 1;
		}
	}

	if (angle_Accel_Flag == 0)
	{
		TargetTheta_Degree += accelAngle;
		if (TargetTheta_Degree>TargetAngle)TargetTheta_Degree = TargetAngle;
	}else{
		TargetTheta_Degree -= accelAngle;
		if (TargetTheta_Degree<TargetAngle)TargetTheta_Degree = TargetAngle;
	}

	PreTargetAngle = TargetAngle;


	TargetTheta = TargetTheta_Degree*M_PI/180;


	if ((floatingEncCount < 500)&&(chasis_Vector_TargetSpeed<maxSpeed))
	{
		chasis_Vector_TargetSpeed += accel;
	}

	if ((floatingEncCount > 500)&&(floatingEncCount < (s - 500)))
	{
		chasis_Vector_TargetSpeed = maxSpeed/2;
	}

	if (floatingEncCount > (s - 400)){
		chasis_Vector_TargetSpeed -= accel ;
	}
	if (floatingEncCount > (s - 300)){
		use_pidTheta = 0;
		r = 0;
	}

	if ((chasis_Vector_TargetSpeed<=0)||(floatingEncCount > s))
	{
		chasis_Vector_TargetSpeed = 0;
		process_ResetFloatingEnc();
		floatingEncCount = 0;
		use_pidTheta = 0;
		process_AutoChose = 0;
		r = 0;
		angle_Accel_Flag = 2;
		step += 1;
	}

	u = cos(Angle*M_PI/180)*chasis_Vector_TargetSpeed ;
	v = sin(Angle*M_PI/180)*chasis_Vector_TargetSpeed ;
}

void process_Accel_FloatingEnc2(float Angle,float maxSpeed,float s,float accel)
{
	use_pidTheta = 1;
	if ((floatingEncCount < 500)&&(chasis_Vector_TargetSpeed<maxSpeed))
	{
		chasis_Vector_TargetSpeed += accel;
	}

	if ((floatingEncCount > 500)&&(floatingEncCount < (s - 500)))
	{
		chasis_Vector_TargetSpeed = maxSpeed;
	}
	if (floatingEncCount > (s - 500)&&floatingEncCount < (s - 400))
	{
		chasis_Vector_TargetSpeed = maxSpeed/2		;
	}
	if (floatingEncCount > (s - 400)){
		chasis_Vector_TargetSpeed -= accel ;
	}
//	if (floatingEncCount > (s - 300)){
//		use_pidTheta = 0;
//		r = 0;
//	}

	if ((chasis_Vector_TargetSpeed<=0)||(floatingEncCount > s))
	{
		chasis_Vector_TargetSpeed = 0;
		process_ResetFloatingEnc();
		r = 0;
		u = 0;
		v = 0;
		use_pidTheta = 0;
		step += 1;
	}else{
		u = cos(Angle*M_PI/180)*chasis_Vector_TargetSpeed ;
		v = sin(Angle*M_PI/180)*chasis_Vector_TargetSpeed ;

	}
}

float AngleNow,AngleTargetPre,AngleTarget,AngleFlag;
void process_Accel_FloatingEnc3(float Angle,float maxSpeed,float s,float accel,float TargetAngle,float RotateTime,float AngleChange)
{
	if (process_SubState == 0)
	{
		AngleTarget = Angle;
		if (AngleTarget > AngleTargetPre){
			AngleFlag = 1;
		}else{
			AngleFlag = 2;
		}
		trajecPlan_SetParam(&trajecTheta, angle_Rad, TargetAngle*M_PI/180, RotateTime, 0, 0);
		process_ResetFloatingEnc();
		process_SubState = 1;
	}else{
		if(AngleFlag == 1)
		{
			AngleNow += AngleChange;
			if (AngleNow > AngleTarget)
			{
				AngleNow = AngleTarget;
			}
		}else if (AngleFlag == 2){
			AngleNow -= AngleChange;
			if (AngleNow < AngleTarget)
			{
				AngleNow = AngleTarget;
			}
		}

		AngleTargetPre = AngleTarget;

		use_pidTheta = 1;
		if ((floatingEncCount < 500)&&(chasis_Vector_TargetSpeed<maxSpeed))
		{
			chasis_Vector_TargetSpeed += accel;
		}
		if ((floatingEncCount > 500)&&(floatingEncCount < (s - 500)))
		{
			chasis_Vector_TargetSpeed = maxSpeed;
		}
		if (floatingEncCount > (s - 500)&&floatingEncCount < (s - 400)){
			chasis_Vector_TargetSpeed = maxSpeed/2		;
			}
		if (floatingEncCount > (s - 400)){
			chasis_Vector_TargetSpeed -= accel ;
		}


		if ((chasis_Vector_TargetSpeed<=0)||(floatingEncCount > s))
		{
			chasis_Vector_TargetSpeed = 0;
			process_ResetFloatingEnc();
			r = 0;
			u = 0;
			v = 0;
			use_pidTheta = 0;
			process_SubState = 0;
			step += 1;
		}else{
			u = cos(AngleNow*M_PI/180)*chasis_Vector_TargetSpeed ;
			v = sin(AngleNow*M_PI/180)*chasis_Vector_TargetSpeed ;

		}
	}

}
uint8_t process_Subsubstate;
void process_Accel_FloatingEnc_Sub(float Angle,float maxSpeed,float s,float accel,float TargetAngle,float RotateTime)
{
	if (process_Subsubstate == 0)
	{
		AngleTarget = Angle;
		if (AngleTarget > AngleTargetPre){
			AngleFlag = 1;
		}else{
			AngleFlag = 2;
		}
		trajecPlan_SetParam(&trajecTheta, angle_Rad, TargetAngle*M_PI/180, RotateTime, 0, 0);
		process_ResetFloatingEnc();
		process_Subsubstate = 1;
	}else{
		if(AngleFlag == 1)
		{
			AngleNow += 5;
			if (AngleNow > AngleTarget)
			{
				AngleNow = AngleTarget;
			}
		}else if (AngleFlag == 2){
			AngleNow -= 5;
			if (AngleNow < AngleTarget)
			{
				AngleNow = AngleTarget;
			}
		}

		AngleTargetPre = AngleTarget;

		use_pidTheta = 1;
		if ((floatingEncCount < 500)&&(chasis_Vector_TargetSpeed<maxSpeed))
		{
			chasis_Vector_TargetSpeed += accel;
		}
		if ((floatingEncCount > 500)&&(floatingEncCount < (s - 500)))
		{
			chasis_Vector_TargetSpeed = maxSpeed;
		}
		if (floatingEncCount > (s - 500)&&floatingEncCount < (s - 400)){
			chasis_Vector_TargetSpeed = maxSpeed/2		;
			}
		if (floatingEncCount > (s - 400)){
			chasis_Vector_TargetSpeed -= accel ;
		}


		if ((chasis_Vector_TargetSpeed<=0)||(floatingEncCount > s))
		{
			chasis_Vector_TargetSpeed = 0;
			process_ResetFloatingEnc();
			r = 0;
			u = 0;
			v = 0;
			use_pidTheta = 0;
			process_Subsubstate = 0;
			process_SubState += 1;
//			step += 1;
		}else{
			u = cos(AngleNow*M_PI/180)*chasis_Vector_TargetSpeed ;
			v = sin(AngleNow*M_PI/180)*chasis_Vector_TargetSpeed ;

		}
	}

}
void process_Accel_FloatingEnc4(float Angle, float maxSpeed, float s, float accel, float TargetAngle, float RotateTime)
{
	if (process_SubState == 0)
			{
		trajecPlan_SetParam(&trajecTheta, angle_Rad, TargetAngle * M_PI / 180, RotateTime, 0, 0);
		process_ResetFloatingEnc();
		use_pidTheta = 1;
		process_SubState = 1;
	}
	else {
		if(chasis_Vector_TargetSpeed>maxSpeed)chasis_Vector_TargetSpeed=maxSpeed;
		if ((floatingEncCount < 500) && (chasis_Vector_TargetSpeed < maxSpeed))
				{
			chasis_Vector_TargetSpeed += accel;
		}
		if ((floatingEncCount > 500) && (floatingEncCount < (s - 500)))
				{
			chasis_Vector_TargetSpeed = maxSpeed;
		}
		if (floatingEncCount > (s - 500) && floatingEncCount < (s - 400)) {
			chasis_Vector_TargetSpeed = maxSpeed / 2;
		}
		if (floatingEncCount > (s - 400)) {
			chasis_Vector_TargetSpeed -= accel*5;
		}
		if ((chasis_Vector_TargetSpeed <= 0) || (floatingEncCount > s))
		{
			u = 0;
			v = 0;
			if (absf(trajecTheta.Pf-angle_Rad)<2*M_PI/180)
			{
				process_SSCheck ++;
			}else{
				process_SSCheck = 0;
			}


			if(process_SSCheck > 10)
			{
			chasis_Vector_TargetSpeed = 0;
			process_ResetFloatingEnc();
			r = 0;
			process_SSCheck = 0;
			use_pidTheta = 0;
			process_SubState = 0;
			step += 1;
			}
		}
		else {
			u = cos(Angle * M_PI / 180) * chasis_Vector_TargetSpeed;
			v = sin(Angle * M_PI / 180) * chasis_Vector_TargetSpeed;

		}
	}

}
void process_LineFollow(){
//    KtLineY=0;
//   for(int i=0;i<=7;i++)
//      {
//         if((DataLineY&(1<<i))==0)
//        {  KtLineY++;
//           LineY =(int)((i+1)*2);
//           if(((DataLineY&(1<<i-1))==0)&&(i!=0))
//                {
//                  LineY--;
//                }
//         }
//      }
}

void process_RunByAngle(float Angle,float speed)
{
	u = cos(Angle*M_PI/180)*speed ;
	v = sin(Angle*M_PI/180)*speed ;
}

void process_RunByAngle2(float Angle,float speed)
{
	if (process_Subsubstate == 0)
		{
			AngleTarget = Angle;
			if (AngleTarget > AngleTargetPre){
				AngleFlag = 1;
			}else{
				AngleFlag = 2;
			}
//			trajecPlan_SetParam(&trajecTheta, angle_Rad, TargetAngle*M_PI/180, RotateTime, 0, 0);
//			process_ResetFloatingEnc();
			process_Subsubstate = 1;
		}else{
			if(AngleFlag == 1)
			{
				AngleNow += 5;
				if (AngleNow > AngleTarget)
				{
					AngleNow = AngleTarget;
				}
			}else if (AngleFlag == 2){
				AngleNow -= 5;
				if (AngleNow < AngleTarget)
				{
					AngleNow = AngleTarget;
				}
			}

			AngleTargetPre = AngleTarget;
	u = cos(Angle*M_PI/180)*speed ;
	v = sin(Angle*M_PI/180)*speed ;
		}
}
void process_LineApproach()
{
	if(process_SubState == 1)
		{
			process_RunByAngle(45,0.1);
			use_pidTheta = 1;
			if (distance<0.3)
			{
				process_ResetFloatingEnc();
				process_SubState = 2;
			}
		}
}

uint8_t use_DirectVct;
void process_Signal_RotationMatrixTransform(float u, float v ,float r)
{
		uControlX = u*cos(angle_Rad) - v*sin(angle_Rad);
		uControlY = u*sin(angle_Rad) + v*cos(angle_Rad);

	uControlTheta = r;
}

void process_Ball_Approach()
{
	if (process_SubState == 0)
	{	process_Count ++;
		if (process_Count > 10)
		{
			process_Count = 0;
			process_SubState = 1;
		}
	}
	else if(process_SubState == 1)
	{
		process_RunByAngle(45,0.1);
		use_pidTheta = 1;
		if (distance<0.3)
		{
			process_ResetFloatingEnc();
			process_SubState = 2;
		}
	}
	else if (process_SubState == 2)
	{
		process_RunByAngle(45,0.1);
		if (floatingEncCount>140)
		{
			u = 0;
			v = 0;
			r = 0;
			use_pidTheta = 0;
			process_SubState = 3;
		}
	}
	else if (process_SubState == 3)
	{
		use_pidTheta = 1;
		process_RunByAngle(135,-0.08);
		if(distance < 0.19)
		{
			u = 0;
			v = 0;
			r = 0;
			use_pidTheta = 0;
			process_SubState = 4;
		}
	}

	else if (process_SubState == 4)
	{
		use_pidTheta = 1;
		process_RunByAngle(135,0.08);
		if(distance > 0.08)
		{
			u = 0;
			v = 0;
			r = 0;
			use_pidTheta = 0;
			process_SubState = 0;
			step += 1;

		}
	}
}

void process_Ball_Approach2()
{
	if (process_SubState == 0)
	{	process_Count ++;
		if (process_Count > 10)
		{
			process_Count = 0;
			process_SubState = 1;
		}
	}
	else if(process_SubState == 1)
	{
		process_RunByAngle(-135,0.1);
		use_pidTheta = 1;
		if (distance<0.3)
		{
			process_ResetFloatingEnc();
			process_SubState = 2;
		}
	}
	else if (process_SubState == 2)
	{
		process_RunByAngle(-135,0.1);
		if (floatingEncCount>100)
		{
			u = 0;
			v = 0;
			r = 0;
			use_pidTheta = 0;
			process_SubState = 3;
		}
	}
	else if (process_SubState == 3)
	{
		use_pidTheta = 1;
		process_RunByAngle(135,-0.08);
		if(distance < 0.21)
		{
			u = 0;
			v = 0;
			r = 0;
			use_pidTheta = 0;
			process_SubState = 4;
		}
	}

	else if (process_SubState == 4)
	{
		use_pidTheta = 1;
		process_RunByAngle(135,0.08);
		if(distance > 0.15)
		{
			u = 0;
			v = 0;
			r = 0;
			use_pidTheta = 0;
			process_SubState = 0;
			step += 1;

		}
	}
}

void process_Ball_Approach3(uint8_t Ball)
{
	float dis = 120 +(214*2*Ball);


	if (process_SubState == 0)
	{

	process_RunByAngle(0,0.15);
	GetballFlag = 0;

			process_Count = 0;
			process_SubState = 1;
			GetballFlag = 5;

	}
	else if(process_SubState == 1)
	{

		use_pidTheta = 1;
		if (distanceFlag == 5)
		{
			process_ResetFloatingEnc();
			process_SubState = 2;
		}
	}
	else if (process_SubState == 2)
	{
		process_RunByAngle(0,0.15);
		if (floatingEncCount>dis)
		{
			u = 0;
			v = 0;
			r = 0;
			use_pidTheta = 0;
			process_SubState = 3;
		}
	}
	if (process_SubState == 3)
		{
			GetballFlag = 3;
			use_pidTheta = 1;
			process_RunByAngle(90,-0.1);
			if(distanceFlag == 3)
			{
				process_Count = 0;
				u = 0;
				v = 0;
				r = 0;
				use_pidTheta = 0;
				process_SubState += 1;
			}
		}

		else if (process_SubState == 4)
		{
			use_pidTheta = 1;
			process_RunByAngle(90,0.1);
			GetballFlag = 4;
			if(distanceFlag == 4)
			{
				process_Count = 0;
				u = 0;
				v = 0;
				r = 0;
				use_pidTheta = 0;
				process_SubState = 0;
				step += 1;
				}
		}
}


void process_Ball_Approach_BLUE(uint8_t Ball)
{
	float dis = 30 +(214*2*Ball);


	if (process_SubState == 0)
	{

	process_RunByAngle2(-180,0.15);
	GetballFlag = 0;

			process_Count = 0;
			process_SubState = 1;
			GetballFlag = 5;

	}
	else if(process_SubState == 1)
	{

		use_pidTheta = 1;
		if (distanceFlag == 5)
		{
			process_ResetFloatingEnc();
			process_SubState = 2;
		}
	}
	else if (process_SubState == 2)
	{
		process_RunByAngle2(-180,0.15);
		if (floatingEncCount>dis)
		{
			u = 0;
			v = 0;
			r = 0;
			use_pidTheta = 0;
			process_SubState = 3;
		}
	}
	if (process_SubState == 3)
		{
			GetballFlag = 3;
			use_pidTheta = 1;
			process_RunByAngle(90,-0.1);
			if(distanceFlag == 3)
			{
				process_Count = 0;
				u = 0;
				v = 0;
				r = 0;
				use_pidTheta = 0;
				process_SubState += 1;
			}
		}

		else if (process_SubState == 4)
		{
			use_pidTheta = 1;
			process_RunByAngle(90,0.1);
			GetballFlag = 4;
			if(distanceFlag == 4)
			{
				process_Count = 0;
				u = 0;
				v = 0;
				r = 0;
				use_pidTheta = 0;
				process_SubState = 0;
				step += 1;
				}
		}
}

float distanceRear,distanceCenter,Ball_Appr;
void process_Ball_Approach4(uint8_t Ball)
{

	if(Ball == 0)
	{
		if (process_SubState == 0)
		{

		process_RunByAngle2(0,0.15);
		GetballFlag = 0;

				process_Count = 0;
				process_SubState = 1;
				GetballFlag = 5;

		}
		else if(process_SubState == 1)
		{

			use_pidTheta = 1;
			if (distanceFlag == 5)
			{
				process_ResetFloatingEnc();
				process_SubState = 2;
			}
		}
		else if (process_SubState == 2)
		{
			process_RunByAngle2(0,0.15);
			if (floatingEncCount>110)
			{
				u = 0;
				v = 0;
				r = 0;
				use_pidTheta = 0;
				process_SubState = 3;
			}
		}
		if (process_SubState == 3)
			{
				GetballFlag = 3;
				use_pidTheta = 1;
				process_RunByAngle(90,-0.1);
				if(distanceFlag == 3)
				{
					process_Count = 0;
					u = 0;
					v = 0;
					r = 0;
					use_pidTheta = 0;
					process_SubState += 1;
				}
			}

			else if (process_SubState == 4)
			{
				use_pidTheta = 1;
				process_RunByAngle(90,0.1);
				GetballFlag = 4;
				if(distanceFlag == 4)
				{
					process_Count = 0;
					u = 0;
					v = 0;
					r = 0;
					use_pidTheta = 0;
					process_SubState = 0;
					step += 1;
					}
			}
	}else {
		if (process_SubState == 0)
		{

		process_RunByAngle2(0,0.15);
		GetballFlag = 0;

				process_Count = 0;
				process_SubState = 1;
				GetballFlag = 5;

		}
		else if(process_SubState == 1)
		{

			use_pidTheta = 1;
			if (distanceFlag == 5)
			{
//				process_ResetFloatingEnc();
				process_SubState = 2;
			}
		}
		if (process_SubState == 2)
		{

			process_RunByAngle2(0,0.15);
//			GetballFlag = 0;

					process_Count = 0;
					process_SubState = 3;
					GetballFlag = 1;

			}
			else if(process_SubState == 3)
			{

				use_pidTheta = 1;
				if (distanceFlag == 1)
				{
					process_ResetFloatingEnc();
					process_SubState = 4;
				}
			}
		else if (process_SubState == 4)
		{
			process_RunByAngle2(0,0.15);
			if (floatingEncCount>150)
			{
				u = 0;
				v = 0;
				r = 0;
				use_pidTheta = 0;
				process_SubState = 5;
			}
		}
		if (process_SubState == 5)
			{
				GetballFlag = 3;
				use_pidTheta = 1;
				process_RunByAngle(90,-0.1);
				if(distanceFlag == 3)
				{
					process_Count = 0;
					u = 0;
					v = 0;
					r = 0;
					use_pidTheta = 0;
					process_SubState += 1;
				}
			}

			else if (process_SubState == 6)
			{
				use_pidTheta = 1;
				process_RunByAngle(90,0.1);
				GetballFlag = 4;
				if(distanceFlag == 4)
				{
					process_Count = 0;
					u = 0;
					v = 0;
					r = 0;
					use_pidTheta = 0;
					process_SubState = 0;
					step += 1;
					}
			}
	}
}
void Process_Ball_Continue()
{
	if (process_SubState == 0)
	{	process_Count ++;
	u = 0;
	v = 0;
	r = 0;
		if (process_Count > 15)
		{
			process_Count = 0;
			process_SubState = 1;
			GetballFlag = 1;
		}
	}
	else if(process_SubState == 1)
	{
		process_RunByAngle(0,0.15);
		use_pidTheta = 1;
		if (distanceFlag == 1)
		{
			process_ResetFloatingEnc();
			process_SubState = 2;
		}
	}
	else if (process_SubState == 2)
	{
		process_RunByAngle(0,0.15);
		if (floatingEncCount>100)
		{
			u = 0;
			v = 0;
			r = 0;
			use_pidTheta = 0;
			process_SubState = 0;
			step += 1;
			GetballFlag = 0;
			distanceFlag = 0;
		}
	}

}

void Process_Ball_Continue_BLUE()
{
	if (process_SubState == 0)
	{	process_Count ++;
	u = 0;
	v = 0;
	r = 0;
		if (process_Count > 15)
		{
			process_Count = 0;
			process_SubState = 1;
			GetballFlag = 1;
		}
	}
	else if(process_SubState == 1)
	{
		process_RunByAngle(-180,0.15);
		use_pidTheta = 1;
		if (distanceFlag == 1)
		{
			process_ResetFloatingEnc();
			process_SubState = 2;
		}
	}
	else if (process_SubState == 2)
	{
		process_RunByAngle(-180,0.15);
		if (floatingEncCount>25)
		{
			u = 0;
			v = 0;
			r = 0;
			use_pidTheta = 0;
			process_SubState = 0;
			step += 1;
			GetballFlag = 0;
			distanceFlag = 0;
		}
	}

}
void process_GetBall_Near()
{
	if (process_SubState == 0)
	{
		GetballFlag = 3;
		use_pidTheta = 1;
		process_RunByAngle(90,-0.1);
		if(distanceFlag == 3)
		{
			process_Count = 0;
			u = 0;
			v = 0;
			r = 0;
			use_pidTheta = 0;
			process_SubState += 1;
		}
	}

	else if (process_SubState == 1)
	{
		use_pidTheta = 1;
		process_RunByAngle(90,0.1);
		GetballFlag = 4;
		if(distanceFlag == 3)
		{
			process_Count = 0;
			u = 0;
			v = 0;
			r = 0;
			use_pidTheta = 0;
			process_SubState = 0;
			step += 1;
			}
	}
}
void process_Ball_Out(){
	if (process_SubState == 0)
		{
			process_RunByAngle(0,-0.3);
			use_pidTheta = 1;
			GetballFlag = 6;
			if (distanceFlag == 6)
			{
				process_SubState += 1;
				process_Count = 0;
			}

		}
	if (process_SubState == 1)
	{
		process_Count++;
		if(process_Count>20){
			u = 0;
			v = 0;
			process_Count = 0;
			process_SubState = 0;
			step ++;
		}
	}
}

void process_Ball_Out_BLUE(){
	if (process_SubState == 0)
		{
			process_RunByAngle(-180,-0.3);
			use_pidTheta = 1;
			GetballFlag = 6;
			if (distanceFlag == 6)
			{
				process_SubState += 1;
				process_Count = 0;
			}

		}
	if (process_SubState == 1)
	{
		process_Count++;
		if(process_Count>20){
			u = 0;
			v = 0;
			process_Count = 0;
			process_SubState = 0;
			step ++;
		}
	}
}
void process_ApproachWall4()
{
	if(process_SubState == 0)
	{
		use_pidTheta = 1;
		process_RunByAngle2(45,0.3);
		if (HAL_GPIO_ReadPin(sensor_3_GPIO_Port, sensor_3_Pin))
		{
			process_SSCheck ++;
		}else{
			process_SSCheck = 0;
		}
		if (process_SSCheck > 5)
		{
			process_SSCheck = 0;
			process_SubState = 1;

		}
	}
	else if(process_SubState == 1)
		{
			process_Count++;
			if (process_Count > 10)
			{
				u = 0;
				v = 0;
				r = 0;
				use_pidTheta = 0;
				r  = 0;
				process_Count = 0;
				process_SubState = 0;
				step+= 1;
			}
		}

}

void process_ApproachWall(uint8_t siloNum)
{

	float targetDistance = 0;
		switch(siloNum){
		case 1:
			targetDistance = SILO_1_DISTANCE;
			break;
		case 2:
			targetDistance = SILO_2_DISTANCE;
			break;
		case 3:
			targetDistance = SILO_3_DISTANCE;
			break;
		case 4:
			targetDistance = SILO_4_DISTANCE;
			break;
		case 5:
			targetDistance = SILO_5_DISTANCE;
			break;
		}

	if(process_SubState == 0)
	{
		use_pidTheta = 1;
//		r = 0;

		process_RunByAngle(45,0.3);
		if (HAL_GPIO_ReadPin(sensor_3_GPIO_Port, sensor_3_Pin))
		{
			process_SSCheck ++;
		}else{
			process_SSCheck = 0;
		}
		if (process_SSCheck > 5)
		{
			process_SSCheck = 0;
			process_SubState = 1;

		}
	}
	else if(process_SubState == 1)
		{
			process_Count++;
			if (process_Count > 20)
			{
				use_pidTheta = 0;
				r  = 0;
				process_Count = 0;
				process_SubState = 2;
				use_DirectVct = 1;
				process_RunByAngle(-38,0);

			}
		}
	else if(process_SubState == 2)
		{
		use_pidTheta = 0;
		r = 0;
			process_Count++;
			if (process_Count > 5)
			{
				use_pidTheta = 1;
				process_Count = 0;
				process_SubState = 3;
				process_RunByAngle(-35,0.12);
			}
		}
	else if(process_SubState == 3)
		{
			if (HAL_GPIO_ReadPin(sensor_7_GPIO_Port, sensor_7_Pin)
					|| distance2 <= targetDistance  - 0.03)
			{
					process_SSCheck = 0;
					process_SubState = 0;
					use_pidTheta = 0;
					r = 0;
					use_DirectVct = 0;
					process_RunByAngle(45,0.05);
					step+= 1;
				}
		}
}

void process_ApproachWall_BLUE(uint8_t siloNum)
{

	float targetDistance = 0;
		switch(siloNum){
		case 1:
			targetDistance = SILO_1_DISTANCE;
			break;
		case 2:
			targetDistance = SILO_2_DISTANCE;
			break;
		case 3:
			targetDistance = SILO_3_DISTANCE;
			break;
		case 4:
			targetDistance = SILO_4_DISTANCE;
			break;
		case 5:
			targetDistance = SILO_5_DISTANCE;
			break;
		}

	if(process_SubState == 0)
	{
		use_pidTheta = 1;
		process_RunByAngle(-225,0.3);
		if (HAL_GPIO_ReadPin(sensor_2_GPIO_Port, sensor_2_Pin))
		{
			process_SSCheck ++;
		}else{
			process_SSCheck = 0;
		}
		if (process_SSCheck > 5)
		{
			process_SSCheck = 0;
			process_SubState = 1;
			use_DirectVct = 1;
//			Reset_MPU_Angle();
		}
	}
	else if(process_SubState == 1)
		{
			process_Count++;
			if (process_Count > 30)
			{
				use_pidTheta = 0;
				r  = 0;
				process_Count = 0;
				process_SubState = 2;
				process_RunByAngle(-160,0);
			}
		}
	else if(process_SubState == 2)
		{
		use_pidTheta = 0;
		r = 0;
			process_Count++;
			if (process_Count > 5)
			{
				use_pidTheta = 1;
				process_Count = 0;
				process_SubState = 3;
				process_RunByAngle(-160,0.12);
			}
		}
	else if(process_SubState == 3)
		{
			if (HAL_GPIO_ReadPin(sensor_4_GPIO_Port, sensor_4_Pin)
					|| distance2 <= targetDistance  - 0.03)
			{
					process_SSCheck = 0;
					process_SubState = 0;
					use_pidTheta = 0;
					r = 0;
					use_DirectVct = 1;
					process_RunByAngle(-225,0.05);
					step++;
				}
		}
}
void process_ApproachWall2()
{
	if(process_SubState == 0)
	{
		use_pidTheta = 1;
		process_RunByAngle2(135,0.2);
		if (HAL_GPIO_ReadPin(sensor_2_GPIO_Port, sensor_2_Pin))
		{
			process_SSCheck ++;
		}else{
			process_SSCheck = 0;
		}
		if (process_SSCheck > 5)
		{
			process_SSCheck = 0;
			process_SubState = 1;

		}
	}
	else if(process_SubState == 1)
		{
			process_Count++;
			if (process_Count > 35)
			{
				process_Count = 0;
				process_SubState = 0;
				step += 1;
				process_RunByAngle(135,0);
//				process_RunByAngle(-40,0.16);
			}
		}
}

void process_ApproachWall3()
{
	if(process_SubState == 0)
	{
		use_pidTheta = 1;
		process_RunByAngle(45,0.2);
		if (HAL_GPIO_ReadPin(sensor_3_GPIO_Port, sensor_3_Pin))
		{
			process_SSCheck ++;
		}else{
			process_SSCheck = 0;
		}
		if (process_SSCheck > 5)
		{
			process_SSCheck = 0;
			process_SubState = 1;

		}
	}
	else if(process_SubState == 1)
		{
			process_Count++;
			if (process_Count > 35)
			{
				process_ResetFloatingEnc();
				use_pidTheta = 0;
				r  = 0;
				process_Count = 0;
				process_SubState = 2;
				use_DirectVct = 1;
				process_RunByAngle(-38,0);
			}
		}
	else if(process_SubState == 2)
		{
			process_Count++;
			if (process_Count > 5)
			{
				use_pidTheta = 1;
				process_Count = 0;
				process_SubState = 3;
				process_RunByAngle(-38,0.12);
			}
		}
	else if(process_SubState == 3)
		{
			if (HAL_GPIO_ReadPin(sensor_7_GPIO_Port, sensor_7_Pin))
			{
					process_SSCheck = 0;
					process_SubState = 0;
					use_pidTheta = 0;
					r = 0;
					process_RunByAngle(45,0.05);
					step++;
			}
			if(floatingEncCount > 1000){
//				process_Accel_FloatingEnc3(130, 0.3, 300, 0.1, 0, 3);
				process_RunByAngle(130,0.12);
				if (floatingEncCount<700){
					process_SSCheck = 0;
					process_SubState = 0;
					use_pidTheta = 0;
					r = 0;
					use_DirectVct = 0;
					process_RunByAngle(45,0.05);
					step++;
				}
			}
		}
}

void process_setVal_PutBall(uint8_t value)
{
	process_GetBall_State =  value;
}

uint8_t Rice;
void process_ReleaseBall()
{
	process_RunByAngle(45,0);
	process_setVal_PutBall(2);
	process_Count++;
	if (process_Count>50){
		process_Count = 0;
		process_setVal_PutBall(3);
		process_SubState = 0;
//		step++;

	}
}

void process_Get_Ball_Fall()
{
	if (process_SubState == 0)
		{

				process_RunByAngle(0,0);

				GetballFlag = 0;
				process_Count += 1;
				if(process_Count>20)
				{
					process_Count = 0;
					process_SubState = 1;
//					GetballFlag = 1;
				}

		}
	if (process_SubState == 1)
	{

			process_RunByAngle(-180,0.15);


			process_Count = 0;
			process_SubState = 2;
			GetballFlag = 10;

	}
	else if(process_SubState == 2)
	{

		use_pidTheta = 1;
		if (distanceFlag == 10)
		{


			process_SubState = 3;
			process_ResetFloatingEnc();
		}
	}
	else if(process_SubState == 3)
		{

			process_RunByAngle(0,0.15);
//			if(floatingEncCount>20)
//			{
//				process_ResetFloatingEnc();
//				process_SubState = 0;
//				u = 0;
//				v = 0;
//				r = 0;
//			}
		process_Count+=1;
		if(process_Count>20)
		{
			process_Count = 0;
			step += 1;
			u = 0;
			v = 0;
			r = 0;
			process_SubState = 0;
			distanceFlag = 0;
			GetballFlag = 0;
			process_ResetFloatingEnc();
		}
		}
}

void process_Get_Ball_Fall_BLUE()
{
	if (process_SubState == 0)
		{

				process_RunByAngle(0,0);

				GetballFlag = 0;
				process_Count += 1;
				if(process_Count>20)
				{
					process_Count = 0;
					process_SubState = 1;
//					GetballFlag = 1;
				}

		}
	if (process_SubState == 1)
	{

			process_RunByAngle(0,0.15);


			process_Count = 0;
			process_SubState = 2;
			GetballFlag = 10;

	}
	else if(process_SubState == 2)
	{

		use_pidTheta = 1;
		if (distanceFlag == 10)
		{


			process_SubState = 3;
			process_ResetFloatingEnc();
		}
	}
	else if(process_SubState == 3)
		{

			process_RunByAngle(-180,0.15);
//			if(floatingEncCount>20)
//			{
//				process_ResetFloatingEnc();
//				process_SubState = 0;
//				u = 0;
//				v = 0;
//				r = 0;
//			}
		process_Count+=1;
		if(process_Count>10)
		{
			process_Count = 0;
			step += 1;
			u = 0;
			v = 0;
			r = 0;
			process_SubState = 0;
			distanceFlag = 0;
			GetballFlag = 0;
			process_ResetFloatingEnc();
		}
		}
}



void process_getBall()
{
	process_setVal_PutBall(0);
	if (PutBall_getFlag()){
		process_setVal_PutBall(1);
		process_ResetFloatingEnc();
		process_SubState = 0;
		step += 1;
	}
}
void process_Error(uint8_t error)
{
	if(error == 1)
	{
		HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, GPIO_PIN_SET);
		HAL_GPIO_TogglePin(Status_GPIO_Port, Status_Pin);
	}
	else if (error == 0)
	{
		HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(Status_GPIO_Port, Status_Pin, GPIO_PIN_RESET);
	}
}
int numOfRow,BallNum,stepAuto;
int colorNum;
void process_AutogetBall()
{
	while(1)
	{
		osDelay(1);
	}
}


////////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////CAM BIEN DO KHOANG CACH/////////////////////////////////////

void readADC(){
	HAL_ADC_Start(&hadc1);
	HAL_ADC_Start(&hadc2);
	HAL_ADC_PollForConversion(&hadc1, 1000);
	HAL_ADC_PollForConversion(&hadc2, 1000);
	adc_val = HAL_ADC_GetValue(&hadc1);
	adc_val2 = HAL_ADC_GetValue(&hadc2);
	count++;
	sum+=adc_val;
	sum2+=adc_val2;
	if(count>50)
	{
		adc_val_Fil=sum/count;
		adc_val_Fil2 = sum2/count;
		sum = 0;
		sum2 = 0;
		count = 0;
	}
	distance = (9.8/3945) * adc_val_Fil - 150 * (9.8/3945) + 0.38;
	distance2 = (9.8/3945) * adc_val_Fil2 - 150 * (9.8/3945) + 0.38;
	distance_Delta = distance - distance_Pre;
	distance_Pre = distance;

	if(GetballFlag == 1)
	{
		if (distance_Delta < -0.035 && distance_Delta >-0.5)
		{
			distanceFlag = 1;
//			distanceRear = distance;
		}
	}
	if(GetballFlag == 10)
	{
		if (absf(distance_Delta) > 0.02 )
		{
			distanceFlag = 10;
//			distanceRear = distance;
		}
	}
	if(GetballFlag == 5)
	{
		if (distance < 0.8)
		{
			distanceFlag = 5;
//			distanceRear = distance;
		}
	}
	if(GetballFlag == 6)
	{
		if (distance > 0.8)
		{
			distanceFlag = 6;
//			distanceRear = distance;
		}
	}
	if(GetballFlag == 2)
	{
		if ((distanceRear - distance) > 0.013)
		{
			distanceFlag = 2;
			distanceCenter = distance;
		}
	}
	if(GetballFlag == 3)
	{
		if (distance < 0.25)
		{
			distanceFlag = 3;
		}
	}
	if(GetballFlag == 4)
	{
		if (distance > 0.15)
		{
			distanceFlag = 4;
		}
	}
	if(GetballFlag == 0)
	{
		distanceFlag = 0;
	}



	HAL_ADC_Stop(&hadc1);
}

void process_WireRelease(bool isUntangleWire){
	for(CAN_DEVICE_ID id = CANCTRL_DEVICE_MOTOR_CONTROLLER_1; id <=CANCTRL_DEVICE_MOTOR_CONTROLLER_3;id++){
		canfunc_SetBoolValue(isUntangleWire,CANCTRL_MODE_UNTANGLE_WIRE);
		while(canctrl_Send(&hcan1, id) != HAL_OK);
	}
}

//////////////////////////////////Ham Xu Ly Bong Mau////////////////////////////////////////////////
HAL_StatusTypeDef UART5_Start_To_Raspberry(){
	if(!Raspberry_Enable) return HAL_ERROR;
	if(HAL_UART_Transmit(&huart5, UARTTX5_Start_Buffer, 2, 100) != HAL_OK)
		return HAL_BUSY;
	Raspberry_Enable = false;
	return HAL_OK;
}

HAL_StatusTypeDef UART5_Is_Received(){
	if(!UART5_IsReceived) return HAL_ERROR;
	Raspberry[0] -= 0x30;
	if(HAL_UART_Transmit(&huart5, UARTTX5_OK_Buffer, 3, 100) != HAL_OK)
		return HAL_BUSY;
	UART5_IsReceived = false;
	return HAL_OK;
}

////////////////////////////////////////////////////////////////////////////////////////////////////

void HAL_FLASH_OperationErrorCallback(uint32_t ReturnValue)
{
	RBFlash_ErrorHandler(ReturnValue);
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
  MX_ADC1_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_USART2_UART_Init();
  MX_UART5_Init();
  MX_ADC2_Init();
  /* USER CODE BEGIN 2 */
  	BrdParam_GetDataFromFlash();

  	colorNum = BrdParam_GetBallSuccess();
  	TargetSilo = BrdParam_GetTargetSilo();
  	HarvestZoneBallPosition_t ballpos = BrdParam_GetTargetBall();
  	column = ballpos.column;
  	row = ballpos.row;

  	if(BrdParam_GetTeamColor() == BLUE_TEAM){
  		HAL_GPIO_WritePin(Status_GPIO_Port, Status_Pin, 1);
  	}
	valve_Init();
	HAL_UART_Receive_IT(&huart3, (uint8_t*) UARTRX3_Buffer, 9);
	HAL_TIM_Base_Start_IT(&htim4);

	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_2);
	HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
	HAL_UART_Receive_DMA(&huart1,(uint8_t*)mpu,10);
	HAL_UART_Receive_IT(&huart5, (uint8_t*) UARTRX5_Buffer, 1);

//	HAL_UARTEx_ReceiveToIdle_DMA(&huart1, uart2_ds, 5);
//	__HAL_DMA_DISABLE_IT(&hdma_usart1_rx,DMA_IT_HT);

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
  osThreadStaticDef(TaskInvKine, InverseKinematic, osPriorityNormal, 0, 256, TaskInvKineBuffer, &TaskInvKineControlBlock);
  TaskInvKineHandle = osThreadCreate(osThread(TaskInvKine), NULL);

  /* definition and creation of TaskCAN */
  osThreadStaticDef(TaskCAN, CAN_Bus, osPriorityAboveNormal, 0, 128, TaskCANBuffer, &TaskCANControlBlock);
  TaskCANHandle = osThreadCreate(osThread(TaskCAN), NULL);

  /* definition and creation of TaskOdometer */
  osThreadDef(TaskOdometer, OdometerHandle, osPriorityNormal, 0, 256);
  TaskOdometerHandle = osThreadCreate(osThread(TaskOdometer), NULL);

  /* definition and creation of taskFlash */
  osThreadDef(taskFlash, TaskFlash, osPriorityIdle, 0, 128);
  taskFlashHandle = osThreadCreate(osThread(taskFlash), NULL);

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
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_11;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 80-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1000-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 80-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 1000-1;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
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
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 80-1;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 1000-1;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */
  HAL_TIM_MspPostInit(&htim5);

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
  * @brief UART5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART5_Init(void)
{

  /* USER CODE BEGIN UART5_Init 0 */

  /* USER CODE END UART5_Init 0 */

  /* USER CODE BEGIN UART5_Init 1 */

  /* USER CODE END UART5_Init 1 */
  huart5.Instance = UART5;
  huart5.Init.BaudRate = 115200;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART5_Init 2 */

  /* USER CODE END UART5_Init 2 */

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
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);
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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, Status_Pin|RobotSignalBtn_VCC_Pin|RobotSignalBtn_GND_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, HC595_CLK_Pin|HC595_RCLK_Pin|HC595_OE_Pin|HC595_DATA_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : Buzzer_Pin */
  GPIO_InitStruct.Pin = Buzzer_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Buzzer_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Status_Pin RobotSignalBtn_VCC_Pin RobotSignalBtn_GND_Pin */
  GPIO_InitStruct.Pin = Status_Pin|RobotSignalBtn_VCC_Pin|RobotSignalBtn_GND_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : HC595_CLK_Pin HC595_RCLK_Pin HC595_OE_Pin HC595_DATA_Pin */
  GPIO_InitStruct.Pin = HC595_CLK_Pin|HC595_RCLK_Pin|HC595_OE_Pin|HC595_DATA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : RobotSignalBtn_RED_Pin RobotSignalBtn_YELLOW_Pin */
  GPIO_InitStruct.Pin = RobotSignalBtn_RED_Pin|RobotSignalBtn_YELLOW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : RobotSignalBtn_BLUE_Pin RobotSignalBtn_GREEN_Pin */
  GPIO_InitStruct.Pin = RobotSignalBtn_BLUE_Pin|RobotSignalBtn_GREEN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : sensor_1_Pin sensor_2_Pin sensor_3_Pin sensor_4_Pin
                           sensor_5_Pin sensor_6_Pin sensor_7_Pin sensor_8_Pin */
  GPIO_InitStruct.Pin = sensor_1_Pin|sensor_2_Pin|sensor_3_Pin|sensor_4_Pin
                          |sensor_5_Pin|sensor_6_Pin|sensor_7_Pin|sensor_8_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */



/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */


uint8_t SetHomeFlag, check;
void RobotSignalButton_PressedCallback(SignalButtonColor color)
{
	if(color == SIGBTN_RED)
	{
		BrdParam_ResetAllDataInFlash();
		BrdParam_SetTeamColor(RED_TEAM);
		BrdParam_SaveDataToFlash();
	}
	else if (color == SIGBTN_BLUE)
	{
		BrdParam_SetTeamColor(BLUE_TEAM);
		BrdParam_SaveDataToFlash();
	}
	else if (color == SIGBTN_YELLOW)
	{
		Retry = RETRY_ENABLE;
	}else if (color == SIGBTN_GREEN)
	{
		Start = START;
	}
}
/**
 * @brief  Function implementing the defaultTask thread.
 * @param  argument: Not used
 * @retval None
 */

uint8_t finalStep;
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
	valve_Output(0,1);
	valve_Output(1,1);
	valve_Output(2,1);
	osDelay(1000);
	valve_Output(0,0);
	valve_Output(1,0);
	valve_Output(2,0);
	RobotSignalButton_RegisterButtonPressedCallback(&RobotSignalButton_PressedCallback);

	/* Infinite loop */
	for (;;) {
		BuzzerBeepProcess();
//		RobotSignalButton_ScanButton();
		osDelay(10);
	}
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_InverseKinematic */

void InvCpltCallback(ModuleID ID, float speed, float angle) {
	CAN_SpeedBLDC_AngleDC speedAngle;
	speedAngle.bldcSpeed = speed;
	speedAngle.dcAngle = angle;
	canfunc_MotorPutSpeedAndAngle(speedAngle);
	while (canctrl_Send(&hcan1, ID) != HAL_OK);
}
/**
 * @brief Function implementing the TaskInvKine thread.
 * @param argument: Not used
 * @retval None
 */

/* USER CODE END Header_InverseKinematic */
void InverseKinematic(void const * argument)
{
  /* USER CODE BEGIN InverseKinematic */
	swer_Init();
	/* Infinite loop */
	for (;;) {

		if(nodeSwerveSetHomeComplete == 14)
		{
			if(xaDay == 0 && Safety_Mode == 0)
			{
				invkine_Implementation(MODULE_ID_3, uControlX, uControlY, uControlTheta, &InvCpltCallback);
				invkine_Implementation(MODULE_ID_1, uControlX, uControlY, uControlTheta, &InvCpltCallback);
				invkine_Implementation(MODULE_ID_2, uControlX, uControlY, uControlTheta, &InvCpltCallback);
			}
			else if (Safety_Mode == 1)
			{
				invkine_Implementation(MODULE_ID_3, 0, 0, 0, &InvCpltCallback);
				invkine_Implementation(MODULE_ID_1, 0, 0, 0, &InvCpltCallback);
				invkine_Implementation(MODULE_ID_2, 0, 0, 0, &InvCpltCallback);
			}
			else if (xaDay == 1)
			{
				process_WireRelease(1);
			}
			else if (xaDay == 2)
			{
				process_WireRelease(0);
				xaDay = 0;
			}
	//
	//
			if (gamepadRxIsBusy) {
				gamepadRxIsBusy = 0;
				HAL_UART_Receive_IT(&huart3, (uint8_t*) UARTRX3_Buffer, 9);
			}
			if ((huart3.Instance->CR1 & USART_CR1_UE) == 0) {
				__HAL_UART_ENABLE(&huart3);
			}

		}
		osDelay(50);
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
	SetHomeFlag = 0;
	osDelay(3000);
	canctrl_RTR_TxRequest(&hcan1, CANCTRL_DEVICE_MOTOR_CONTROLLER_1,
			CANCTRL_MODE_SET_HOME);
	osDelay(1);
	canctrl_RTR_TxRequest(&hcan1, CANCTRL_DEVICE_MOTOR_CONTROLLER_2,
			CANCTRL_MODE_SET_HOME);
	osDelay(1);
	canctrl_RTR_TxRequest(&hcan1, CANCTRL_DEVICE_MOTOR_CONTROLLER_3,
			CANCTRL_MODE_SET_HOME);
	osDelay(1);

	osDelay(500);
	SetHomeFlag = 1;
//	canctrl_RTR_TxRequest(&hcan1, CANCTRL_DEVICE_MOTOR_CONTROLLER_4, CANCTRL_MODE_SET_HOME);
//	osDelay(1);
	uint32_t modeID;
	/* Infinite loop */
	for (;;) {
		if (xTaskNotifyWait(pdFALSE, pdFALSE, &modeID, portMAX_DELAY)) {
			CAN_RxHeaderTypeDef rxHeader = canctrl_GetRxHeader();
			uint32_t targetID = rxHeader.StdId >> CAN_DEVICE_POS;
			if ((modeID == CANCTRL_MODE_SET_HOME
					|| modeID == CANCTRL_MODE_ROBOT_ERROR)
					&& targetID) {
				handleFunctionCAN(modeID, targetID);
			}
			HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
		}
	}
  /* USER CODE END CAN_Bus */
}

/* USER CODE BEGIN Header_OdometerHandle */
/**
 * @brief Function implementing the TaskOdometer thread.
 * @param argument: Not used
 * @retval None
 */
int ss;
float encPutBall;
float testAngle;
uint8_t testball;
uint8_t testSS;

void process_DetectBall()
{
	if (process_SubState == 0)
	{
		 if(putBall_getStopFlag())
		 {
			 process_SubState+=1;
			 Raspberry_Enable = 1;
		 }
	}
	if (process_SubState == 1){
		UART5_Start_To_Raspberry();
		process_SubState += 1;
	}if (process_SubState == 2)
	{
		if((UART5_Is_Received() == HAL_OK))
		{
			process_SubState+=1;
		}
	}
	if (process_SubState == 3)
	{
		if (Raspberry[0] == 3)
		{
			process_setVal_PutBall(5);
			process_Count++;
			if(process_Count>40)
			{
				process_setVal_PutBall(6);
				process_Count = 0;
				process_SubState = 0;
			}
		}
		else{
//						process_ReleaseBall();
			step+= 1;
			process_Count = 0;
			process_SubState = 0;
		}
//
	}
}

void process_SiloScan(int dir)
{
	if(process_SubState==0)
	{
		process_ResetFloatingEnc();
		use_pidTheta = 1;
		process_SubState += 1;
	}else if (process_SubState == 1)
	{
		AngleNow = -135;
		process_RunByAngle2(-135, 0.5);
		if (floatingEncCount>800)
		{
			process_SubState += 1;
			process_ResetFloatingEnc();
			process_Subsubstate = 0;
		}
	}else if (process_SubState==2)
	{
		process_RunByAngle2(135, dir*0.5);
		if (floatingEncCount>2000)
		{
			process_SubState += 1;
			process_ResetFloatingEnc();
			process_Subsubstate = 0;
		}
	}else if (process_SubState==3)
	{

			step += 1;
			u = 0;
			v = 0;
			r = 0;
			process_SubState = 0;
			process_Subsubstate = 0;
	}
}

void FindSilo(uint8_t siloNum, bool mode){
	float targetDistance = 0;
	switch(siloNum){
	case 1:
		targetDistance = SILO_1_DISTANCE;
		break;
	case 2:
		targetDistance = SILO_2_DISTANCE;
		break;
	case 3:
		targetDistance = SILO_3_DISTANCE;
		break;
	case 4:
		targetDistance = SILO_4_DISTANCE;
		break;
	case 5:
		targetDistance = SILO_5_DISTANCE;
		break;

	}

	if(distance2 >= targetDistance + 0.1 && mode == false)
	{
		process_SubState = 0;
		step+=1;
		u = 0;
		v = 0;
		r = 0;

	}
	else if(distance2 <= targetDistance + 0.2 && mode == true)
	{
		process_SubState = 0;
		step+=1;
		u = 0;
		v = 0;
		r = 0;
	}
}


#define RED_RUN 	0
#define BLUE_RUN	1
#define RED_RETRY	2
#define BLUE_RETRY	3

/* USER CODE END Header_OdometerHandle */
void OdometerHandle(void const * argument)
{
  /* USER CODE BEGIN OdometerHandle */
		  /* USER CODE BEGIN OdometerHandle */

			process_Init();
			osDelay(1000);
//			TargetSilo = 2;
//			process_ReadVel_Init();
			/* Infinite loop */
			for (;;) {

	///////////////////////////////////////////////////////////////////////////////////////////////////////
	/*---------------------------------------------------------------------------------------------------
				if(step == 0)// Buoc khoi dong sethome
				{// Reset co cau ve mac dinh
					// if (dieu kien sethome da xong)step = 1;
				}
				else if(step == 2)//	Doi nhan nut de chay
				{
					// if (dieu kien nut nhan duoc nhan)step = 2;
				}
				else if(step == 3)//	Robot chay toi khu vuc lay banh
				{
					// if (s chua toi hoac s nho hon quang duong)process_Accel_FloatingEnc(Angle,maxSpeed,s,accel);
					// if (da toi) DungRobot(); Step = 4;
				}
				else if(step == 4)// Doi Nhan nut de chay tiep
				{
					// if (dieu kien nut nhan duoc nhan)step = 5;
				}
	--------------------------------------------CODE MAU--------------------------------------------------*/
				testSS = HAL_GPIO_ReadPin(sensor_2_GPIO_Port, sensor_2_Pin);
				trajecTheta.t += DELTA_T;
				Get_MPU_Angle();
				angle_Rad = (a_Now/10.0)*M_PI/180.0;


				process_SetFloatingEnc();
				trajecPlan_Cal(&trajecTheta);

				if (use_pidTheta)
				{

					r = -(PID_Cal(&pid_Angle,(float) trajecTheta.xTrajec,(float)angle_Rad)+(float)trajecTheta.xdottraject);

				}

				process_Error(check);
	///////////////////////////////////////////////////CODE O DAY/////////////////////////////////////////////////////


if(Run == 10)
{
				if (step == 49)
				{
					process_getBall();

				}
				if (step == 50)
				{

					process_DetectBall();
				}

}
//


					if (step == 0)
						{	// Ra lenh cho co Cau lay bong di xuong
							process_setVal_PutBall(4);
							step+=1;

						}
					else if (step == 1)
						{	//Ra lenh cho co Cau lay bong di len cham chu U


							if((Team == RED)&&(Retry == RETRY_DISABLE)&&(Start == START))
							{
								Reset_MPU_Angle();
								process_ResetFloatingEnc();
								// Set thong so quy hoach quy dao :
								step = 2;
								Run = RED_RUN;
								column = 0;
								row = 0;
								colorNum = 0;
								TargetSilo = 2;
							}
							if((Team == BLUE)&&(Retry == RETRY_DISABLE)&&(Start == START))
							{
								Reset_MPU_Angle();
								process_ResetFloatingEnc();
								// Set thong so quy hoach quy dao :
								step =2;
								column = 0;
								row = 0;
								colorNum = 0;
								Run = BLUE_RUN;
								TargetSilo = 2;
							}
							if((Team == RED)&&(Retry == RETRY_ENABLE)&&(Start == START))
							{
								Reset_MPU_Angle();
								process_ResetFloatingEnc();
								// Set thong so quy hoach quy dao :
								step = 2;
								Run = RED_RETRY;
//								column -= 1;
								colorNum -= 1;
//								putBall_ResetStopFlag();
							}
							if((Team == BLUE)&&(Retry == RETRY_ENABLE)&&(Start == START))
							{
								Reset_MPU_Angle();
								process_ResetFloatingEnc();
								putBall_ResetStopFlag();
								// Set thong so quy hoach quy dao :
//								column -= 1;
								colorNum -= 1;
								step = 2;
								Run = BLUE_RETRY;
							}

							if((Team == RED)&&(Retry == RETRY_DISABLE))
							{
								process_setVal_PutBall(1);
								process_RunByAngle(-133, 0.001);
							}
							if((Team == BLUE)&&(Retry == RETRY_DISABLE))
							{
								process_setVal_PutBall(1);
								process_RunByAngle(-46, 0.001);
							}
							if((Team == RED)&&(Retry == RETRY_ENABLE))
							{
								AngleNow = -133;
								process_RunByAngle(-133, 0.001);
							}
							if((Team == BLUE)&&(Retry == RETRY_ENABLE))
							{
								AngleNow = -46;
								process_RunByAngle(-46, 0.001);
							}
						}

/////////////////////////////////////////////////////RED TEAM////////////////////////////////////////////
					if(Run == RED_RUN)
					{

						if(step == 2)
						{
							process_setVal_PutBall(1);
							AngleNow = -135;
							process_PD_Critical();
							process_Accel_FloatingEnc3(-135, 1.2, 10000, 0.3, 0, 3,5);
							if(floatingEncCount > 8700)
							{
								step += 1;
								process_SubState = 0;
							}
						}
						else if (step == 3)
						{

							process_Accel_FloatingEnc3(-145, 1.2, 10000, 0.1, 0, 3,5);
							if(floatingEncCount > 2000)
							{
								Reset_MPU_Angle();
							}
							if(floatingEncCount > 3600)
							{
								step += 1;
								process_SubState = 0;
							}
						}
						else if (step == 4)
						{
							process_Accel_FloatingEnc3(-45, 1.2, 20000, 0.1, 0, 3,10);
							if(floatingEncCount > 7600)
							{
//								if(distance2 < 1.4)
//								{
									step += 1;
									process_SubState = 0;
									process_ResetFloatingEnc();
//								}
							}
//							if(distance2<1.6)
//							{
//								process_SubState = 0;
//								step += 1;
//							}
						}
						else if (step == 5)
						{
							process_Accel_FloatingEnc3(-135, 0.8, 5700, 0.1, 0, 2.5,5);


//								if (floatingEncCount > 5000){
//									step += 1;
//									process_SSCheck = 0;
//									process_SubState = 0;
//								}

//								if(HAL_GPIO_ReadPin(sensor_2_GPIO_Port, sensor_2_Pin) )
//								{
//									process_SSCheck++;
//								}else{
//									process_SSCheck = 0;
//								}
//								if (process_SSCheck>1){
//									step += 1;
//									process_SSCheck = 0;
//									process_SubState = 0;
//								}

						}
						else if (step == 6)
						{
//							process_Accel_FloatingEnc3(-135, 1, 300, 0.1, 0, 2.5,5);
							step += 1;


						}
						else if (step == 7)
						{
							process_PD_OnStrainghtPath();
							process_Accel_FloatingEnc4(-135, 1, 0, 0.1, 90, 2.5);
							if(abs(a_Now/10 - 90)<10){
								process_SubState = 0;
								step++;
							}
						}
						else if (step == 8)
						{
							AngleNow = -225;
							process_Accel_FloatingEnc3(-225, 1, 10000, 0.1,90,2.5,5);
							if(HAL_GPIO_ReadPin(sensor_2_GPIO_Port, sensor_2_Pin))
							{
								process_SSCheck++;
							}else{
								process_SSCheck = 0;
							}
							if (process_SSCheck>10){
								step += 1;
								process_SSCheck = 0;
								process_SubState = 0;
							}
						}
						else if (step == 9)
						{
//							AngleNow = -230;
//							process_Accel_FloatingEnc3(-235, 0.8, 3000, 0.1,90,2.5,5);
							float speedTest = 0.7;
									if(floatingEncCount > 1000)
									{
										speedTest = 0.5;
									}
									process_Accel_FloatingEnc3(-240, speedTest, 5000, 0.1,90,2.5,5);

						}
						else if (step == 10)
						{
							AngleNow = 0;
							Reset_MPU_Angle();
							process_setVal_PutBall(1);
							trajecPlan_Reset(&trajecTheta);
							u = 0;
							v = 0;
							r = 0;
							use_pidTheta = 0;
							process_PD_OnStrainghtPath();
							pid_Angle.u = 0;
//							process_Accel_FloatingEnc3(0, 0, 0, 0.1,0,2.5,5);
							process_Count++;
							if(process_Count > 40)
							{
								trajecPlan_Reset(&trajecTheta);
								step+= 1;
								process_SubState = 0;
								process_Count = 0;
								process_SSCheck = 0;
							}
						}
						else if (step == 11)
						{

							AngleNow = -90;
							if (floatingEncCount<1600)
							{
								process_Accel_FloatingEnc3(-90, 0.6, 10000, 0.1,0,2.5,5);
							}

							if ((floatingEncCount>1600)&&(floatingEncCount<1800))
							{
								process_Accel_FloatingEnc3(-90, 0.2, 10000, 0.1,0,2.5,5);
							}

							if (floatingEncCount>1800)
							{
								step = 12;
								process_SubState = 0;
							}
						}
						else if (step == 12)
						{

								process_Accel_FloatingEnc3(0, 0.15, 10000, 0.1,0,2.5,10);

							if (floatingEncCount>300)
							{
								step = 13;
								process_SubState = 0;
							}
						}

						else if (step == 13)
						{
							if(row>2){
								process_Ball_Approach3(0);
							}else{
								process_Ball_Approach3(column);
							}

						}
						else if (step == 14)
						{
							process_getBall();
						}
						else if (step == 15)
						{
//							step = 70;
//							process_DetectBall();
//							process_SubState = 0;
//							Raspberry[0] = 1;
//							step += 1;

							if (((column == 2)&&(row == 0))
								|| ((column == 1)&&(row == 1))
								|| ((column == 3)&&(row == 1))
								|| ((column == 0)&&(row == 2))
								|| ((column == 2)&&(row == 2))
								|| ((column == 1)&&(row == 3)))
							{
								Raspberry[0]  = 1;
							}else{
								Raspberry[0] = 0 ;
							}
							step += 1;
						}
						else if (step == 16)
						{
							if (Raspberry[0] == 0)
							{
								putBall_ResetStopFlag();
								process_SSCheck = 0;
								process_SubState = 0;
								process_Count = 0;
								step = 70;
							}else{
								step = 50;
							}
						}

						else if (step == 50)
						{
							valve_Output(0, 1);
							valve_Output(2, 1);

							AngleNow = 45;
							float speed = 0.8;
							if(floatingEncCount>6000)
							{
								speed = 0.25;
								process_setVal_PutBall(2);
							}
							process_Accel_FloatingEnc3(45, speed, 50000, 0.1,-5,2.5,10);
							if (HAL_GPIO_ReadPin(sensor_3_GPIO_Port, sensor_3_Pin))
							{
								process_SSCheck++;
							}else {
								process_SSCheck = 0;
							}

							if(process_SSCheck>10)
							{
								process_SubState = 0;
								step += 1;
							}
						}
						else if (step == 51)
						{
							// move silo increase (or move left)
//							AngleNow =-45;
							process_Accel_FloatingEnc3(-45, 0.3, 20000, 0.1,0,2.5,10);
							FindSilo(TargetSilo,1);
						}
						else if (step == 52)
						{
//							AngleNow =135;
							process_Accel_FloatingEnc3(125, 0.3, 20000, 0.1,0,2.5,10);
							FindSilo(TargetSilo,0);
						}
						else if (step== 53)
						{
							process_ApproachWall(TargetSilo);
						}
						else if (step == 54)
						{
							valve_Output(0, 0);
							osDelay(1000);

							colorNum+= 1;
							if(colorNum>2)
							{
								colorNum = 0;
								TargetSilo += 1;
							}
							column += 1;
							if(column>3)
							{
								column = 0;
								row+=1;
							}

							if(TargetSilo != 2)
							{
								xaDay = 1;
								osDelay(1000);
								xaDay = 2;
								step+= 1;
							}else {
								step = 58;
							}

						}
						else if (step == 55)
						{
							// move silo increase (or move left)
//							AngleNow =-45;
							process_Accel_FloatingEnc3(-45, 0.3, 20000, 0.1,0,2.5,10);
							FindSilo(2,1);
						}
						else if (step == 56)
						{
//							AngleNow =135;
							process_Accel_FloatingEnc3(135, 0.3, 20000, 0.1,0,2.5,10);
							FindSilo(2,0);
						}
						else if (step == 57)
						{
							process_ApproachWall(2);
						}
						else if (step == 58)
						{
							BrdParam_SetBallSuccess(colorNum);
							BrdParam_SetTargetSilo(TargetSilo);
							BrdParam_SetTargetBall(row, column);
							startToFlash = true;

							Reset_MPU_Angle();
							step += 1;
						}
						else if (step == 59)
						{
							float angleRun = 225;
							if(row<1)
							{
								angleRun = 220;

							}
							AngleNow = angleRun;
							process_Accel_FloatingEnc4(angleRun, 0.6, 10000, 0.1,0,2.5);
						}
						else if(step == 60){
							step = 13;

						}


						else if(step == 70)
						{
							process_setVal_PutBall(2);
							valve_Output(1,0);
							valve_Output(2,0);
							if(putBall_getStopFlag())
							{
								column+= 1;
								if(column>3)
								{
									column = 0;
									row+= 1;
								}

								if (lastRow == row)
								{
									step+=1;
								}else {
									step = 80;
								}
								lastRow = row;
							}

						}
						else if(step == 71)
						{
							BrdParam_SetBallSuccess(colorNum);
							BrdParam_SetTargetSilo(TargetSilo);
							BrdParam_SetTargetBall(row, column);
							startToFlash = true;
							Process_Ball_Continue();
						}else if(step == 72)
						{
							process_getBall();
						}else if(step == 73)
						{

							step = 15;
						}

						else if (step == 80)
						{
							process_Ball_Out();
						}else if (step == 81)
						{
							step = 13;
						}






					}

////////////////////////////////////////////////////BLUE TEAM///////////////////////////////////////////
					if (Run == BLUE_RUN)
					{
						if(step == 2)
						{
							process_setVal_PutBall(1);
							AngleNow = -46;
							process_PD_Critical();
							process_Accel_FloatingEnc3(-46, 1.2, 10000, 0.3, 0, 3,5);
							if(floatingEncCount > 8700)
							{
								step += 1;
								process_SubState = 0;
							}
						}
						else if (step == 3)
						{

							process_Accel_FloatingEnc3(-35, 1.2, 10000, 0.1, 0, 3,5);
							if(floatingEncCount > 2000)
							{
								Reset_MPU_Angle();
							}
							if(floatingEncCount > 3600)
							{
								step += 1;
								process_SubState = 0;
							}
						}
						else if (step == 4)
						{
							process_Accel_FloatingEnc3(-135, 1.2, 10000, 0.1, 0, 3,10);
							if(floatingEncCount > 7800)
							{
								step += 1;
								process_SubState = 0;
								process_ResetFloatingEnc();
	//							if(distance2<1.6)
	//							{
	//								process_SubState = 0;
	//								step += 1;
	//							}
							}
						}
						else if (step == 5)
						{
							process_Accel_FloatingEnc3(-45, 0.8, 5700, 0.1, 0, 2.5,5);


//								if (floatingEncCount > 5200){
//								if(HAL_GPIO_ReadPin(sensor_3_GPIO_Port, sensor_3_Pin) )
//								{
//									process_SSCheck++;
//								}else{
//									process_SSCheck = 0;
//								}
//								if (process_SSCheck>1){
//									step += 1;
//									process_SSCheck = 0;
//									process_SubState = 0;
//								}
//							}

						}
						else if (step == 6)
						{
//							process_Accel_FloatingEnc3(-45, 1, 3000, 0.1, 0, 2.5,5);
							step += 1;

						}
						else if (step == 7)
						{
							process_PD_OnStrainghtPath();
							process_Accel_FloatingEnc4(45, 1, 0, 0.1, -90, 2.5);
							if(abs(a_Now/10 - 90)<10){
								process_SubState = 0;
								step++;
							}
						}
						else if (step == 8)
						{
							AngleNow = 47;
							process_Accel_FloatingEnc3(47, 1, 10000, 0.1,-90,2.5,5);
							if(HAL_GPIO_ReadPin(sensor_3_GPIO_Port, sensor_3_Pin))
							{
								process_SSCheck++;
							}else{
								process_SSCheck = 0;
							}
							if (process_SSCheck>10){
								step += 1;
								process_SSCheck = 0;
								process_SubState = 0;
							}
						}
						else if (step == 9)
						{

					//		u = 0;
					//		v = 0;
					//		r = 0;
							float speedTest = 0.7;
							if(floatingEncCount > 1000)
							{
								speedTest = 0.5;
							}
							process_Accel_FloatingEnc3(50, speedTest, 3000, 0.1,-90,2.5,5);

						}
						else if (step == 10)
							{
								AngleNow = 0;
								Reset_MPU_Angle();
								process_setVal_PutBall(1);
								trajecPlan_Reset(&trajecTheta);
								u = 0;
								v = 0;
								r = 0;
								use_pidTheta = 0;
								process_PD_OnStrainghtPath();
								pid_Angle.u = 0;
					//							process_Accel_FloatingEnc3(0, 0, 0, 0.1,0,2.5,5);
								process_Count++;
								if(process_Count > 20)
								{
									trajecPlan_Reset(&trajecTheta);
									step+= 1;
									process_SubState = 0;
									process_Count = 0;
									process_SSCheck = 0;
								}
							}
							else if (step == 11)
							{

								AngleNow = -90;
								if (floatingEncCount<1600)
								{
									process_Accel_FloatingEnc3(-90, 0.6, 10000, 0.1,0,2.5,5);
								}

								if ((floatingEncCount>1600)&&(floatingEncCount<1800))
								{
									process_Accel_FloatingEnc3(-90, 0.2, 10000, 0.1,0,2.5,5);
								}

								if (floatingEncCount>1800)
								{
									step = 12;
									u = 0;
									v = 0;
									r = 0;
									process_SubState = 0;
								}
							}
							else if (step == 12)
							{

									process_Accel_FloatingEnc3(-180, 0.15, 10000, 0.1,0,2.5,10);

								if (floatingEncCount>500)
								{
									step = 13;
									process_SubState = 0;
								}
							}
							else if (step == 13)
							{
								if(row > 2)
								process_Ball_Approach_BLUE(0);
								else
								process_Ball_Approach_BLUE(column);
							}else if (step == 14)
							{
								process_getBall();
							}
							else if (step == 15)
							{
//								process_DetectBall();
								if (((column == 1)&&(row == 0))
									|| ((column == 0)&&(row == 1))
									|| ((column == 2)&&(row == 1))
									|| ((column == 1)&&(row == 2))
									|| ((column == 3)&&(row == 2))
									|| ((column == 2)&&(row == 3)))
								{
									Raspberry[0]  = 1;
								}else{
									Raspberry[0] = 0 ;
								}
								step += 1;
//								step = 50;
							}
							else if (step == 16)
							{
								if (Raspberry[0] == 0)
								{
									putBall_ResetStopFlag();
									process_SSCheck = 0;
									process_SubState = 0;
									process_Count = 0;
									step = 70;
								}else{
									step = 50;
								}
							}
							else if (step == 50)
							{
								valve_Output(1, 1);
								valve_Output(2, 0);

								AngleNow = -225;
								float speed = 0.8;
								if(floatingEncCount>6000)
								{
									speed = 0.25;
									process_setVal_PutBall(2);
								}
								process_Accel_FloatingEnc3(-225, speed, 50000, 0.1,-5,2.5,10);
								if (HAL_GPIO_ReadPin(sensor_2_GPIO_Port, sensor_2_Pin))
								{
									process_SSCheck++;
								}else {
									process_SSCheck = 0;
								}

								if(process_SSCheck>10)
								{
									process_SubState = 0;
									step += 1;
								}
							}
							else if (step== 51)
								{
									process_Accel_FloatingEnc3(-135, 0.3, 20000, 0.1,0,2.5,10);
									FindSilo(TargetSilo,1);
								}
							else if (step == 52)
							{
	//							AngleNow =135;
								process_Accel_FloatingEnc3(45, 0.3, 20000, 0.1,0,2.5,10);
								FindSilo(TargetSilo,0);
							}

							else if (step== 53)
							{
								process_ApproachWall_BLUE(TargetSilo);
							}
							else if (step == 54)
							{
								valve_Output(1, 0);
								osDelay(1000);

								colorNum+= 1;
								if(colorNum>2)
								{
									colorNum = 0;
									TargetSilo += 1;
								}
								column += 1;
								if(column>3)
								{
									column = 0;
									row+=1;
								}

								if(TargetSilo != 2)
								{
									xaDay = 1;
									osDelay(1000);
									xaDay = 2;
									step+= 1;
								}else {
									step = 58;
								}

							}
							else if (step == 55)
							{
								// move silo increase (or move left)
	//							AngleNow =-45;
								process_Accel_FloatingEnc3(-135, 0.3, 20000, 0.1,0,2.5,10);
								FindSilo(2,1);
							}
							else if (step == 56)
							{
	//							AngleNow =135;
								process_Accel_FloatingEnc3(45, 0.3, 20000, 0.1,0,2.5,10);
								FindSilo(2,0);
							}
							else if (step == 57)
							{
								process_ApproachWall(2);
							}
							else if (step == 58)
							{
								Reset_MPU_Angle();
								step += 1;
							}
							else if (step == 59)
							{
								float angleRun = 135;
								if(row<1)
								{
									angleRun = 130;

								}
								AngleNow = angleRun;
								process_Accel_FloatingEnc4(angleRun, 0.6, 10000, 0.1,0,2.5);
							}
							else if(step == 60){
								step = 13;

							}

							else if(step == 70)
							{
								process_setVal_PutBall(2);
								valve_Output(0,0);
								valve_Output(2,1);
								if(putBall_getStopFlag())
								{
									column+= 1;
									if(column>3)
									{
										column = 0;
										row+= 1;
									}

									if (lastRow == row)
									{
										step+=1;
									}else {
										step = 80;
									}
									lastRow = row;
								}
							}else if(step == 71)
							{
								Process_Ball_Continue_BLUE();
							}else if(step == 72)
							{
								process_getBall();
							}else if(step == 73)
							{

								step = 15;
							}

							else if (step == 80)
							{
								process_Ball_Out();
							}else if (step == 81)
							{
								step = 13;
							}


						}


					if (Run == RED_RETRY)
					{


//						if(step == 2)
//						{
//							process_Get_Ball_Fall();
//						}else if(step == 3)
//						{
////							process_ApproachWall_BLUE(3);
//							process_getBall();
//						}
//						if (step == 2)
//						{
//							process_Accel_FloatingEnc4(0, 0, 0, 0, 90, 3);
//						}
//						else if (step == 3)
//						{
//							AngleNow = 0;
//							Reset_MPU_Angle();
//							process_setVal_PutBall(1);
//							trajecPlan_Reset(&trajecTheta);
//							u = 0;
//							v = 0;
//							r = 0;
//							use_pidTheta = 0;
//							process_PD_OnStrainghtPath();
//							pid_Angle.u = 0;
//				//							process_Accel_FloatingEnc3(0, 0, 0, 0.1,0,2.5,5);
//							process_Count++;
//							if(process_Count > 20)
//							{
//								step+= 1;
//								process_SubState = 0;
//								process_Count = 0;
//								process_SSCheck = 0;
//							}
//						}
//						else if (step == 4)
//						{
//							process_Accel_FloatingEnc3(0, 0.3, 2000, 0.05, 0, 1, 5);
//						}

//						if(step == 2)
//						{
//							process_ApproachWall(3);
//						}
//						else if (step == 3)
//						{
//							Reset_MPU_Angle();
//							process_RunByAngle(0,0);
////							osDelay(1000);
//							process_Count++;
//							if(process_Count>5)
//							{
//								step = 4;
//								process_SubState = 0;
//								use_DirectVct = 0;
//							}
//						}
//						else if (step == 4)
//						{
//							process_Accel_FloatingEnc3(-135, 0.3, 5000, 0.1, 0, 1, 5);
//						}
//						if(step == 2)
//						{
//							process_RunByAngle(0,0.15);
//							step += 1;
//						}
//						if(step == 3)
//						{
////							process_ApproachWall_BLUE();
//							process_Ball_Approach3(1);
//						}
//						else if (step == 4)
//						{
//							process_getBall();
//						}

//						if(step == 2)
//						{
//							process_setVal_PutBall(2);
//							valve_Output(1,0);
//							valve_Output(2,0);
//							if(putBall_getStopFlag())
//							{
//								step += 1;
//							}
//						}else if(step == 3)
//						{
//							Process_Ball_Continue();
//						}else if(step == 4)
//						{
//							process_getBall();
//						}else if(step == 5)
//						{
//
//							if(!HAL_GPIO_ReadPin(sensor_8_GPIO_Port, sensor_8_Pin))
//							{
//								step = 2;
//							}else{
//								step = 10;
//
//							}
////							step = 2;
//						}
//
//						if(step == 10)
//						{
//							process_Get_Ball_Fall();
//						}else if (step == 11)
//						{
//							process_getBall();
//						}else if (step == 12)
//						{
//							step = 5;
//						}

						if(step == 2)
						{
//							process_setVal_PutBall(1);
							process_Accel_FloatingEnc3(-135, 1, 10000, 0.1,0,2.5,5);
							if(floatingEncCount>200)
							{
								process_SubState = 0;
								step += 1;
							}

						}
						if(step == 3)
						{
							process_Accel_FloatingEnc3(-45, 1, 10000, 0.1,0,2.5,5);
							if(floatingEncCount>7900)
							{
								process_SubState = 0;
								step += 1;
							}
//							if(distance2<1.6)
//							{
//								process_SubState = 0;
//								step += 1;
//							}

						}else if (step == 4)
						{
							Run = RED_RUN;
							step = 5;
						}
					}

					if (Run == BLUE_RETRY)
						{
//
//						if(step == 2)
//						{
//							process_setVal_PutBall(2);
//							valve_Output(1,0);
//							valve_Output(2,0);
//							if(putBall_getStopFlag())
//							{
//								step += 1;
//							}
//						}else if(step == 3)
//						{
//							Process_Ball_Continue_BLUE();
//						}else if(step == 4)
//						{
//							process_getBall();
//						}else if(step == 5)
//						{
//
//							if(!HAL_GPIO_ReadPin(sensor_8_GPIO_Port, sensor_8_Pin))
//							{
//								step = 2;
//							}else{
//								step = 10;
//
//							}
////							step = 2;
//						}
//
//						if(step == 10)
//						{
//							process_Get_Ball_Fall_BLUE();
//						}else if (step == 11)
//						{
//							process_getBall();
//						}else if (step == 12)
//						{
//							step = 5;
//						}

//						if(step == 2)
//						{
//							process_ApproachWall_BLUE(3);
//						}
//						else if(step == 3)
//						{
////							process_ApproachWall_BLUE(3);
//							process_getBall();
//						}
//
//						if(step == 2)
//						{
//							process_RunByAngle(-180,0.15);
//							step += 1;
//						}
//						if(step == 3)
//						{
////							process_ApproachWall_BLUE();
//							process_Ball_Approach_BLUE(1);
//						}
//						else if (step == 4)
//						{
//							process_getBall();
//						}

//						if(step == 4)
//						{
//							Process_Ball_Continue_BLUE();
//						}

//						if(step == 2)
//						{
//							process_setVal_PutBall(2);
//							valve_Output(1,0);
//							valve_Output(2,0);
//							if(putBall_getStopFlag())
//							{
//
//								step += 1;
//							}
//						}else if(step == 3)
//						{
//							Process_Ball_Continue_BLUE();
//						}else if(step == 4)
//						{
//							process_getBall();
//						}else if(step == 5)
//						{
//
//							step = 2;
//						}

							if(step == 2)
							{
//								process_setVal_PutBall(1);
								process_Accel_FloatingEnc3(-45, 1, 10000, 0.1,0,2.5,5);
								if(floatingEncCount>200)
								{
									process_SubState = 0;
									step += 1;
								}

							}
							if(step == 3)
							{
								process_Accel_FloatingEnc3(-135, 1, 10000, 0.1,0,2.5,5);
								if(floatingEncCount>7800)
								{
									process_SubState = 0;
									step += 1;
								}
	//							if(distance2<1.6)
	//							{
	//								process_SubState = 0;
	//								step += 1;
	//							}

							}else if (step == 4)
							{
								Run = BLUE_RUN;
								step = 5;
							}
						}



					if(!use_DirectVct)
					{
						process_Signal_RotationMatrixTransform(u, v, r);
					}else{
							uControlX = u;
							uControlY = v;
							r = 0;
					}

	////////////////////////////////////////////////////////////////////////////////////////////////////////////

				osDelay(DELTA_T*1000);

					}
  /* USER CODE END OdometerHandle */
}

/* USER CODE BEGIN Header_TaskFlash */
/**
* @brief Function implementing the taskFlash thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_TaskFlash */
void TaskFlash(void const * argument)
{
  /* USER CODE BEGIN TaskFlash */
  /* Infinite loop */
  for(;;)
  {
	  if(startToFlash){
		  HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, 1);
		  BrdParam_SaveDataToFlash();
		  HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, 0);
		  startToFlash = false;
	  }
	  if(resetAllDataFlash){
		  HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, 1);
		  BrdParam_ResetAllDataInFlash();
		  HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, 0);
		  resetAllDataFlash = false;
	  }
	  osDelay(1);
  }
  /* USER CODE END TaskFlash */
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
  if (htim->Instance == TIM4) {
    readADC();
    startPutBall(process_GetBall_State);

  }
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
