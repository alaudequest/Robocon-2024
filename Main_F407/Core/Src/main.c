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
#include "RB1ActuatorValve.h"
#include "MainF4Robot1App.h"
#include "ActuatorGun.h"
#include "Robot1_Sensor.h"
#include "PositionControl.h"
#include "Encoder.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum MainEvent {
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
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim9;
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
osThreadId TaskProcessHandle;
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
uint8_t encEnb, encDis;
CAN_SpeedBLDC_AngleDC nodeSpeedAngle[3] = { 0 };

uint8_t UARTRX3_Buffer[9];
uint8_t DataTayGame[9];

float Xleft, Yleft;
float Xright;

_GamePad GamePad;
uint32_t gamepadRxIsBusy = 0;

float DeltaYR, DeltaYL, DeltaX;
uint8_t Run, resetParam, breakProtect;
float uControlX, uControlY, uControlTheta;
uint8_t stateRun = 0, steadycheck;
uint8_t xaDay;
uint8_t ssCheck, stateChange, rst, Gamepad;
uint8_t numOfBall = 0;

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
uint8_t use_pidTheta;
float TargetTheta, TargetTheta_Degree;
//////////Floating Enc///////
uint8_t reset_ENC;
Encoder_t FloatingEnc;
int floatingEncCount;

//////////Control Signal/////
float u, v, r;

//////////Process ///////////
uint8_t RunProcess;
float chasis_Vector_TargetSpeed;
uint8_t angle_Accel_Flag;
float PreTargetAngle;
float process_AutoChose;
float process_AutoChose_Count;
float process_SubState;
uint8_t process_GetBall_State;
uint8_t process_SSCheck;
uint8_t process_Count;

///////////////////////////////////////Quy Hoach Quy Dao///////////////////////////////////////////
trajec_Param trajecTheta;
//////////////////////////////////////////////////////////////////////////////////////////////////////////

uint32_t nodeSwerveSetHomeComplete = 0;
int gunCount1, gunCount2;
float gunCurrentSpeed1, gunCurrentSpeed2, gunTargetSpeed1, gunTargeSpeed2;
QueueHandle_t qShoot;
bool testTick = false;

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
static void MX_DMA_Init(void);
static void MX_CAN1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM10_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM9_Init(void);
void StartDefaultTask(void const * argument);
void InverseKinematic(void const * argument);
void CAN_Bus(void const * argument);
void Actuator(void const * argument);
void OdometerHandle(void const * argument);
void TaskRunProcess(void const * argument);

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
	HAL_CAN_ActivateNotification(&hcan1,
	CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_RX_FIFO1_MSG_PENDING);
	canctrl_Filter_Mask16(&hcan1, CANCTRL_MODE_SET_HOME << 5,
			CANCTRL_MODE_NODE_REQ_SPEED_ANGLE << 5,
			CANCTRL_MODE_SET_HOME << 5,
			CANCTRL_MODE_NODE_REQ_SPEED_ANGLE << 5, 0,
			CAN_RX_FIFO0);
}

void setHomeComplete() {

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
		case CANCTRL_MODE_UNTANGLE_WIRE:
			canfunc_SetBoolValue(1, CANCTRL_MODE_UNTANGLE_WIRE);
			while (canctrl_Send(&hcan1, targetID) != HAL_OK);
			break;
		default:
			break;
	}

}

void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan) {
	while (1);
}
/*=============================== UART ===============================*/
uint8_t YawHandle;
uint8_t AngleData[5];
int CurrAngle;

char ds[12];
uint8_t uart2_ds[5], ds_ind, ds_cnt, ds_flg;
uint8_t UARTRX3_Buffer[9];
bool phatHienLuaTrai = false;
bool phatHienLuaPhai = false;
bool phatHienBongTrai = false;
bool phatHienBongPhai = false;
void Receive(uint8_t *DataArray) {
	uint8_t *pInt = NULL;
	if (DataArray[4] == 13) {
		pInt = (uint8_t*) &CurrAngle;
		for (uint8_t i = 0; i < 4; i++) {
			*(pInt + i) = DataArray[i];
		}
	}
	memset(DataArray, 0, 5);
	YawHandle = 1;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	appintf_ReceiveDataInterrupt(huart);
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

		}
		else {
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
/*=============================== GPIO EXTI ===============================*/
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {

	if (GPIO_Pin == Enc1A_Pin) {
		if (HAL_GPIO_ReadPin(Enc1B_GPIO_Port, Enc1B_Pin)) {
			gunCount1--;
		}
		else
			gunCount1++;
		return;
	}
	if (GPIO_Pin == Enc2A_Pin) {
		if (HAL_GPIO_ReadPin(Enc2B_GPIO_Port, Enc2B_Pin)) {
			gunCount2--;
		}
		else
			gunCount2++;
		return;
	}
	RB1_WaitSensorInInterrupt(GPIO_Pin);
}

float absf(float num)
{
	if (num >= 0)
		return num;
	else
		return num * -1;
}
///////////////////////////MPU//////////////////////////
#define DELTA_T 0.05
void Get_MPU_Angle()
{
	send_mpu = 'z';
	HAL_UART_Transmit(&huart1, &send_mpu, 1, 1);
	angle = mpu[0] << 8 | mpu[1];
	a_Now = angle;
}

void Reset_MPU_Angle()
{
	send_mpu = 'a';
	HAL_UART_Transmit(&huart1, &send_mpu, 1, 1);

}
/////////////////////Process///////////////////
void process_Init()
{
	////////PID/////////
	pid_Angle.kP = 0.5;
	pid_Angle.kI = 0;
	pid_Angle.kD = 0;
	pid_Angle.alpha = 0;
	pid_Angle.deltaT = DELTA_T;
	pid_Angle.u_AboveLimit = 5;
	pid_Angle.u_BelowLimit = -5;
	pid_Angle.kB = 1 / DELTA_T;

	process_AutoChose = 0;

	encoder_Init(&FloatingEnc, &htim1, 200, DELTA_T);
//	encoder_Init(&FloatingEnc, &htim2, 200, DELTA_T);

}

void process_PD_OnStrainghtPath()
{
	pid_Angle.kP = 1.2;
	pid_Angle.kI = 0;
	pid_Angle.kD = 0;
	pid_Angle.alpha = 0;
	pid_Angle.deltaT = DELTA_T;
	pid_Angle.u_AboveLimit = 5;
	pid_Angle.u_BelowLimit = -5;
	pid_Angle.kB = 1 / DELTA_T;
}

void process_PD_OnTrajecPath()
{
	pid_Angle.kP = 0.6;
	pid_Angle.kI = 0;
	pid_Angle.kD = 0;
	pid_Angle.alpha = 0;
	pid_Angle.deltaT = DELTA_T;
	pid_Angle.u_AboveLimit = 5;
	pid_Angle.u_BelowLimit = -5;
	pid_Angle.kB = 1 / DELTA_T;
}

void process_PD_Auto_Chose(float Target, float Current)
{
	if (process_AutoChose == 0)
			{
		if (absf(Target - Current) < 8 * M_PI / 180)
				{
			process_AutoChose_Count++;
		}
		else {
			process_AutoChose_Count = 0;
		}
	}

	if (process_AutoChose_Count > 15)
			{
		process_AutoChose_Count = 0;
		process_AutoChose = 1;
	}

	if (process_AutoChose == 1)
			{
		process_PD_OnStrainghtPath();
	}
	else {
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

void process_Accel_FloatingEnc3(float Angle, float maxSpeed, float s, float accel, float TargetAngle, float RotateTime)
{
	if (process_SubState == 0)
			{
		trajecPlan_SetParam(&trajecTheta, angle_Rad, TargetAngle * M_PI / 180, RotateTime, 0, 0);
		process_ResetFloatingEnc();
		use_pidTheta = 1;
		process_SubState = 1;
	}
	else {

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
			chasis_Vector_TargetSpeed -= accel;
		}
			if ((chasis_Vector_TargetSpeed <= 0) || (floatingEncCount > s))
			{



				chasis_Vector_TargetSpeed = 0;
				process_ResetFloatingEnc();
				r = 0;
				u = 0;
				v = 0;
				use_pidTheta = 0;
				process_SubState = 0;
				step += 1;

		}
		else {
			u = cos(Angle * M_PI / 180) * chasis_Vector_TargetSpeed;
			v = sin(Angle * M_PI / 180) * chasis_Vector_TargetSpeed;

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
			chasis_Vector_TargetSpeed -= accel;
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

void process_Accel_FloatingEnc5(float Angle, float maxSpeed, float s, float accel, float TargetAngle, float RotateTime)
{
	if (process_SubState == 0)
			{
		trajecPlan_SetParam(&trajecTheta, angle_Rad, TargetAngle * M_PI / 180, RotateTime, 0, 0);
		process_ResetFloatingEnc();
		use_pidTheta = 1;
		process_SubState = 1;
	}
	else {

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
			chasis_Vector_TargetSpeed -= accel;
		}if(floatingEncCount > (s - 150)) {
			use_pidTheta = 0;
			r = 0;
		}
		if ((chasis_Vector_TargetSpeed <= 0) || (floatingEncCount > s))
		{

			chasis_Vector_TargetSpeed = 0;
			process_ResetFloatingEnc();
			r = 0;
			u = 0;
			v = 0;
			use_pidTheta = 0;
			process_SubState = 0;
			step += 1;

		}
		else {
			u = cos(Angle * M_PI / 180) * chasis_Vector_TargetSpeed;
			v = sin(Angle * M_PI / 180) * chasis_Vector_TargetSpeed;

		}
	}

}

void process_RunByAngle(float Angle, float speed)
{
	u = cos(Angle * M_PI / 180) * speed;
	v = sin(Angle * M_PI / 180) * speed;
}

void process_Signal_RotationMatrixTransform(float u, float v, float r)
{
	uControlX = u * cos(angle_Rad) - v * sin(angle_Rad);
	uControlY = u * sin(angle_Rad) + v * cos(angle_Rad);
	uControlTheta = r;
}

void process_Error(uint8_t error)
{
	if (error == 1)
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

void process_WireRelease() {
	handleFunctionCAN(CANCTRL_MODE_UNTANGLE_WIRE, CANCTRL_DEVICE_MOTOR_CONTROLLER_1);
	osDelay(1);
	handleFunctionCAN(CANCTRL_MODE_UNTANGLE_WIRE, CANCTRL_DEVICE_MOTOR_CONTROLLER_2);
	osDelay(1);
	handleFunctionCAN(CANCTRL_MODE_UNTANGLE_WIRE, CANCTRL_DEVICE_MOTOR_CONTROLLER_3);
	osDelay(1);
}

void process_PhatHienLuaTrai() {
	phatHienLuaTrai = true;
}

void process_PhatHienLuaPhai() {
	phatHienLuaPhai = true;
}
bool process_ResetToaDo() {

	bool gapLuaThanhCong = false;
//	uint8_t soLanDoc = 10;
	// sau khi đ�?c tín hiệu ngắt cả 2 cảm biến
//		uint16_t soLanPhatHienLuaTrai = 0;
//		uint16_t soLanPhatHienLuaPhai = 0;
		Sensor_t camBienLuaTrai = RB1_GetSensor(RB1_SENSOR_ARM_LEFT);
		Sensor_t camBienLuaPhai = RB1_GetSensor(RB1_SENSOR_ARM_RIGHT);
		//đ�?c liên tục 2000 lần ở cả 2 cảm biến để chắc chắn không có nhiễu
//		for (uint16_t i = 0; i < soLanDoc; i++) {
//			if (HAL_GPIO_ReadPin(camBienLuaTrai.sensorPort, camBienLuaTrai.sensorPin)) {
//				soLanPhatHienLuaTrai++;
//			}
//			else
//				soLanPhatHienLuaTrai = 0;
//			if (HAL_GPIO_ReadPin(camBienLuaPhai.sensorPort, camBienLuaPhai.sensorPin)) {
//				soLanPhatHienLuaPhai++;
//			}
//			else
//				soLanPhatHienLuaPhai = 0;
//		}
//		if (soLanPhatHienLuaTrai > (soLanDoc - 8) && soLanPhatHienLuaPhai > (soLanDoc - 8)) {
//			valve_BothCatch();
//			gapLuaThanhCong = true;
//		}
		if (HAL_GPIO_ReadPin(camBienLuaTrai.sensorPort, camBienLuaTrai.sensorPin)) {
//			valve_BothCatch();
			gapLuaThanhCong = true;
		}

	return gapLuaThanhCong;
}

bool process_ThucHienGapLua() {

	bool gapLuaThanhCong = false;
//	uint8_t soLanDoc = 10;
	// sau khi đ�?c tín hiệu ngắt cả 2 cảm biến
//		uint16_t soLanPhatHienLuaTrai = 0;
//		uint16_t soLanPhatHienLuaPhai = 0;
		Sensor_t camBienLuaTrai = RB1_GetSensor(RB1_SENSOR_ARM_LEFT);
		Sensor_t camBienLuaPhai = RB1_GetSensor(RB1_SENSOR_ARM_RIGHT);
		//đ�?c liên tục 2000 lần ở cả 2 cảm biến để chắc chắn không có nhiễu
//		for (uint16_t i = 0; i < soLanDoc; i++) {
//			if (HAL_GPIO_ReadPin(camBienLuaTrai.sensorPort, camBienLuaTrai.sensorPin)) {
//				soLanPhatHienLuaTrai++;
//			}
//			else
//				soLanPhatHienLuaTrai = 0;
//			if (HAL_GPIO_ReadPin(camBienLuaPhai.sensorPort, camBienLuaPhai.sensorPin)) {
//				soLanPhatHienLuaPhai++;
//			}
//			else
//				soLanPhatHienLuaPhai = 0;
//		}
//		if (soLanPhatHienLuaTrai > (soLanDoc - 8) && soLanPhatHienLuaPhai > (soLanDoc - 8)) {
//			valve_BothCatch();
//			gapLuaThanhCong = true;
//		}
		if (HAL_GPIO_ReadPin(camBienLuaPhai.sensorPort, camBienLuaPhai.sensorPin)) {
//			valve_BothCatch();
			gapLuaThanhCong = true;
		}

	return gapLuaThanhCong;
}

void process_LuaBongTrai() {
	phatHienBongTrai = true;
}
void process_LuaBongPhai() {
	valve_RightCollectBall();
	osDelay(3000);
	valve_RightWaitCollectBall();
}

bool process_ThucHienLuaBongTrai() {
	bool thucHienThanhCong = false;
	// Nếu cảm biến lùa bóng trái phát hiện bóng
	if (phatHienBongTrai) {
		// Rút xilanh để lùa bóng vào
		valve_LeftCollectBall();
		osDelay(3000);
		// �?ẩy xilanh ra và ch�? cảm biến phát hiện
		valve_LeftWaitCollectBall();
		// ch�? bóng bắn xong
		osDelay(2000);
		phatHienBongTrai = false;
		thucHienThanhCong = true;
	}
	return thucHienThanhCong;
}
void process_ResetWallAppRoach()
{
	if(process_SubState == 0)
	{
		use_pidTheta = 1;
		process_RunByAngle(20,0.15);
		if(process_ResetToaDo() == true){
			process_Error(1);

			osDelay(50);
			process_SubState = 1;
		}
	}
	else if(process_SubState == 1){
		process_Error(0);
		process_RunByAngle(90,0.01);
		process_SubState = 2;
	}
	else if(process_SubState == 2){
		process_ResetFloatingEnc();
		Reset_MPU_Angle();
		process_SubState = 0;
		step += 1;
	}
}

void process_RiceAppRoach()
{
	if(process_SubState == 0)
	{
		use_pidTheta = 1;
		process_RunByAngle(20,0.15);
		if(process_ThucHienGapLua() == true){
			process_Error(1);

			osDelay(50);
			process_SubState = 1;
		}
	}
	else if(process_SubState == 1){
		process_Error(0);
		process_RunByAngle(90,0.01);
		process_SubState = 2;
	}
	else if(process_SubState == 2){
		valve_BothCatch();
		process_SubState = 3;
	}
	else if(process_SubState == 3){
		process_ResetFloatingEnc();
		process_SubState = 0;
		step += 1;
	}
}
// Danh cho co cau ban
void Gun_ShootBall(uint16_t Target1)
{
	gun_PIDSpeed1(Target1);
	gun_PIDSpeed2(Target1);
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
  MX_TIM5_Init();
  MX_TIM9_Init();
  /* USER CODE BEGIN 2 */

	HAL_UART_Receive_IT(&huart3, (uint8_t*) UARTRX3_Buffer, 9);
	HAL_UART_Receive_DMA(&huart1, (uint8_t*) mpu, 10);
	HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
	__HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);
	CAN_Init();
	valve_Init();
	gun_Init();
	process_Init();
	RB1_CollectBallMotor_Init();

	RB1_SensorRegisterPin(Sensor7_GPIO_Port, Sensor7_Pin, RB1_SENSOR_ARM_LEFT);
	RB1_SensorRegisterPin(Sensor5_GPIO_Port, Sensor5_Pin, RB1_SENSOR_ARM_RIGHT);
	RB1_SensorRegisterPin(Sensor8_GPIO_Port, Sensor8_Pin, RB1_SENSOR_COLLECT_BALL_LEFT);
	RB1_SensorRegisterPin(Sensor4_GPIO_Port, Sensor4_Pin, RB1_SENSOR_COLLECT_BALL_RIGHT);
	RB1_RegisterSensorCallBack(&process_PhatHienLuaTrai, RB1_SENSOR_ARM_LEFT);
	RB1_RegisterSensorCallBack(&process_PhatHienLuaPhai, RB1_SENSOR_ARM_RIGHT);
	RB1_RegisterSensorCallBack(&process_LuaBongTrai, RB1_SENSOR_COLLECT_BALL_LEFT);
	MainF4Robot1App_Init();

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
  osThreadDef(TaskActuator, Actuator, osPriorityNormal, 0, 128);
  TaskActuatorHandle = osThreadCreate(osThread(TaskActuator), NULL);

  /* definition and creation of TaskOdometer */
  osThreadDef(TaskOdometer, OdometerHandle, osPriorityAboveNormal, 0, 128);
  TaskOdometerHandle = osThreadCreate(osThread(TaskOdometer), NULL);

  /* definition and creation of TaskProcess */
  osThreadDef(TaskProcess, TaskRunProcess, osPriorityNormal, 0, 256);
  TaskProcessHandle = osThreadCreate(osThread(TaskProcess), NULL);

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
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
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
  htim2.Init.Period = 65535;
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
  * @brief TIM9 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM9_Init(void)
{

  /* USER CODE BEGIN TIM9_Init 0 */

  /* USER CODE END TIM9_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM9_Init 1 */

  /* USER CODE END TIM9_Init 1 */
  htim9.Instance = TIM9;
  htim9.Init.Prescaler = 80-1;
  htim9.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim9.Init.Period = 1000-1;
  htim9.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim9.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim9) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim9, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim9) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim9, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim9, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM9_Init 2 */

  /* USER CODE END TIM9_Init 2 */
  HAL_TIM_MspPostInit(&htim9);

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Status_GPIO_Port, Status_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, HC595_CLK_Pin|HC595_RCLK_Pin|HC595_OE_Pin|HC595_DATA_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : Buzzer_Pin */
  GPIO_InitStruct.Pin = Buzzer_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Buzzer_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Status_Pin */
  GPIO_InitStruct.Pin = Status_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Status_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : HC595_CLK_Pin HC595_RCLK_Pin HC595_OE_Pin HC595_DATA_Pin */
  GPIO_InitStruct.Pin = HC595_CLK_Pin|HC595_RCLK_Pin|HC595_OE_Pin|HC595_DATA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : Sensor1_Pin Sensor2_Pin Sensor4_Pin Sensor5_Pin
                           Sensor6_Pin Sensor7_Pin Sensor8_Pin */
  GPIO_InitStruct.Pin = Sensor1_Pin|Sensor2_Pin|Sensor4_Pin|Sensor5_Pin
                          |Sensor6_Pin|Sensor7_Pin|Sensor8_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : Enc2B_Pin */
  GPIO_InitStruct.Pin = Enc2B_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(Enc2B_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Enc2A_Pin */
  GPIO_InitStruct.Pin = Enc2A_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(Enc2A_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Enc1B_Pin */
  GPIO_InitStruct.Pin = Enc1B_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(Enc1B_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Enc1A_Pin */
  GPIO_InitStruct.Pin = Enc1A_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(Enc1A_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void InvCpltCallback(ModuleID ID, float speed, float angle) {
	CAN_SpeedBLDC_AngleDC speedAngle;
	speedAngle.bldcSpeed = speed;
	speedAngle.dcAngle = angle;
	canfunc_MotorPutSpeedAndAngle(speedAngle);
	while (canctrl_Send(&hcan1, ID) != HAL_OK)
		;
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
 * @brief  Function implementing the defaultTask thread.
 * @param  argument: Not used
 * @retval None
 */

/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
	swer_Init();
	osDelay(3000);
	/* Infinite loop */
	for (;;) {
		for (;;) {
			if (xaDay == 0)
					{
				invkine_Implementation(MODULE_ID_3, uControlX, uControlY, uControlTheta, &InvCpltCallback);
				invkine_Implementation(MODULE_ID_1, uControlX, uControlY, uControlTheta, &InvCpltCallback);
				invkine_Implementation(MODULE_ID_2, uControlX, uControlY, uControlTheta, &InvCpltCallback);
			}
			else if (xaDay == 1) {
				process_WireRelease();
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
			osDelay(50);
		}
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
	uint32_t modeID;
	/* Infinite loop */
	for (;;) {
		if (xTaskNotifyWait(pdFALSE, pdFALSE, &modeID, portMAX_DELAY)) {
			CAN_RxHeaderTypeDef rxHeader = canctrl_GetRxHeader();
			uint32_t targetID = rxHeader.StdId >> CAN_DEVICE_POS;
			if ((modeID == CANCTRL_MODE_SET_HOME
					|| modeID == CANCTRL_MODE_NODE_REQ_SPEED_ANGLE)
					&& targetID) {
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
HAL_StatusTypeDef Delay_tick(uint32_t tick) {
	static TickType_t xStartTime = 0;
	if (!xStartTime) {
		xStartTime = xTaskGetTickCount();
	}
	if (xTaskGetTickCount() - xStartTime >= tick) {
		xStartTime = 0;
		return HAL_OK;
	}
	return HAL_BUSY;
}
bool beginToCollectBallLeft = false;
bool beginToCollectBallRight = false;
float TestAngle,TestSpeed;
/* USER CODE END Header_Actuator */
void Actuator(void const * argument)
{
  /* USER CODE BEGIN Actuator */
	gun_Init();
		bool IsShoot = false;
		TickType_t xStartTime = 0, xOccurredTime = 0;
	/* Infinite loop */
	for (;;) {
		if (beginToCollectBallLeft) {
			valve_LeftWaitCollectBall();
			beginToCollectBallLeft = false;
		}
		if (beginToCollectBallRight) {
			valve_RightWaitCollectBall();
			beginToCollectBallRight = false;
		}
		RB1_CollectBallMotor_ControlSpeed();
		RB1_SensorTriggerHandle();
		if (testTick) {
			IsShoot = 1;
			xStartTime = xTaskGetTickCount();
		}
		else {
			IsShoot = 0;
			gun_ResetEncoder(&gunCount1, &gunCount2);
		}
		if (IsShoot) {
			xOccurredTime = xTaskGetTickCount() - xStartTime;
			//		  if(xOccurredTime > 7000/portTICK_PERIOD_MS){
			//			  IsGetBall = 0;
			// SẼ UNCOMMENT HÀNG NÀY SAU KHI CODE XONG	IsShoot = 0;
			//			  xOccurredTime = 0;
			//			  xStartTime = xTaskGetTickCount();
			//		  }else{
			//			  gun_StartGetBall();

			// Nếu tốc hiện tại nh�? hơn target thì gia tốc
			if (gunCurrentSpeed1 < gunTargetSpeed1) {
				if (gunTargetSpeed1 - gunCurrentSpeed1 < 100) {
					gunCurrentSpeed1 = gunTargetSpeed1;
				}
				// �?ang tăng tốc
				else {
					Gun_ShootBall(gunCurrentSpeed1);
					if (Delay_tick(200) == HAL_OK) {
						gunCurrentSpeed1 += 100;
					}
				}

				if (xOccurredTime > 2000 / portTICK_PERIOD_MS) {
					Gun_ShootBall(gunTargetSpeed1);
				}
			}
			// Nếu tốc hiện tại lớn hơn target thì giảm tốc
			else if (gunCurrentSpeed1 > gunTargetSpeed1) {
				// Sau 2s thì xác lập
				// �?ang giảm tốc
				if (gunCurrentSpeed1 - gunTargetSpeed1 < 100) {
					gunCurrentSpeed1 = gunTargetSpeed1;
				}
				else {
					Gun_ShootBall(gunCurrentSpeed1);
					if (Delay_tick(200) == HAL_OK) {
						gunCurrentSpeed1 -= 100;
					}
				}
				if (xOccurredTime > 2000 / portTICK_PERIOD_MS) {
					Gun_ShootBall(gunTargetSpeed1);
				}
			}

		}
		gun_VelCal(gunCount1, gunCount2);
		osDelay(10);
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
void OdometerHandle(void const * argument)
{
  /* USER CODE BEGIN OdometerHandle */

	/* Infinite loop */
	osDelay(1000);

//			process_ReadVel_Init();
	/* Infinite loop */
	for (;;) {
//				ss = HAL_GPIO_ReadPin(sensor_5_GPIO_Port, sensor_5_Pin);
//				encPutBall = encoder_GetPulse(&PutBall_Enc, MODE_X4);
//	///////////////////////////////////////////////////////////////////////////////////////////////////////
//	/*---------------------------------------------------------------------------------------------------
//				if(step == 0)// Buoc khoi dong sethome
//				{// Reset co cau ve mac dinh
//					// if (dieu kien sethome da xong)step = 1;
//				}
//				else if(step == 2)//	Doi nhan nut de chay
//				{
//					// if (dieu kien nut nhan duoc nhan)step = 2;
//				}
//				else if(step == 3)//	Robot chay toi khu vuc lay banh
//				{
//					// if (s chua toi hoac s nho hon quang duong)process_Accel_FloatingEnc(Angle,maxSpeed,s,accel);
//					// if (da toi) DungRobot(); Step = 4;
//				}
//				else if(step == 4)// Doi Nhan nut de chay tiep
//				{
//					// if (dieu kien nut nhan duoc nhan)step = 5;
//				}
//	--------------------------------------------CODE MAU--------------------------------------------------*/
//
		trajecTheta.t += DELTA_T;
		Get_MPU_Angle();
		angle_Rad = (a_Now / 10.0) * M_PI / 180.0;
		process_SetFloatingEnc();
		trajecPlan_Cal(&trajecTheta);
//		process_PD_Auto_Chose(trajecTheta.Pf, angle_Rad);
		if (use_pidTheta)
		{
			r = -(PID_Calculate(&pid_Angle, (float) trajecTheta.xTrajec, (float) angle_Rad) + (float) trajecTheta.xdottraject);

		}
//
//				process_Error(check);
//	///////////////////////////////////////////////////CODE O DAY/////////////////////////////////////////////////////
//
		if (step == 0)
				{	//Ra lenh cho co Cau lay bong di len cham chu U

			if (GamePad.Up)
			{
				osDelay(500);
				if (GamePad.Up)
				{	//Reset thong so enc tha troi va la ban :
					Reset_MPU_Angle();
					process_ResetFloatingEnc();
					// Set thong so quy hoach quy dao :
					step = 1;
				}
			}
		}
		else if (step == 1)
		{
			process_PD_OnStrainghtPath();
			process_Accel_FloatingEnc5(-25, 0.5, 3300, 0.08, 0, 3);
		}
//		else if (step == 2)
//		{
//			process_PD_OnStrainghtPath();
//			process_RiceAppRoach();
//		}
//		else if(step == 3)
//		{
//			process_Accel_FloatingEnc5(-45, 0.5, 9800, 0.08, 90, 3);
//		}
//		else if (step == 4)
//		{
//			process_Accel_FloatingEnc4(180, 0.2, 1200, 0.08, 90, 3);
//		}
//		else if (step == 5)
//		{
//			valve_ArmDownAndHandHold();
//			step += 1;
//		}
//		else if (step == 6)
//		{
//			Gamepad = 1;
//			if (GamePad.Up)
//			{
//				osDelay(500);
//				if (GamePad.Up)
//				{	//Reset thong so enc tha troi va la ban :
////					Reset_MPU_Angle();
//					process_ResetFloatingEnc();
//					// Set thong so quy hoach quy dao :
//
//					Gamepad = 0;
//					step += 1;
//					// tha tay gap
//					process_Error(1);
//					valve_HandRelease();
//					osDelay(50);
//					process_Error(0);
//					osDelay(50);
//					process_Error(1);
//					osDelay(50);
//					valve_ArmUp();
//					osDelay(50);
//					process_Error(0);
//				}
//			}
//		}
//		else if(step == 7 )
//		{
//			process_Accel_FloatingEnc5(132, 0.5, 9600, 0.08, 0, 3);
//		}
//		else if(step == 8)
//		{
//			process_RiceAppRoach();
//		}
//		else if(step == 9)
//		{
//			process_Accel_FloatingEnc5(-70, 0.5, 8300, 0.08, 90, 3);
//		}
//		else if(step == 10)
//		{
//			process_Accel_FloatingEnc4(180, 0.2, 900, 0.08, 90, 3);
//		}
//		else if(step == 11)
//		{
//			valve_ArmDownAndHandHold();
//			step += 1;
//		}
//		else if (step == 12)
//		{
//			Gamepad = 1;
//			if (GamePad.Up)
//			{
//				osDelay(500);
//				if (GamePad.Up)
//				{	//Reset thong so enc tha troi va la ban :
////					Reset_MPU_Angle();
//					process_ResetFloatingEnc();
//					// Set thong so quy hoach quy dao :
//
//					Gamepad = 0;
//					step += 1;
//					// tha tay gap
//					process_Error(1);
//					valve_HandRelease();
//					osDelay(50);
//					process_Error(0);
//					osDelay(50);
//					process_Error(1);
//					osDelay(50);
//					valve_ArmUp();
//					osDelay(50);
//					process_Error(0);
//				}
//			}
//		}
//		else if(step == 13)
//		{
//			process_Accel_FloatingEnc5(100, 0.5, 8600, 0.08, 0, 3);
//		}
//		else if(step == 14)
//		{
//			process_ResetWallAppRoach();
//		}
//		else if(step == 15)
//		{
//			process_Accel_FloatingEnc4(-18, 0.5, 9200, 0.08, 0, 3);
//		}
//		else if (step == 16)
//		{
//			if (GamePad.Up)
//			{
//				osDelay(500);
//				if (GamePad.Up)
//				{
//					beginToCollectBallLeft = true;
//					use_pidTheta = false;
//					r = 0;
//					RB1_CollectBallMotor_On();
//					testTick = 1;
//					gunTargetSpeed1 = 3500;
//					step+=1;
//				}
//
//			}
//
//		}
//		else if(step == 17){
//			Sensor_t collectBallLeft = RB1_GetSensor(RB1_SENSOR_COLLECT_BALL_LEFT);
//			// khi doc duoc cam bien thi ngung chay va dong cua lua
//			use_pidTheta = 1;
//			process_RunByAngle(-90, 0.3);
//			if (HAL_GPIO_ReadPin(collectBallLeft.sensorPort, collectBallLeft.sensorPin)) {
//				numOfBall++;
//				use_pidTheta = 0;
//				Gamepad = 0;
//				u = 0;
//				v = 0;
//				r = 0;
//				step++;
//			}
//		}
//		else if(step == 18){
//			// dong cua lua banh
//			valve_LeftCollectBall();
//			osDelay(1000);
//
//				if(numOfBall < 1){
//					step = 16
//
//
//
//
//
//							;
//					valve_LeftWaitCollectBall();
//				} else {
//					gunTargetSpeed1 = 0;
//					RB1_CollectBallMotor_Off();
//					step +=1;
//
//			}
//
//
//		}


//		else if(step == 14)
//		{
//			use_pidTheta = 1;
//			process_RunByAngle(90, 0.2);
//			process_Count ++;
//			if (process_Count > 35)
//			{
//				Reset_MPU_Angle();
//				use_pidTheta = 0;
//				r = 0;
//				u = 0;
//				v = 0;
//				process_Error(1);
//				osDelay(50);
//				process_Error(0);
//				process_RunByAngle(90, 0);
//				process_Count = 0;
//				step ++;
//			}
//		}

//			process_PD_OnTrajecPath();
//			process_Accel_FloatingEnc3(-45, 0.5, 20000, 0.08, 90, 3);
//			if (floatingEncCount>10000)
//				{
//					process_ResetFloatingEnc();
//					step+=1;
//				}
//		}
//		else if(step == 4)
//		{
//			process_PD_OnStrainghtPath();
//			process_Accel_FloatingEnc4(180, 0.2, 2500, 0.08, 90, 3);
//		}
//		else if(step == 5){
//			// ha canh tay nhung van giu lua
//			valve_ArmDownAndHandHold();
//			step += 1;
//		}
//		else if (step == 6)
//		{
//			Gamepad = 1;
//			if (GamePad.Up)
//			{
//				osDelay(500);
//				if (GamePad.Up)
//				{	//Reset thong so enc tha troi va la ban :
////					Reset_MPU_Angle();
//					process_ResetFloatingEnc();
//					// Set thong so quy hoach quy dao :
//
//					Gamepad = 0;
//					step += 1;
//					// tha tay gap
//					process_Error(1);
//					valve_HandRelease();
//					osDelay(50);
//					process_Error(0);
//					osDelay(50);
//					process_Error(1);
//					osDelay(50);
//					valve_ArmUp();
//					osDelay(50);
//					process_Error(0);
//				}
//			}
//		}
//		else if(step == 7 )
//		{
//			process_Accel_FloatingEnc3(132, 0.5, 9500, 0.08, 0, 3);
//		}
//		else if(step == 8)
//		{
//			process_RiceAppRoach();
//		}
//		else if(step == 8)
//		{
//			process_Accel_FloatingEnc5(-76, 0.4, 8500, 0.08, 90, 2);
//		}
//		else if(step == 9){
//			process_Error(1);
//			osDelay(50);
//			process_Error(0);
//			// ha canh tay nhung van giu lua
//			valve_ArmDownAndHandHold();
//			step++;
//		}
//		else if (step == 10)
//		{
//			Gamepad = 1;
//			if (GamePad.Up)
//			{
//				osDelay(500);
//				if (GamePad.Up)
//				{	//Reset thong so enc tha troi va la ban :
////					Reset_MPU_Angle();
//					process_ResetFloatingEnc();
//					// Set thong so quy hoach quy dao :
//
//					Gamepad = 0;
//					step += 1;
//					// tha tay gap
//					process_Error(1);
//					valve_HandRelease();
//					osDelay(50);
//					process_Error(0);
//					osDelay(450);
//					process_Error(1);
//					osDelay(50);
//					process_Error(0);
//					valve_ArmUp();
//					osDelay(450);
//				}
//			}
//		}
//		else if(step == 11)
//		{
//			process_Accel_FloatingEnc3(95, 0.5, 8000, 0.08, 0, 3);
//		}
//		else if(step == 12)
//		{
//			process_RunByAngle(90, 0.2);
//			process_Count ++;
//			if (process_Count > 35)
//			{
//				process_Error(1);
//				osDelay(50);
//				process_Error(0);
//				process_RunByAngle(90, 0);
//				process_Count = 0;
//				step ++;
//			}
//		}
//		else if(step == 12)
//		{
//			process_Accel_FloatingEnc4(0, 0.5, 8200, 0.08, 0, 3);
//		}
//		else if(step == 13)
//		{
////			process_Accel_FloatingEnc3(0, 0.5, 8200, 0.08, 0, 3);
//			Gamepad = 1;
//			if (GamePad.Up)
//			{
//				osDelay(500);
//				if (GamePad.Up)
//				{
//					beginToCollectBallLeft = true;
////					process_RunByAngle(90,0);
//					use_pidTheta = false;
//					r = 0;
//					RB1_CollectBallMotor_On();
//					testTick = 1;
//					gunTargetSpeed1 = 3500;
//					step++;
//				}
//			}
//		}
//		else if(step == 14){
//			Sensor_t collectBallLeft = RB1_GetSensor(RB1_SENSOR_COLLECT_BALL_LEFT);
//			// khi doc duoc cam bien thi ngung chay va dong cua lua
//			Gamepad = 1;
//			if (HAL_GPIO_ReadPin(collectBallLeft.sensorPort, collectBallLeft.sensorPin)) {
//				numOfBall++;
//				Gamepad = 0;
//				u = 0;
//				v = 0;
//				r = 0;
//				step++;
//			}
//		}
//		else if(step == 15){
//			// dong cua lua banh
//			valve_LeftCollectBall();
//			osDelay(1000);
////			process_Count ++ ;
////			if (process_Count > 1000/50)
////			{
////				process_Count = 0;
//
//				if(numOfBall < 2){
//					step = 14;
//					valve_LeftWaitCollectBall();
//				} else {
//					gunTargetSpeed1 = 0;
//					RB1_CollectBallMotor_Off();
//					step +=1;
//
//			}
//
//
//		}




//		else if(step == 12)
//		{
//			process_Accel_FloatingEnc3(0, 0.5, 8200, 0.08, 0, 3);
//		}
//		else if(step == 13){
//			valve_LeftWaitCollectBall();
//			step++;
//		}
//		else if(step == 14)
//		{
//			process_Accel_FloatingEnc3(-90, 0.1, 2300, 0.08, 0, 3);
//		}
//		else if(){
//			beginToCollectBallLeft = true;
//			step++;
//		}
//		else if(){
//			Sensor_t collectBallLeft = RB1_GetSensor(RB1_SENSOR_COLLECT_BALL_LEFT);
//			// khi doc duoc cam bien thi ngung chay va dong cua lua
//			if (HAL_GPIO_ReadPin(collectBallLeft.sensorPort, collectBallLeft.sensorPin)) {
//				process_RunByAngle(90,0);
//				use_pidTheta = false;
//				r = 0;
//				RB1_CollectBallMotor_On();
//				testTick = 1;
//				gunTargetSpeed1 = 1000;
//				step++;
//			}
//		}
//		else if(){
//			// dong cua lua banh
//			valve_LeftCollectBall();
//			osDelay(3000);
//			gunTargetSpeed1 = 0;
//			RB1_CollectBallMotor_Off();
//
//		}
//	////////////////////////////////////////////////NUT BAM////////////////////////////////////////////////////////////



		if(GamePad.Triangle)
		{
			osDelay(100);
			if(GamePad.Triangle)
			{
				valve_Reset();
			}
		}
		if (GamePad.Down && GamePad.Cross)	//Chuyen Sang Che Do GamePad
				{
			osDelay(100);
			if (GamePad.Down && GamePad.Cross)
					{
				Gamepad = 1;
			}
		}
//
		if ((GamePad.Square == 1) && (GamePad.Right == 1))	// Xa day
				{
			osDelay(100);
			if ((GamePad.Square == 1) && (GamePad.Right == 1))
					{
				xaDay = 1;
			}
		}


//
		if (Gamepad == 1)
		{

			uControlX = -GamePad.XLeftCtr;
			uControlY = GamePad.YLeftCtr;
			uControlTheta = GamePad.XRightCtr;

		}
		else if (Gamepad == 0) {
			process_Signal_RotationMatrixTransform(u, v, r);
		}
//	////////////////////////////////////////////////////////////////////////////////////////////////////////////
////				xTaskNotify(TaskInvKineHandle,1,eSetValueWithOverwrite);
		osDelay(DELTA_T * 1000);

	}
  /* USER CODE END OdometerHandle */
}

/* USER CODE BEGIN Header_TaskRunProcess */
/**
 * @brief Function implementing the TaskProcess thread.
 * @param argument: Not used
 * @retval None
 */

uint8_t processStep = 100;
/* USER CODE END Header_TaskRunProcess */
void TaskRunProcess(void const * argument)
{
  /* USER CODE BEGIN TaskRunProcess */
	/* Infinite loop */
	for (;;) {
//		if(process_ThucHienGapLua() == true){
//			process_Error(1);
//			osDelay(5);
//			process_Error(0);
//			osDelay(500);
//			__NOP();
//		}
		osDelay(100);
		}
  /* USER CODE END TaskRunProcess */
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
