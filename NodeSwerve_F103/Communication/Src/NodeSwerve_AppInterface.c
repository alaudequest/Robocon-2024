/*
 * NodeSwerve_AppInterface.c
 *
 *  Created on: Mar 30, 2024
 *      Author: SpiritBoi
 */

#include "NodeSwerve_AppInterface.h"
#include "Flag.h"
extern UART_HandleTypeDef huart1;
extern bool IsSetHome;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern bool BLDC_IsEnablePID;
extern bool DC_IsEnablePID;
extern bool UntangleBLDC;
uint8_t txBuffer[80] = { 0 };
uint8_t rxBuffer[80] = { 0 };
BoardID brdID = 0;
AppPararmPID_t _appPID;
float appTargetSpeedBLDC = 0;
float appTargetAngleDC = 0;
uint8_t relayCommand = 0;
static inline void Convert_AppParamPID_to_PIDParam(AppPararmPID_t appPID, PID_Param *pid);
static inline void Convert_PIDParam_to_AppParamPID(AppPararmPID_t *appPID, PID_Param pid);

static void ReceiveCommandHandler(CommandList cmdlist);
void SwerveApp_ErrorHandler(AppErrorCode err);

void SwerveApp_Init()
{
	// get ID board from flash memory
	uint32_t swerveID = *(__IO uint32_t*) (0x08000000 + 1024 * 64);
	switch (swerveID) {
		case 0:
			SwerveApp_ErrorHandler(APPERR_BOARD_NOT_FOUND);
			break;
		case 1:
			brdID = BOARD_NodeSwerve1;
			break;
		case 2:
			brdID = BOARD_NodeSwerve2;
			break;
		case 3:
			brdID = BOARD_NodeSwerve3;
			break;
	}
	/* Lưu giá trị Setpoint để khi set từ app không bị mất giá trị
	 * Setpoint(vì app sẽ thay đổi giá trị của brdParam.targetSpeedBLDC và brdParam.targetAngleDC
	 */
	appTargetAngleDC = brd_GetTargetAngleDC();
	appTargetSpeedBLDC = brd_GetTargetSpeedBLDC();

	appintf_Init(&huart1, txBuffer, sizeof(txBuffer), rxBuffer, sizeof(rxBuffer));
	// If any error happen, calling to this function
	appintf_RegisterErrorCallbackEvent(&SwerveApp_ErrorHandler);
	// After receiving data from app, calling to this function to handle command list
	appintf_RegisterReceivedCallbackEvent(&ReceiveCommandHandler);
	appintf_RegisterArgument((void*) &brdID, sizeof(brdID), CMD_Swerve_IdentifyBoard);
	appintf_RegisterArgument((void*) &_appPID, sizeof(_appPID), CMD_Swerve_GetPID);
	appintf_RegisterArgument((void*) &_appPID, sizeof(_appPID), CMD_Swerve_SetPID);
	appintf_RegisterArgument((void*) &appTargetSpeedBLDC, sizeof(appTargetSpeedBLDC), CMD_Swerve_SetTargetSpeedBLDC);
	appintf_RegisterArgument((void*) &appTargetAngleDC, sizeof(appTargetAngleDC), CMD_Swerve_SetTargetAngleDC);
	appintf_RegisterArgument((void*) &relayCommand, sizeof(relayCommand), CMD_Swerve_RelayCommand);
}

void SwerveApp_ErrorHandler(AppErrorCode err)
{
	while (1);
}

static void SendArgumentToApp(CommandList cmdlist) {
	appintf_MakeFrame(cmdlist);
	appintf_SendFrame();
}

static void HandleCommandSetPID() {
	appintf_GetValueFromPayload();
	if (_appPID.type < PID_DC_SPEED || _appPID.type > PID_BLDC_SPEED)
		SwerveApp_ErrorHandler(APPERR_BOARD_FEATURE_NOT_SUPPORT);
	PID_Param pid;
	Convert_AppParamPID_to_PIDParam(_appPID, &pid);
	brd_SetPID(pid, _appPID.type);
}

static void HandleCommandGetPID() {

	// vì trên app truyền giá trị byte xuống đại diện cho enum PID_type, nên khi lấy ra cũng phải khớp kiểu uint8_t
	uint8_t temp;
	appintf_GetValueFromPayload_2((void*) &temp, sizeof(temp));
	PID_type typePID = temp;
	PID_Param pid = brd_GetPID(typePID);
	if (typePID < PID_DC_SPEED || typePID > PID_BLDC_SPEED)
		SwerveApp_ErrorHandler(APPERR_BOARD_FEATURE_NOT_SUPPORT);
	Convert_PIDParam_to_AppParamPID(&_appPID, pid);
	appintf_MakeFrame(CMD_Swerve_GetPID);
	appintf_SendFrame();
}

static void SelectRunMotorInManualOrPID() {

	// Điều khiển chạy động cơ BLDC
	if (CHECKFLAG(relayCommand, RelayCmd_EnablePID_BLDC)) {
		// Cho phép tính toán PID và khởi chạy encoder
		HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
		BLDC_IsEnablePID = true;
		if (CHECKFLAG(relayCommand, RelayCmd_RunMotorBLDC)) {
			brd_SetTargetSpeedBLDC(appTargetSpeedBLDC);
		}
		else
			brd_SetTargetSpeedBLDC(0);
	}
	else { // Nếu không bật PID
		   // Tắt tính toán PID và tắt đọc encoder
		HAL_TIM_Encoder_Stop(&htim4, TIM_CHANNEL_ALL);
		BLDC_IsEnablePID = false;
		MotorBLDC mbldc = brd_GetObjMotorBLDC();
		if (CHECKFLAG(relayCommand, RelayCmd_RunMotorBLDC)) {
			MotorBLDC_Drive(&mbldc, appTargetSpeedBLDC);
		}
		else
			MotorBLDC_Drive(&mbldc, 0);
	}

	// Điều khiển chạy động cơ DC
	if (CHECKFLAG(relayCommand, RelayCmd_EnablePID_DC)) {
		HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
		DC_IsEnablePID = true;
		// PID ở đây là của vị trí góc quay động cơ nên setpoint là góc quay
		// không phải tốc độ động cơ, do đó không thể nhập góc 0 để dừng động cơ được
		if (CHECKFLAG(relayCommand, RelayCmd_RunMotorDC)) {
			brd_SetTargetAngleDC(appTargetAngleDC);
		}
//		else{
//			PID_Param pidSpeedDC = brd_GetPID(PID_DC_SPEED)
//		}
	}
	else {		// Nếu không bật PID
		// Tắt tính toán PID và tắt đọc encoder
		HAL_TIM_Encoder_Stop(&htim3, TIM_CHANNEL_ALL);
		DC_IsEnablePID = false;
		MotorDC mdc = brd_GetObjMotorDC();
		if (CHECKFLAG(relayCommand, RelayCmd_RunMotorDC)) {
			// giới hạn tốc độ quay DC
			if (appTargetAngleDC > 200)
				appTargetAngleDC = 200;
			else if (appTargetAngleDC < -200)
				appTargetAngleDC = -200;
			MotorDC_Drive(&mdc, appTargetAngleDC);
		}
		else
			MotorDC_Drive(&mdc, 0);
	}
}

/**
 * Gửi lên app trạng thái ban đầu của board, để đồng bộ trạng thái
 */
static void SendBoardInitStateToApp()
{
	// gửi trạng thái cờ bật PID của BLDC
	if (BLDC_IsEnablePID)
		SETFLAG(relayCommand, RelayCmd_EnablePID_BLDC);
	else
		CLEARFLAG(relayCommand, RelayCmd_EnablePID_BLDC);

	// gửi trạng thái cờ bật PID của BLDC
	if (DC_IsEnablePID)
		SETFLAG(relayCommand, RelayCmd_EnablePID_DC);
	else
		CLEARFLAG(relayCommand, RelayCmd_EnablePID_DC);
	appintf_MakeFrame(CMD_Swerve_RelayCommand);
	appintf_SendFrame();

}

static void ReceiveCommandHandler(CommandList cmdlist)
{
	switch (cmdlist) {
		case CMD_Swerve_IdentifyBoard:
			SendArgumentToApp(cmdlist);
			for (uint16_t i = 0; i < 20000; i++)		// delay để app phân biệt 2 gói tin gửi liền nhau
				__NOP();
			SendBoardInitStateToApp();
			break;
		case CMD_Swerve_SetTargetSpeedBLDC:
			case CMD_Swerve_SetTargetAngleDC:
			appintf_GetValueFromPayload();
			/* Khi vừa cài target setpoint xong thì chạy tốc độ mới luôn
			 thay vì phải click checkbox on/off từ trên app gửi xuống
			 */
			SelectRunMotorInManualOrPID();
			break;
		case CMD_Swerve_SetPID:
			HandleCommandSetPID();
			break;
		case CMD_Swerve_GetPID:
			HandleCommandGetPID();
			break;
		case CMD_Swerve_RelayCommand:
			appintf_GetValueFromPayload();
			SelectRunMotorInManualOrPID();
			break;
		case CMD_Swerve_SetHome:
			IsSetHome = true;
		case CMD_Swerve_UntangleBLDC:
			UntangleBLDC = true;
		default:
			break;
	}
}

static inline void Convert_AppParamPID_to_PIDParam(AppPararmPID_t appPID, PID_Param *pid)
{
	pid->kP = appPID.kp;
	pid->kI = appPID.ki;
	pid->kD = appPID.kd;
	pid->alpha = appPID.alpha;
	pid->deltaT = appPID.deltaT;
	pid->u_AboveLimit = appPID.limitHigh;
	pid->u_BelowLimit = appPID.limitLow;
}

static inline void Convert_PIDParam_to_AppParamPID(AppPararmPID_t *appPID, PID_Param pid)
{
	appPID->kp = pid.kP;
	appPID->ki = pid.kI;
	appPID->kd = pid.kD;
	appPID->alpha = pid.alpha;
	appPID->deltaT = pid.deltaT;
	appPID->limitHigh = pid.u_AboveLimit;
	appPID->limitLow = pid.u_BelowLimit;

}
