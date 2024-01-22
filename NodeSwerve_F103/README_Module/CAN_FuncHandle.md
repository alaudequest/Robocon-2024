Trở về [README chính](../README.md)

Tham khảo [CAN_FuncHandle.h](../../Main_F407/CAN/Inc/CAN_FuncHandle.h)

Tham khảo [CAN_FuncHandle.c](../../Main_F407/CAN/Src/CAN_FuncHandle.c)

# Chức năng


Dùng để đóng gói, gửi đi (Put and Send message) và giải mã dữ liệu gói tin CAN nhận được (GetMessage) cho từng chế độ chạy được quy định trong CAN_MODE_ID (tham khảo [CAN_Control](CAN_Control.md))


# Cấu trúc dữ liệu

## CAN_SpeedBLDC_AngleDC

Dùng để đóng gói và phần giải gói tin chứa giá trị float tốc độ của động cơ BLDC và góc quay DC

## CAN_RTR_Encx4BLDC_AngleDC

Dùng để đóng gói và phần giải gói tin chứa giá trị số xung X4 (int) của encoder động cơ BLDC và góc quay DC (float)

## CAN_PID

Dùng để đóng gói và phần giải gói tin chứa 5 thông số kp, ki, kd, alpha, deltaT của PID

# API

### canfunc_RTR_GetEncoderX4CountBLDC_Angle (Sender xử lý gói tin từ Receiver)
+ Chức năng: sau khi Receiver phản hồi bản tin CAN thì phân giải và tách lấy và return dữ liệu nhận được
+ Return: CAN_RTR_Encx4BLDC_AngleDC
+ Ví dụ:
Code Sender gửi request và xử lý bản tin CAN trả về từ Receiver:
```
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
	HAL_CAN_DeactivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
	CAN_MODE_ID modeID = canctrl_Receive_2(hcan, CAN_RX_FIFO0);
	BaseType_t HigherPriorityTaskWoken = pdFALSE;
	xTaskNotifyFromISR(TaskCANHandle, modeID, eSetValueWithOverwrite, &HigherPriorityTaskWoken);
	portYIELD_FROM_ISR(HigherPriorityTaskWoken);
}
void handleFunctionCAN(CAN_MODE_ID mode, CAN_DEVICE_ID targetID) {
	switch (mode) {
		case CANCTRL_MODE_NODE_REQ_SPEED_ANGLE:
			nodeSpeedAngle[targetID - 1] = canfunc_MotorGetSpeedAndAngle();
		break;
		default:
			break;
	}
}

void CAN_Init() {
	HAL_CAN_Start(&hcan1);
	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_RX_FIFO1_MSG_PENDING);
	canctrl_Filter_Mask16(&hcan1,
			CANCTRL_MODE_NODE_REQ_SPEED_ANGLE << 5,
			0
			CANCTRL_MODE_NODE_REQ_SPEED_ANGLE << 5,
			0
			0,
			CAN_RX_FIFO0);
}


void CAN_Bus(void const * argument)
{
  /* USER CODE BEGIN CAN_Bus */
	CAN_Init();
    // gửi request cho node thứ 3
    canctrl_RTR_TxRequest(&hcan1, CANCTRL_ID3, CANCTRL_MODE_NODE_REQ_SPEED_ANGLE);
	uint32_t modeID;
	/* Infinite loop */
	for (;;) {
		if (xTaskNotifyWait(pdFALSE, pdFALSE, &modeID, portMAX_DELAY)) {
			CAN_RxHeaderTypeDef rxHeader = canctrl_GetRxHeader();
			uint32_t targetID = rxHeader.StdId >> CAN_DEVICE_POS;
			if (modeID == CANCTRL_MODE_NODE_REQ_SPEED_ANGLE ) && targetID) {
				handleFunctionCAN(modeID, targetID);
			}
			HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
		}
	}
  /* USER CODE END CAN_Bus */
}
```


### canfunc_RTR_SetEncoderX4CountBLDC_Angle (Receiver phản hồi)

+ Chức năng: sau khi nhận được request RTR (có bit RTR trong header nhận được - rxHeader) tham khảo hàm __*canctrl_RTR_TxRequest*__ của [CAN_Control](CAN_Control.md) do Sender yêu cầu, Receiver đáp ứng và thực hiện đóng gói số xung encoder X4 của BLDC và giá trị góc quay của động cơ DC và gửi vào mạng CAN tới Sender, trong header gửi đi phải kèm theo giá trị CAN_DEVICE_ID để thông báo cho Sender biết node nào đang gửi 
+ Mode: CANCTRL_MODE_NODE_REQ_SPEED_ANGLE
+ Return: void
+ Đối số: 
    + CAN_HandleTypeDef *can: con trỏ trỏ tới địa chỉ của ngoại vi CAN

    + CAN_RTR_Encx4BLDC_AngleDC rtrData: cấu trúc dữ liệu chứa số xung encoder X4 và góc quay
+ Ví dụ:



Code Receiver response:

__*Note*__: 

+ Các hàm liên quan tới brd_* tham khảo [BoardParameter](/NodeSwerve_F103/README_Module/BoardParamter.md)

+ Ví dụ bên dưới có liên quan tới sử dụng hệ điều hành RTOS

```
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
	HAL_CAN_DeactivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
	CAN_MODE_ID modeID = canctrl_Receive_2(hcan, CAN_RX_FIFO0);
    BaseType_t HigherPriorityTaskWoken = pdFALSE;
	xTaskNotifyFromISR(TaskHandleCANHandle, modeID, eSetValueWithOverwrite, &HigherPriorityTaskWoken);
	portYIELD_FROM_ISR(HigherPriorityTaskWoken);
}
void CAN_Init() {
	HAL_CAN_Start(&hcan);
	HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING
			| CAN_IT_RX_FIFO1_MSG_PENDING
			| CAN_IT_RX_FIFO0_FULL);
	canctrl_Filter_Mask16(&hcan,
			1 << CAN_RTR_REMOTE, // this is highID 
			0,// this is lowID, not use
			1 << CAN_RTR_REMOTE, // this is maskHigh
			0,// this is maskLow, not use
			2,
			CAN_RX_FIFO0);
}

void handle_CAN_RTR_Response(CAN_HandleTypeDef *can, CAN_MODE_ID modeID) {
	PID_Param pid;
	switch (modeID) {
		case CANCTRL_MODE_NODE_REQ_SPEED_ANGLE:
			CAN_RTR_Encx4BLDC_AngleDC rtrData;
            // có thể cho tùy ý một số int để test
			rtrData.encx4BLDC = brd_GetCurrentCountBLDC();
            // có thể cho tùy ý một số float để test
			rtrData.dcAngle = brd_GetCurrentAngleDC();
			canfunc_RTR_SetEncoderX4CountBLDC_Angle(can, rtrData);
		break;
		default:
			break;
	}
}

void main()
{
    HAL_Init();
    SystemClock_Config();
    MX_CAN_Init();
    osThreadStaticDef(TaskHandleCAN, StartCANbus, osPriorityAboveNormal, 0, 128, TaskHandleCANBuffer, &TaskHandleCANControlBlock);
    TaskHandleCANHandle = osThreadCreate(osThread(TaskHandleCAN), NULL);
    osKernelStart();
    while(1){

    }
}

void StartCANbus(void const * argument)
{
  /* USER CODE BEGIN StartCANbus */
	CAN_Init();
	uint32_t modeID;
	/* Infinite loop */
	for (;;) {
		if (xTaskNotifyWait(pdFALSE, pdFALSE, &modeID, portMAX_DELAY)) {
			CAN_RxHeaderTypeDef rxHeader = canctrl_GetRxHeader();
            // Kiểm tra ID request có phải là ID trong bộ nhớ flash hay không
			if (((rxHeader.StdId >> CAN_DEVICE_POS) == *(__IO uint32_t*) FLASH_ADDR_TARGET)) {
				if (rxHeader.RTR == CAN_RTR_REMOTE)
					handle_CAN_RTR_Response(&hcan, modeID);
				if (rxHeader.RTR == CAN_RTR_DATA)
					handleFunctionCAN((CAN_MODE_ID) modeID);
			}
			HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_RX_FIFO1_MSG_PENDING);
		}
//    osDelay(1);
	}
  /* USER CODE END StartCANbus */
}

```

### canfunc_HandleRxEvent

+ Chức năng: kiểm tra và xóa cờ sự kiện canEvent, nếu có sự kiện xảy ra - khi canEvent > 0 do chương trình nhảy tới hàm ngắt và gọi hàm canctrl_Receive  - thì thực hiện kiểm tra bit cờ và gọi tới hàm do người dùng viết (nếu có)

    Lưu ý: 
    + Khi có sự kiện xảy ra, hàm sẽ thực hiện kiểm tra từng macro của enum CAN_MODE_ID để tìm ra sự kiện cần xử lý (do đó phát sinh thời gian overhead khi kiểm lần lượt các giá trị trong enum)
    + Hàm này được sử dụng nếu MCU chạy ở non-blocking (không có sử dụng hệ điều hành RTOS)
+ Return: void
+ Đối số truyền vào: hàm xử lý do người dùng truyền địa chỉ hàm, có đối số là __*CAN_MODE_ID*__

+ Ví dụ:
	```
    // this is user function to handle CAN RX event
    void canRxEventCallback(CAN_MODE_ID ID){
        switch(ID){
            case CANCTRL_MODE_MOTOR_SPEED_ANGLE:
            // Do something
            break;
        }
    }

    // the function wil pass the address of above function as argument
    canfunc_HandleRxEvent(&canRxEventCallback);
	```

### canfunc_MotorPutSpeedAndAngle

+ Chức năng: dùng để gửi tốc độ động cơ BLDC và góc quay của động cơ DC
+ Đối số: struct CAN_SpeedBLDC_AngleDC
+ Mode: CANCTRL_MODE_MOTOR_SPEED_ANGLE
+ Return: void
+ Ví dụ: 
    ```
    CAN_SpeedBLDC_AngleDC speedBLDC_AngleDC = {
        .bldcSpeed = -100.30,
        .dcAngle = -30.5,
    };
    canfunc_MotorPutSpeedAndAngle(speedBLDC_AngleDC);
    while(canctrl_Send(&hcan,CANCTRL_ID1) != HAL_OK);
    ```
### canfunc_MotorGetSpeedAndAngle

+ Chức năng: dùng để nhận tốc độ động cơ BLDC và góc quay của động cơ DC từ bản tin CAN
+ Mode: CANCTRL_MODE_MOTOR_SPEED_ANGLE
+ Return: struct CAN_SpeedBLDC_AngleDC
+ Ví dụ:
    ```
    CAN_SpeedBLDC_AngleDC speedAngle = canfunc_MotorGetSpeedAndAngle();
    // Do something with speedAngle
    ```

### canfunc_PutAndSendParamPID

+ Chức năng: gửi 5 thông số bao gồm kp, ki, kd, alpha, deltaT. 
+ Nguyên lý gửi: Tất cả 5 thông số đều là kiểu float (1 biến float chiếm 4 byte) do đó một bản tin CAN có thể truyền được tối đa 2 biến float (8 byte data), vì vậy cần truyền 3 lần:
    + Lần 1: truyền kp và ki.
    + Lần 2: truyền kd và alpha.
    + Lần 3: truyền deltaT

    Vì chỉ lấy 5 thông số từ PID_Param của thư viện PID.h nên trong file CAN_FuncHandle.h tạo ra một struct chỉ chứa 5 thông số trên:
    ```
    typedef struct CAN_PID{
	float kp;
	float ki;
	float kd;
	float alpha;
	float deltaT;
    }CAN_PID;
    ```
+ Mode: PID_BLDC_SPEED
+ Đối số:
    + CAN_HandleTypeDef *can: con trỏ trỏ tới địa chỉ của ngoại vi CAN
    + CAN_DEVICE_ID targetID: chỉ định thiết bị nào trong mạng CAN sẽ nhận gói tin này
    + PID_Param pid: bộ 5 thông số PID cần truyền
    + PID_type type: một module Swerve cần có 3 bộ PID (PID cho tốc độ BLDC, PID cho tốc độ và góc xoay động cơ DC), vì vậy cần chỉ định chọn bộ PID nào để truyền đi - tham khảo enum __*PID_type*__ của [BoardParameter.h](../Core/Inc/BoardParameter.h)

+ Ví dụ:
    ```
    PID_Param pidParam = brd_GetPID(PID_BLDC_SPEED);
    CAN_PID canpid;
    canpid.kp = pidParam.kP
    canpid.ki = pidParam.kI
    canpid.kd = pidParam.kD
    canpid.alpha = pidParam.alpha
    canpid.deltaT = pidParam.deltaT
    canfunc_PutAndSendParamPID(can,DEVICE_ID1,canpid,PID_BLDC_SPEED
    )
    ```

### canfunc_GetPID

+ Chức năng: nhận 5 thông số bao gồm kp, ki, kd, alpha, deltaT. 
+ Đối số:
    + void (*pCallback)(PID_Param pid, PID_type type): con trỏ hàm trỏ tới hàm do người dùng tự viết trong main.c sau khi nhận đủ 5 thông số để xử lý, hàm này cần tuân thủ các điều kiện sau:
        + Return: void
        + Đối số PID_Param pid: bộ 5 thông số PID cần truyền
        + Đối số PID_type type: một module Swerve cần có 3 bộ PID (PID cho tốc độ BLDC, PID cho tốc độ và góc xoay động cơ DC), vì vậy cần chỉ định chọn bộ PID nào để truyền đi - tham khảo enum __*PID_type*__ của [BoardParameter.h](../Core/Inc/BoardParameter.h)
+ Mode: PID_BLDC_SPEED
+ Ví dụ:

    ```
    void can_GetPID_CompleteCallback(CAN_PID canPID, PID_type type){
	PID_Param pid = brd_GetPID(type);
	canfunc_Convert_CAN_PID_to_PID_Param(canPID, &pid);
	brd_SetPID(pid, type);
    }   

    void handleFunctionCAN(CAN_MODE_ID mode){
        switch(mode){
        case CANCTRL_MODE_PID_DC_SPEED:
        case CANCTRL_MODE_PID_DC_ANGLE:
        case CANCTRL_MODE_PID_BLDC_SPEED:
            canfunc_GetPID(&can_GetPID_CompleteCallback);
            break;
        }
    }
    ```
### canfunc_SetBoolValue

+ Chức năng: Gửi lệnh đơn để bật/tắt chế độ hoạt động
+ Return: void
+ Đối số: 
    + bool bVal: giá trị để bật / tắt chế độ hoạt động
    + CAN_MODE_ID modeID: chọn chế độ hoạt động, hiện có 4 chế độ có thể sử dụng ở dạng bật/tắt là SetHome, hủy bảo vệ BLDC của JY-01, TestMode và Brake của JY-01
    tương ứng với các macro CANCTRL_MODE_TEST, CANCTRL_MODE_PID_BLDC_BREAKPROTECTION, CANCTRL_MODE_SET_HOME, CANCTRL_MODE_MOTOR_BLDC_BRAKE
+ Ví dụ:
    ```
    canfunc_SetBoolValue(1,CANCTRL_MODE_SET_HOME);
    while(canctrl_Send(&hcan,CANCTRL_ID1) != HAL_OK);
    ```
### canfunc_GetBoolValue

+ Chức năng: Nhận lệnh đơn để bật / tắt chế độ hoạt động
+ Return: bool
+ Ví dụ:
    ```
    bool bVal = canfunc_GetBoolValue();
    ```

