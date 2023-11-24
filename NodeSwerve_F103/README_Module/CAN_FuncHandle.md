Trở về [README chính](../README.md)

Tham khảo [CAN_FuncHandle.h](../../Main_F407/CAN/Inc/CAN_FuncHandle.h)

Tham khảo [CAN_FuncHandle.c](../../Main_F407/CAN/Src/CAN_FuncHandle.c)

# Chức năng


Dùng để đóng gói, gửi đi (Put and Send message) và giải mã dữ liệu gói tin CAN nhận được (GetMessage) cho từng chế độ chạy được quy định trong CAN_MODE_ID (tham khảo [CAN_Control](CAN_Control.md))

# API

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

