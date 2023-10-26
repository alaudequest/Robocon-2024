Trở về [README chính](../README.md)

Tham khảo [CAN_FuncHandle.h](../Slave_F103/Core/Inc/CAN_FuncHandle.h)

Tham khảo [CAN_FuncHandle.c](../Slave_F103/Core/Src/CAN_FuncHandle.c)

# Chức năng


Dùng để đóng gói, gửi đi(Put and Send message) và giải mã dữ liệu gói tin CAN nhận được (GetMessage) cho từng chế độ chạy được quy định trong CAN_MODE_ID (tham khảo [CAN_Control](CAN_Control.md))

# API

### canfunc_HandleRxEvent

+ Chức năng: kiểm tra và xóa cờ sự kiện canEvent, nếu có sự kiện xảy ra - khi canEvent > 0 hay gọi hàm canctrl_Receive trong ngắt - thì gọi tới hàm do người dùng viết (nếu có)

    Lưu ý: 
    + Khi có sự kiện xảy ra, hàm sẽ thực hiện quét enum CAN_MODE_ID để tìm ra sự kiện cần xử lý (do đó phát sinh thời gian overhead khi quét lần lượt các giá trị trong enum)
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

### canfunc_SetHomeValue - canfunc_GetHomeValue

+ Chức năng: dùng để truyền và nhận lệnh SetHome




