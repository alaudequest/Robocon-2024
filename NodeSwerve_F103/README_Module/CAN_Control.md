
Trở về [README chính](../README.md)

Tham khảo [CAN_Control.h](../../Main_F407/CAN/Inc/CAN_Control.h)

Tham khảo [CAN_Control.c](../../Main_F407/CAN/Src/CAN_Control.c)

__*Lưu ý*__: Project NodeSwerve_F103 không có CAN_Control và CAN_FuncHandle mà chỉ có Main_F407 mới có (để đồng nhất API sử dụng cho cả F4 và F1 khi update, thêm tính năng mới hoặc chỉnh sửa) vì vậy cần phải liên kết giữa Project __*NodeSwerve_F013*__ và folder __*CAN*__ ở Project __*Main_F407*__

Thực hiện liên kết theo các bước sau:

	1. Trong STM32CubeIDE, mục bên trái tên Project Explorer, click vào NodeSwerve_F103 nhấn tổ hợp Alt+Enter (hoặc chọn Properties)
	2. C/C++ Build -> Settings, mục Tool Settings -> MCU GCC Compiler -> Include paths
	3. Add directory path (symbol hình mặt giấy màu trắng có dấu + )
	4. Chọn Workspace -> Main_F407 -> CAN -> Inc (xong phần include các file .h), sau đó bấm OK và thoát ra tới mục C/C++ Build
	5. Chọn C/C++ General -> Paths and Symbols -> Source Location -> Link Folder
	6. Tick vào ô Link to folder in the file system, sau đó bấm Browse và trỏ tới thư mục CAN trong Main_F407, chọn Select Folder sau đó thoát ra, ở mục Source Location sẽ xuất hiện /NodeSwerve_F103/CAN, chọn Apply and Close, giờ đây trong project __*NodeSwerve_F013*__ sẽ xuất hiện folder CAN

# Cấu trúc thiết kế gói tin

+ Mô hình truyền thông CAN bus sử dụng ID chuẩn (Standard ID sau này viết tắt là StdID) để định danh gói tin, với mô hình này sẽ có 11 bit StdID được sử dụng để phân biệt gói tin truyền nhận.

+ Mô hình sử dụng chế độ lọc gói tin nhận được là List với Scalable 16 bit vì lý do sau:

    + Vì sử dụng ID chuẩn nhỏ hơn 16 bit nên chọn scale là 16 bit thay vì 32 bit(được dùng cho Extended CAN)
    + Số lượng thiết bị ít, nhỏ hơn 8 thiết bị, nếu dùng chế độ lọc là Mask thì 1 bit tương ứng với 1 node thiết bị (DEVICE) HOẶC 1 chế độ thực thi (MODE RUN) dẫn tới số bit được dùng tối đa chỉ là 11 (Giả sử cần 7 MODE thực thi thì sẽ chỉ còn dư lại 4 DEVICE để phân biệt).
    + Trong khi đó, nếu sử dụng chế độ List thì nhiều bit hợp thành một mã 
    thiết bị DEVICE hoặc chế độ chạy MODE RUN, mở rộng phạm vi phân biệt gói tin

+ Xét thấy số lượng thiết bị không vượt quá 8 và có thể >= 4 nên lựa chọn quy định 3 bit cuối từ 9-11 làm bit DEVICE[11:9] của StdID. Trong CAN_Control.h tạo một enum như sau:

```
typedef enum CAN_DEVICE_ID{
	CANCTRL_ID1 = 1
	CANCTRL_ID2
	CANCTRL_ID3
	CANCTRL_ID4
	CANCTRL_ID5
	CANCTRL_ID6
	CANCTRL_ID7
}CAN_DEVICE_ID;
```

__*Lưu ý*__: enum đặt trên ở trên không nhất thiết phải giống với trong code base do có thể tên thiết bị sẽ thay đổi trong tương lai

Để vị trí của mã ID thiết bị nằm ở DEVICE[11:9] thì cần dịch giá trị của enum CAN_DEVICE_ID đi 8, do đó định nghĩa macro sau:
```
#define CAN_DEVICE_POS 8
```
Ví dụ:
```
Example: uint32_t StdID |= CANCTRL_ID1 << CAN_DEVICE_POS
or uint32_t StdID |= 1 << 8 //equivalent 0x100
```
Sau khi định danh được thiết bị trong mã StdID, tiếp theo gắn thêm yêu cầu thực hiện chế độ hoạt động MODE RUN cho thiết bị, vì yêu chức năng của cụm bánh Swerve không vượt quá 16 nên sử dụng 4 bit LSB làm MODE[0:3]. Trong CAN_Control.h tạo một enum như sau:

```
typedef enum CAN_MODE_ID{
	CANCTRL_MODE_START,
	CANCTRL_MODE_LED_BLUE,
	CANCTRL_MODE_SHOOT,
	CANCTRL_MODE_ENCODER,
	CANCTRL_MODE_SET_HOME,
	CANCTRL_MODE_MOTOR_SPEED_ANGLE,
	CANCTRL_MODE_MOTOR_BLDC_BRAKE,
	CANCTRL_MODE_PID_DC_SPEED,
	CANCTRL_MODE_PID_DC_ANGLE,
	CANCTRL_MODE_PID_BLDC_SPEED,
	CANCTRL_MODE_PID_BLDC_BREAKPROTECTION,
	CANCTRL_MODE_TEST,
	CANCTRL_MODE_END,
}CAN_MODE_ID;
```
Vì là bit LSB nên không cần dịch bit.

Giả sử mong muốn cụm bánh có ID1 sẽ chạy với tốc độ BLDC __X__ và góc xoay DC __Y__ thì mã StdID cần gửi xuống cho cụm bánh đó sẽ là 0x106 theo như enum CAN_MODE_ID

Còn dư 4 bit ở giữa đang bỏ trống không dùng

# Các hàm API

### canctrl_Receive
+ Chức năng: Xử lý gói tin nhận được đang trong trạng thái chờ (message pending), set một bit cờ sự kiện canEvent tương ứng với enum CANCTRL_MODE_ID (Ví dụ có sự kiện CANCTRL_MODE_PID_DC_SPEED thì canEvent = 1 << CANCTRL_MODE_PID_DC_SPEED), lúc này biến rxHeader và biến rxData (biến private) sẽ được cập nhật với gói tin nhận được, dùng hàm canctrl_GetRxHeader để lấy rxHeader và dùng hàm canctrl_GetMessage để lấy dữ liệu từ rxData.
+ Hàm __*canfunc_HandleRxEvent*__ được dùng để kiểm tra và xóa cờ sự kiện canEvent, và gọi tới một hàm do người dùng tự viết để xử lý các sự kiện đó, tham khảo [CAN_FuncHandle](CAN_FuncHandle.md)
+ Return: HAL_StatusTypeDef
+ Đối số truyền vào:
    + CAN_HandleTypeDef *can: con trỏ trỏ tới địa chỉ của ngoại vi CAN
    + uint32_t FIFO: macro của RX_FIFO_0 và RX_FIFO_1
+ Sử dụng: gọi trong hàm ngắt __void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)__ hoặc __void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)__
+ Ví dụ:
	```
	void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
	{
	 HAL_CAN_DeactivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
	 canctrl_Receive(hcan, CAN_RX_FIFO0);
	}
	```
	__*Note*__: Sử dụng HAL_CAN_DeactivateNotification vì FIFO có 3 Mailbox, giả sử có Mailbox(1) đang trong quá trình xử lý dữ liệu nhận được và Mailbox(2) trong bộ đệm chờ xử lý thì hàm ngắt HAL_CAN_RxFifo0MsgPendingCallback sẽ được gọi liên tục và copy dữ liệu từ Mailbox(2) ghi đè vào Mailbox(1) và bị Critical Section trong khi Mailbox(1) chưa được xử lý xong, chỉ khi Mailbox 1 xử lý xong và set cờ sự kiện để ActiveNotification lại thì mới xử lý tiếp

### canctrl_Receive_2

+ Chức năng: Xử lý gói tin nhận được đang trong trạng thái chờ (message pending), trả về enum CAN_MODE_ID (khác với hàm canctrl_Receive là set cờ) 
+ Return: CAN_MODE_ID
+ Đối số truyền vào:
    + CAN_HandleTypeDef *can: con trỏ trỏ tới địa chỉ của ngoại vi CAN
    + uint32_t FIFO: macro của RX_FIFO_0 và RX_FIFO_1
+ Ví dụ (dùng trong FreeRTOS với API taskNotify):
	```
	void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan){
	 HAL_CAN_DeactivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
	 CAN_MODE_ID modeID = canctrl_Receive_2(hcan, CAN_RX_FIFO0);
	 BaseType_t HigherPriorityTaskWoken = pdFALSE;
	 xTaskNotifyFromISR(TaskHandleCANHandle,modeID,eSetValueWithOverwrite,&HigherPriorityTaskWoken);
	}

	// Task xử lý dữ liệu từ hàm ngắt ***************************
	void StartCANbus(void const * argument)
	{
		/* USER CODE BEGIN StartCANbus */
		CAN_Init();
		uint32_t modeID;
		/* Infinite loop */
		for(;;)
		{
			if(xTaskNotifyWait(pdFALSE, pdFALSE, &modeID, portMAX_DELAY)){
				handleMesgCAN((CAN_MODE_ID)modeID);
			}
		//    osDelay(1);
		}
		/* USER CODE END StartCANbus */
	}
	```
	Lưu ý trong hàm ngắt nhận Message Pending đã thực hiện tắt ngắt để chương trình không nhảy vào hàm ngắt khi bản tin CAN chưa được xử lý xong, vì vậy trong hàm xử lý của người dùng (trong trường hợp này là __*handleMesgCAN*__ )cần phải bật ngắt Message Pending trở lại dùng API __*HAL_CAN_ActivateNotification*__


### canctrl_Send

+ Chức năng: Truyền bản tin CAN tới Node được chỉ định trong mạng CAN 
+ Đối số truyền vào:
	+ CAN_HandleTypeDef *can: con trỏ trỏ tới địa chỉ của ngoại vi CAN
	+ CAN_DEVICE_ID targetID: chỉ định thiết bị nào trong mạng CAN sẽ nhận gói tin này
+ Return: HAL_StatusTypeDef
	+ HAL_OK: bản tin CAN được thêm thành công vào TX mailbox có sẵn và chờ được gửi vào mạng CAN
	+ HAL_ERROR: DLC của bản tin CAN trước đó chưa được xóa hoặc sai địa chỉ ngoại vi CAN
	+ HAL_BUSY: Không có TX mailbox nào trống để đưa dữ liệu vào, cần phải chờ đến khi một trong ba TX mailbox gửi đi thành công

	Ví dụ:
	```
	while(canctrl_Send(&hcan,CANCTRL_ID1)!= HAL_OK);
	```
	Trong ví dụ này, có thể thấy CPU phải chờ đến khi TX mailbox sẵn sàng để thêm gói tin vào và khiến CPU bị tiêu tốn thời gian chờ (test với baudrate 500 000 bit/s thì 1 gói tin CAN mất hơn 100uS), giải pháp cải thiện là sử dụng Queue hoặc Ring buffer và loại bỏ vòng lặp while

### canctrl_GetRxHeader

+ Chức năng: trả về Header của gói tin nhận được (nên dùng hàm này sau khi có sự kiện ngắt nhận được gói tin CAN gọi hàm __*HAL_CAN_GetRxMessage*__ sẽ cập nhật rxHeader mới nhất)
+ Return: CAN_RxHeaderTypeDef

Sử dụng: tạo một biến có kiểu dữ liệu CAN_RxHeaderTypeDef để chứa Header
```
CAN_RxHeaderTypeDef rxHeader = canctrl_GetRxHeader();
```

### canctrl_SetTargetDevice

+ Chức năng: Chọn ID node nhận để gửi dữ liệu tới.
+ Đối số: enum CAN_DEVICE_ID.

Ví dụ:
```
canctrl_SetTargetDevice(CANCTRL_ID1)
```
### canctrl_PutMessage

+ Chức năng: copy user data vào txData, chuẩn bị cho việc truyền dữ liệu .

	__*Lưu ý*__: chỉ truyền được dữ liệu nhỏ hơn 9 byte.
+ Return: HAL_StatusTypeDef.
+ Đối số: 
	+ void *data: con trỏ void trỏ tới mảng dữ liệu cần truyền (do người dùng cung cấp).
	+ size_t sizeOfDataType: kích thước gói tin cần truyền (nhỏ hơn 8 byte).

Ví dụ
```
float a;
struct B {
	float a;
	int c;
	uint8_t b;
};

struct B d;
canctrl_PutMessage((void*)&a,sizeof(a));
canctrl_Send(&hcan,CANCTRL_ID2);
canctrl_PutMessage((void*)&d,sizeof(d));
canctrl_Send(&hcan,CANCTRL_ID3);
```

### canctrl_GetMessage

+ Chức năng: Lấy dữ liệu nhận được từ rxData, copy ra địa chỉ của biến data.
	+ __*Lưu ý*__: 
		+ Chỉ nhận được dữ liệu nhỏ hơn 9 byte.
		+ Kích thước và kiểu dữ liệu phải khớp hoàn toàn với dữ liệu đưa vào hàm canctrl_PutMessage.
+ Return: HAL_StatusTypeDef.
+ Đối số: 
	+ void *data: con trỏ void trỏ tới mảng dữ liệu cần nhận (do người dùng lựa chọn).
	+ size_t sizeOfDataType: kích thước gói tin cần truyền (nhỏ hơn 8 byte).

Ví dụ:

```
float e;
struct B {
	float a;
	int c;
	uint8_t b;
};

struct B f;
canctrl_GetMessage((void*)&e,sizeof(e));
canctrl_GetMessage((void*)&f,sizeof(f));
```

### canctrl_SendMultipleMessages

+ Chức năng: Gửi nhiều gói tin CAN cùng lúc (lưu ý là cấu trúc dữ liệu nhận và gửi phải giống nhau).
+ Return: HAL_StatusTypeDef
	+ HAL_OK: Nhận thành công đầy đủ gói tin CAN.
	+ HAL_ERROR: truyền con trỏ NULL cho ngoại vi CAN hoặc data hoặc kích thước gói tin là 0.
	+ HAL_BUSY: gói tin CAN chưa gửi xong.
+ Đối số: 
	+ CAN_HandleTypeDef *can: con trỏ trỏ tới địa chỉ của ngoại vi CAN.
	+ CAN_DEVICE_ID targetID: chỉ định thiết bị nào trong mạng CAN sẽ nhận gói tin này.
	+ void *data: con trỏ void trỏ tới mảng dữ liệu cần truyền (do người dùng cung cấp).
	+ size_t sizeOfDataType: kích thước gói tin cần truyền.
+ Ví dụ:
	```
	struct A{
		float param1;
		uint8_t param2;
		uint16_t param3;
	}
	struct A a{
		.param1 = -16.25,
		.param2 = 20,
		.param3 = 500,
	}
	if(canctrl_SendMultipleMessages(hcan,CANCTRL_ID1,(void*)&a,sizeof(struct A)) == HAL_OK){
		// Do something after sending data complete
	}
	```
### canctrl_GetMultipleMessages

+ Chức năng: Nhận nhiều gói tin CAN cùng lúc (lưu ý là cấu trúc dữ liệu nhận và gửi phải giống nhau).
+ Return: HAL_StatusTypeDef.
+ Đối số: 
	+ void *data: con trỏ void trỏ tới mảng dữ liệu cần truyền (do người dùng cung cấp).
	+ size_t sizeOfDataType: kích thước gói tin cần truyền.
+ Ví dụ:
	```
	struct A{
		float param1;
		uint8_t param2;
		uint16_t param3;
	}
	struct A b;
	if(canctrl_GetMultipleMessages(hcan,CANCTRL_ID1,(void*)&b,sizeof(struct A)) == HAL_OK){
		// Do something after receiving data complete
	}
	```
### canctrl_RTR_TxRequest

+ Chức năng: Gửi yêu cầu lấy dữ liệu tới node CAN
+ Return: HAL_StatusTypeDef
+ Đối số: 
	+ CAN_HandleTypeDef *can: con trỏ trỏ tới địa chỉ của ngoại vi CAN.
	+ CAN_DEVICE_ID targetID: chỉ định thiết bị nào trong mạng CAN sẽ nhận gói tin này.
	+ CAN_MODE_ID modeID: thông số yêu cầu node trả về dữ liệu.
+ Ví dụ:
	```
	canctrl_RTR_TxRequest(&hcan1, CANCTRL_DEVICE_MOTOR_CONTROLLER_1, CANCTRL_MODE_SET_HOME);
	```

### canctrl_RxHeaderGetModeID
+ Chức năng: Tách thông số MODE_ID từ Header CAN nhận được.
+ Return: CAN_MODE_ID.
### canctrl_Filter_List16

+ Chức năng: cài đặt ID lọc cho module CAN theo Scalable là 16bit (vì dùng chuẩn Standard ID chỉ có 11 bit), và mode là List (mã ID khớp hoàn toàn với mã cài đặt thì gói tin mới được chấp nhận)
+ Return: HAL_StatusTypeDef
+ Đối số: 
	+ CAN_HandleTypeDef *can: địa chỉ module CAN cần cài đặt bộ lọc.
	+ uint16_t ID1, ID2, ID3, ID4: 4 mã ID của 1 filter bank khi dùng chế độ List (phải bao gồm định danh của Node là CANCTRL_DEVICE_ID và chế độ chạy CANCTRL_MODE_ID).
	+ uint32_t filBank: lựa chọn hàng lọc thứ filBank (0,1,2 ... tới giới hạn hàng lọc của ngoại vi CAN).
	+ uint32_t FIFO: cài đặt hàng lọc này với 4 ID trên sẽ áp dụng cho FIFO nào.

Ví dụ:

Giả sử lọc Node 1 là CANCTRL_ID1 và Mode là CANCTRL_MODE_MOTOR_SPEED_ANGLE
thì ID lọc là 0x105 cho filter bank số 1 áp dụng cho FIFO 0

```
canctrl_Filter_List16(&hcan, 0x105, 0, 0, 0, 1, CAN_RX_FIFO0)
```

### canctrl_Filter_Mask16

+ Chức năng: cài đặt ID lọc cho module CAN theo Scalable là 16bit (vì dùng chuẩn Standard ID chỉ có 11 bit), và mode là Mask (mã ID khớp một trong số các bit 1 của mặt nạ)
+ Return: HAL_StatusTypeDef
+ Đối số: 
	+ CAN_HandleTypeDef *can: địa chỉ module CAN cần cài đặt bộ lọc.
	+ uint16_t lowID, maskLow: địa chỉ ID (giống với ID1 trong List 16) và mặt nạ bit áp dụng cho địa chỉ này (giống với ID2 trong List 16).
	+ uint16_t highID, maskHigh: địa chỉ ID (giống với ID3 trong List 16) và mặt nạ bit áp dụng cho địa chỉ này (giống với ID4 trong List 16).
	+ uint32_t filBank: lựa chọn hàng lọc thứ filBank (0,1,2 ... tới giới hạn hàng lọc của ngoại vi CAN).
	+ uint32_t FIFO: cài đặt hàng lọc này với 4 ID trên sẽ áp dụng cho FIFO nào.

__*Lưu ý*__: trong chế độ này, nếu sử dụng 3 bit cuối của StdID thì chỉ có thể lọc ra được 3 Node tương ứng với bit 8, bit 9 và bit 10 và Mode chỉ có phân biệt được 4 Mode khác nhau tương ứng với bit 0, 1 ,2 ,3.

Ví dụ:

Giả sử lọc Node 1 là CANCTRL_ID1 cho lowID(nghĩa là chỉ cần xuất hiện giá trị 1 trong bit 8 của StdID thì bộ lọc sẽ chấp nhận gói tin) áp dụng cho FIFO 0 với filter bank là 1.

```
canctrl_Filter_List16(&hcan, 0x105, 0, (CANCTRL_ID1 << CAN_DEVICE_POS), 0, 1, CAN_RX_FIFO0)
```
Ở đây có thể thấy ID là 0x105, tuy nhiên Mask lại là 0x100. Vậy ý nghĩa của bộ lọc là: "Với bất kỳ gói tin nào có bit thứ 8 mang giá trị 1 (chỉ định Node 1) thì sẽ được chấp nhận, không quan tâm tới chế độ chạy của Node.", vậy sau khi nhận được gói tin dành cho Node 1 thì cần phải kiểm tra StdID (StdID & 0x0f) để biết được bản tin đang yêu cầu Node 1 chạy ở chế độ nào. Ngược lại nếu Mask là 0x005 thì tất cả các Node có chức năng được quy định bởi mã 0x005 sẽ thực hiện cùng một lúc (ví dụ chế độ Set Home cho các Node). Giả sử Mask = 0x020 và ID = 0x105 thì bộ lọc sẽ vô dụng do không khớp bit giữa ID và Mask nên sẽ không chấp nhận bất kỳ gói tin nào dù StdID nhận được là 0x105.
