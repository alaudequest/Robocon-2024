
I. Định dạng gói tin của giao thức CAN - The Bit Fields of CAN
1. Giao thức thông thường - Standard CAN

2. Giao thức mở rộng -  Extended CAN

II. Bộ lọc CAN - CAN Bus Filter
1. Khái quát về bộ lọc
Trong giao thức CAN, bộ lọc CAN (CAN Bus Filter) là một tính năng quan trọng được hỗ trợ trong vi điều khiển STM32 và các vi điều khiển khác. Bộ lọc này cho phép xác định thông điệp nào sẽ được chấp nhận bởi vi điều khiển. Khi một thông điệp được nhận vào vi điều khiển, nó sẽ so sánh với các bộ lọc được cấu hình trước đó. Nếu thông điệp khớp với bộ lọc nào, vi điều khiển sẽ lưi thông điệp vào ô nhớ FIFO (First In First Out) hoặc FIFO truyền, hoặc kích hoạt ngắt (Interrupt) để thông báo về việc nhận thông điệp.

Bộ lọc CAN Thường bao gồm 2 phần quan trọng:
	- 32-bit filter mask (Msk): Đây là giá trị mask được dùng để quyết định thông điệp nào sẽ được lưu trữ hoặc nhận dựa trên ID (Identifier) của thông điệp.

	- 32-bit filter code (Id): Đây là giá trị ID mà thông điệp được so sánh với. Nếu một thông điệp có ID khớp với bộ lọc, nó được chấp nhận.

Một số bộ lọc có thể được cấu hình để xử lý một số ít thông điệp hoặc các nhóm thông điệp dựa trên ID. Bộ lọc cũng có thể được cấu hình để chấp nhận hoặc từ chối các thông điệp có chiều dài dữ liệu (DLC) cụ thể.

Việc sử dụng bộ lọc CAN giúp giảm tải xử lý của vi điều khiển bằng cách chỉ xử lý các thông điệp quan trọng và loại bỏ những thông điệp không liên quan, đồng thời giúp tránh xảy ra lỗi khi nhận nhầm thông điệp từ các thiết bị không mong muốn trên mạng CAN.

2. Một số giá trị cài đặt của bộ lọc CAN - CAN Filter Config

    CAN_FilterTypeDef canfilterconfig;

    canfilterconfig.FilterActivation = CAN_FILTER_ENABLE;		
    canfilterconfig.FilterBank = 10;
    canfilterconfig.FilterFIFOAssignment = CAN_RX_FIFO0;
    canfilterconfig.FilterIdHigh = 0;
    canfilterconfig.FilterIdLow = 0x0000;
    canfilterconfig.FilterMaskIdHigh = 0;
    canfilterconfig.FilterMaskIdLow = 0x0000;
    canfilterconfig.FilterMode = CAN_FILTERMODE_IDMASK;
    canfilterconfig.FilterScale = CAN_FILTERSCALE_32BIT;
    canfilterconfig.SlaveStartFilterBank = 0;

    HAL_CAN_ConfigFilter(&hcan, &canfilterconfig);

    FilterActivation: sử dụng bộ lọc hay 
    FilterBank: là tập hợp các thuộc tính bộ lọc được hỗ trợ bởi phần cứng của vi điều khiển cho phép ta cấu hình nhiều bộ lọc CAN để lọc các thông điệp CAN theo nhiều tiêu chí khác. Số lượng FilterBank phụ thuộc vào phần cứng điều khiển. Nó sẽ bao gồm các thông tin như lọc ID nhận vào, thông tin nhận vào, lọc theo hướng (Chỉ truyền hoặc chỉ nhận), lọc theo chiều dài dữ liệu.

III. Các tính năng của CAN
1. Loopback mode
Đây là chức năng hữu dụng của CAN khi cho phép ta gửi thông tin và đồng thời nhận lại chính cái thông tin vừa gửi đi trên đường dây đó. Loopback mode thường được dùng cho mục đích kiểm tra và sửa lỗi khi cho phép các node xác nhận sự chuẩn xác của thông tin vừa gửi đi mà không cần phải kết nối thêm bất kì node nào nữa.

2. Normal mode