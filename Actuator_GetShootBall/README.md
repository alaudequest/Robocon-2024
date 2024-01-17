- Cơ cấu sử dụng 5 động cơ: 1 động cơ xoay góc bắn (RotaryGun), 2 động cơ bắn (RuloGun), 2 động cơ lấy bóng (RuloBall). Ngoài ra còn có 3 encoder: 2 encoder cho động cơ bắn và 1 encoder cho động cơ xoay góc.
- Động cơ xoay góc và 2 động cơ bắn sẽ điều khiển bằng xung PWM còn 2 động cơ rulo lấy bóng sẽ điều khiển bằng cách xuất mức logic (1: chạy; 0: dừng).
- Encoder trên 2 động cơ bắn sẽ đọc bằng ngắt và encoder trên động cơ xoay góc sẽ đọc bằng encoder mode.
- Tất cả động cơ sẽ không chạy liên tục. Cụm động cơ bắn và lấy bóng sẽ hoạt động khi có cảm biến bắt được bóng hoặc có tín hiệu gửi từ main xuống (Cảm biến đang update).
- Cảm biến sẽ có 2 phương án kết nối: board input (tín hiệu từ main gửi xuống) hoặc jack cảm biến chữ U trên driver swever bldc V1.0 (xử lý nội trong driver).
- CANBus lựa chọn cấu hình với baudrate là 1000000 bit/s (đã cấu hình).
- Sử dụng hệ điều hành RTOS để đảm bảo tốc độ đáp ứng của hệ thống.

| STM32 Pin | Feature              | Type    |
|-----------|----------------------|---------|
| PA0       | Rulo Ball 1          | GPIO    |
| PA4       | Rulo Ball 2          | GPIO    |
| PA2       | Rotary Gun 1         | TIM2_CH3|
| PA3       | Rotary Gun 2         | TIM2_CH4|
| PA6       | Encoder Rotary A     | TIM3_CH1|
| PA7       | Encoder Rotary B     | TIM3_CH2|
| PA9       | Rulo Gun 1           | TIM1_CH2|
| PA10      | Rulo Gun 1           | TIM1_CH3|
| PA11      | CANBus TX            | CAN_TX  |
| PA12      | CANBus RX            | CAN_RX  |
| PB6       | Encoder Gun 1 A      | EXTI6   |
| PB7       | Encoder Gun 1 B      | EXTI7   |
| PB3       | Encoder Gun 2 A      | EXTI3   |
| PB15      | Encoder Gun 2 B      | EXTI15  |