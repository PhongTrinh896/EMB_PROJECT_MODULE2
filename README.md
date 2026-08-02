# EMB_PROJECT_MODULE2  
  
**Bài tập này chú trọng vào quá trình giao tiếp giữa Node và Gateway. Node có nhiệm vụ thu thập thông tin vầ độ ẩm và nhiệt độ môi trường thông qua cảm biến, hiển thị lên màn hình LCD, điều khiển động cơ máy tưới nước, đồng thời gửi thông tin tới Gateway thông qua LoRa. Gateway có nhiệm vụ thu thập thông tin từ Node, hiển thị lên màn hình LCD, gửi thông tin điều khiển động cơ máy bơm nước (nếu càn) và cập nhật thông tin lên máy tính thông qua giao thức UART, hiển thị trên Hercules hoặc Cutecom.**  
  
**Flowchart của toàn hệ thống**  
  
<img width="878" height="673" alt="Screenshot from 2026-08-02 17-49-32" src="https://github.com/user-attachments/assets/41273dcf-ed91-407e-b1e8-b963252f695f" />

**Cấu hình Gateway**  
- USART1: Giao tiếp với module LoRa.  
- USART2: Giao tiếp với máy tính qua USB–UART; sử dụng Hercules để giám sát dữ liệu và gửi lệnh.  
- I2C2: Giao tiếp với màn hình LCD thông qua module I2C.  
- PB3: GPIO Output – LED báo Gateway nhận hoặc chuyển tiếp gói tin thành công.  
- PB0: GPIO External Interrupt – nút dừng khẩn cấp, sử dụng ngắt EXTI0.    

**Cấu hình Node**
- ADC1 : Đọc tín hiệu tương tự phục vụ đo điện áp pin.  
- I2C1 : Giao tiếp với màn hình LCD I2C.  
- TIM2 : Tạo xung PWM để điều khiển công suất máy bơm.  
- USART1 : Thực hiện truyền và nhận dữ liệu giữa STM32 và module LoRa.  
  
**Cấu hình LoRa**  
- Baud rate: 115200
- ID: 1111
- Channel: 5
  
**Cấu trúc gói tin**  
* Cấu trúc gói tin truyền qua LoRa được thống nhất từ Node sang Gateway gồm 8 byte thông tin lần lượt như sau: 
  
| Byte | Ý nghĩa      |
| ---- | ------------ |
| 0    | 0xAA         |
| 1    | Humidity     |
| 2    | Temperature  |
| 3    | Battery1     |
| 4    | Battery2     |
| 5    | Motor State  |
| 6    | Checksum XOR |
| 7    | 0xBB         |  

- Trong đó, các byte thông tin về độ ẩm, nhiệt độ, tình trạng pin của 2 cảm biến, tình trạng máy bơm nước, byte checksum kiểm tra tính toàn vẹn của gói tin đều được thống nhất định dạng uint16_t. Byte Checksum được sử dụng phương thức tính XOR của từng thông tin (khả năng thấp có sự trùng hợp gây lỗi byte này mà Gateway vẫn đọc gói tin là hợp lệ).   
- Hai byte đánh dấu đầu cuối gói tin được thống nhất trong cả Gateway và Node là 0xAA và 0xBB. Hai giá trị này khả năng thấp sẽ trùng với thông tin từ các cảm biến, tuy nhiên vẫn cần có phương án đảm bảo an toàn thêm.  

* Cấu trúc gói tin truyền qua LoRa được thống nhất từ Gateway sang Node như sau:
  
| Byte | Ý nghĩa             |
| ---- | ------------------- |
| 0    | 0xAA                |
| 1    | Điều khiển motor    |
| 2    | 0xBB                |

- Byte điều khiển motor là tín hiệu PWM (0 - 10230) gửi sang Node để điều khiển mức cho động cơ máy bơm nước tưới cây.  
