# EMB_PROJECT_MODULE2
Bài tập này chú trọng vào quá trình giao tiếp giữa Node và Gateway. Node có nhiệm vụ thu thập thông tin vầ độ ẩm và nhiệt độ môi trường thông qua cảm biến, hiển thị lên màn hình LCD, điều khiển động cơ máy tưới nước, đồng thời gửi thông tin tới Gateway thông qua LoRa. Gateway có nhiệm vụ thu thập thông tin từ Node, hiển thị lên màn hình LCD, gửi thông tin điều khiển động cơ máy bơm nước (nếu càn) và cập nhật thông tin lên máy tính thông qua giao thức UART, hiển thị trên Hercules hoặc Cutecom.  
  
*Flowchart của hệ thống*  
  
<img width="878" height="673" alt="Screenshot from 2026-08-02 17-49-32" src="https://github.com/user-attachments/assets/41273dcf-ed91-407e-b1e8-b963252f695f" />
  
*Cấu trúc gói tin*  
- Cấu trúc gói tin truyền qua LoRa được thống nhất giữa Node và Gateway gồm 8 byte thông tin:
  
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

