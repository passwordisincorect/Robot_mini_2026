# Robot_mini_2026

Firmware robot 4 bánh mecanum dùng **Arduino Mega 2560**, **ESP32-WROOM-32** (tay cầm PS3 qua UART2), **MPU6050** và **L298N**.

## Phiên bản hiện tại

**[V25_FIX_M2_LED](ROBOT_V25_FIX_M2_LED/)** — bản firmware đã sửa chiều bánh M2, giữ PID giữ hướng V25, phanh điện mềm V24 và bổ sung hai LED báo lệnh hút ở **Mega D50/D52**.

- **[Mã Arduino Mega](ROBOT_V25_FIX_M2_LED/MEGA_PS3_UART_V25/MEGA_PS3_UART_V25.ino)** — mở thư mục sketch trong Arduino IDE (cùng hai file `Mpu6050.h` và `YawPid.h`).
- **[Mã ESP32 PS3 bridge](ROBOT_V25_FIX_M2_LED/ESP32_PS3_BRIDGE_V11/ESP32_PS3_BRIDGE_V11.ino)** — không thay đổi so với bản firmware V25.
- **[Hướng dẫn nối LED và nạp](ROBOT_V25_FIX_M2_LED/HUONG_DAN_LED_HUT_D50_D52.md)**.
- **[Báo cáo kiểm thử](ROBOT_V25_FIX_M2_LED/KET_QUA_KIEM_THU_LED_HUT.md)**.
- **[Bộ test host](ROBOT_V25_FIX_M2_LED/tests/)** — kiểm thử logic bằng C++ mocks; không dùng thiết bị thật.

Bố trí bánh, nhìn từ trên xuống (đầu robot phía trên):

```text
M1   M4
M2   M3
```

`REVERSE={false,false,false,true}`. Bơm hút D28; hai LED báo lệnh hút active-HIGH D50 và D52.

## Kiểm thử và an toàn

Bản phân phối trên PC đã biên dịch Mega 2560 và đạt **73/73 nhóm kiểm thử host** và bridge UART mô phỏng. **Chưa xác minh phần cứng thực tế**, nên kết quả này không bảo đảm robot chạy thẳng, PID đúng dấu hay dòng phanh nằm trong giới hạn L298N.

Kê bánh khỏi sàn khi kiểm tra chiều quay. Đấu LED 5V qua điện trở riêng (ví dụ 330 Ω), không cấp đèn 12/24V trực tiếp từ chân Mega. Có công tắc ngắt nguồn động lực độc lập. Tắt PID khi kiểm tra hướng bánh lần đầu; chỉ bật thử PID sau khi xác nhận chiều phản hồi.

Các file HEX biên dịch và log tại máy phát triển không được đưa vào repository; biên dịch lại từ mã nguồn trong Arduino IDE.
