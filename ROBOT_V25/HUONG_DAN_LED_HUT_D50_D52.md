# ROBOT V25 — 2 đèn báo hút D50 và D52

## Chức năng

Phát triển từ bản **V25_FIX_M2**. Giữ nguyên Arduino Mega, ESP32 cầu nối PS3, PID V25, phanh 120 ms và chiều bốn bánh:
```
     ĐẦU ROBOT
      M1  M4
      M2  M3
     ĐUÔI ROBOT
REVERSE = {false,false,false,true}
```

Hai chân D50 và D52 làm hai ngõ ra LED báo hút, mức kích **HIGH**.

| Trạng thái | D28 bơm | D50 LED1 | D52 LED2 |
|---|---|---|---|
| Mới bật nguồn / chưa ARM | LOW | LOW | LOW |
| R1 kích hút | HIGH | HIGH | HIGH |
| Thả R1, tiếp tục hút | HIGH | HIGH | HIGH |
| L1 kích van xả, tắt hút | LOW | LOW | LOW |
| R3 dừng khẩn | LOW | LOW | LOW |
| Gói PS3/UART lỗi hoặc mất kết nối | LOW | LOW | LOW |

D28 vẫn là lệnh bật bơm; hai LED chỉ báo trạng thái **lệnh điều khiển hút** của firmware, không chứng minh bơm thực sự có nguồn, đang chạy hay đã tạo lực hút. Khi lỗi điện ngoài hoặc bơm kẹt, LED có thể vẫn sáng.

## Đấu dây

- **Mega D50 → điện trở 330Ω → cực dương/anode LED1; cực âm/cathode LED1 → GND Mega.**
- **Mega D52 → điện trở 330Ω → cực dương/anode LED2; cực âm/cathode LED2 → GND Mega.**

Mỗi LED một điện trở riêng; 220–1.000 Ω cũng thường dùng cho LED nhỏ 5V với dòng phù hợp. Không nối thẳng LED vào 5V/24V và không dùng D50/D52 để kéo trực tiếp đèn công suất hay cuộn relay. Nếu dùng đèn báo 12V/24V, cần mạch đệm transistor/MOSFET và điện trở phù hợp.

**D50 = MISO, D52 = SCK** là chân SPI trên Mega 2560. Firmware hiện hành giao tiếp PS3 bằng UART2 và MPU6050 bằng I²C, không sử dụng SPI trên hai chân này; nếu về sau thêm thiết bị SPI, cần chuyển hai LED sang chân khác.

## Thử an toàn

1. Ngắt nguồn động lực trước khi đấu LED/nạp Mega. Đấu điện trở riêng cho mỗi LED và nối chung GND Arduino.
2. Mở `MEGA_PS3_UART_V25/MEGA_PS3_UART_V25.ino` trong Arduino IDE, giữ `Mpu6050.h` và `YawPid.h` cùng thư mục. Chọn **Arduino Mega 2560**.
3. Chỉ nạp Mega, không cần nạp lại ESP32.
4. Khi mới bật nguồn hai LED phải tắt. Sau khi PS3 kết nối và Mega ARM, nhấn R1 để bật hút, cả hai LED phải sáng và vẫn sáng sau khi nhả R1.
5. Nhấn L1: cả hai LED phải tắt ngay; R3 hoặc mất UART cũng phải tắt. Chỉ bật nguồn bơm thật sau khi kiểm tra tín hiệu phần mềm và nối dây đúng.
6. Nếu LED sáng ngược, kiểm tra đúng loại module LED (LED rời active-HIGH so với module active-LOW), không đảo tùy tiện nguồn 5V/24V.

## Phần thay đổi trong code

```cpp
const uint8_t SUCTION_LED1=50, SUCTION_LED2=52;
```

- Trong `setup()`: đặt LOW trước khi cấu hình `OUTPUT`, tránh sáng khi khởi động.
- Trong `toolsTask()`: cả hai LED ghi cùng mức `suctionOn` như D28.
- Trong `stopTools()`: đặt D28, D50, D52 LOW khi dừng/ngắt điều khiển.
- Không thay R1/L1, van D40, PID, phanh hay chiều bánh.

## Báo cáo kiểm thử

- Build host C++: 5/5 nhóm biên dịch PASS (mega, esp32, yaw, auto, pid_v25).
- Mega 32/32 PASS, trong đó bài mới `suction_leds` thử khởi động, R1 bật hút, L1 tắt hút, R3, mất kết nối và timeout.
- ESP32 17/17 PASS; PID thủ công 5/5 PASS; PID auto 9/9 PASS; PID nâng cao 10/10 PASS.
- **Tổng 73/73 nhóm giả lập PASS**, test bridge ESP32→Mega PASS.
- Arduino CLI `arduino:avr:mega` compile PASS: 20.866 byte flash (8%); 1.125 byte RAM tĩnh (13%).
- **Chưa nạp lên Mega, chưa kiểm tra LED/bơm thật.**

## Hoàn tác

Bản gốc trước khi thêm LED vẫn giữ tại:
`D:\Users\ngoti\Downloads\ROBOT_V25_FIX_M2\MEGA_PS3_UART_V25\MEGA_PS3_UART_V25.ino`

Ngắt nguồn động lực và nạp lại firmware gốc khi cần.
