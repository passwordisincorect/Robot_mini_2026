# LED báo hút D50 và D52

Hướng dẫn điều khiển, UART HWT901B và PID hiện tại: [HUONG_DAN_HWT901B.md](HUONG_DAN_HWT901B.md). Bản này dùng yaw qua STM32 → Mega RX3 D15, không còn đọc MPU6050 I2C trên Mega.

## Hoạt động

| Trạng thái | D28 bơm | D50 LED1 | D52 LED2 |
|---|---|---|---|
| Bật nguồn / chưa ARM | LOW | LOW | LOW |
| R1 bấm lần đầu | HIGH | HIGH | HIGH |
| Thả/giữ R1 sau khi bật | HIGH | HIGH | HIGH |
| R1 thả rồi bấm lần hai | LOW | LOW | LOW |
| R3 hoặc mất PS3/UART2 | LOW | LOW | LOW |

L1/L2/R2 để trống. R1 lần hai tắt bơm và mở van D40 active-HIGH trong 0,5 giây. Hai LED tắt suốt lúc xả; bấm R1 bật hút lại sẽ đóng van. R3/mất PS3 tắt cả bơm, van và LED. Hai LED thể hiện **lệnh hút của firmware**, không xác nhận bơm thật có nguồn hay đã tạo lực hút. Mất UART3 góc chỉ tắt PID, không tự tắt bơm hay điều khiển tay; R3/mất PS3 vẫn dừng toàn bộ.

## Đấu dây LED

- Mega D50 → điện trở 330Ω → anode LED1; cathode → GND Mega.
- Mega D52 → điện trở 330Ω → anode LED2; cathode → GND Mega.

Mỗi LED một điện trở riêng. Không nối thẳng LED vào nguồn hoặc dùng chân Mega kéo trực tiếp đèn công suất/cuộn relay. D50 là MISO, D52 là SCK: nếu bổ sung thiết bị SPI, phải xử lý xung đột chân; bản hiện tại nhận PS3 bằng UART2 và yaw bằng UART3.

Mở sketch với `Hwt901b.h` và `YawPid.h` cùng thư mục, chọn Arduino Mega 2560. Bản đổi UART chỉ cần nạp Mega nếu STM32 đã chuyển tiếp đúng khung WIT 11 byte/115200 như giao thức đã thống nhất; không cần đổi ESP32. Không có thao tác nạp tự động trong lần sửa này.

Thử khi ngắt nguồn động lực: hai LED phải tắt lúc khởi động, sáng sau R1, không chớp/bật tắt lặp khi giữ R1, tắt sau khi thả rồi bấm R1 lần nữa; R3/mất PS3 cũng phải tắt.

## Kiểm thử và hoàn tác

Báo cáo hiện tại: [KET_QUA_V26_3_20261107.md](KET_QUA_V26_3_20261107.md). Báo cáo LED 73 nhóm và đổi nút 74 nhóm là các giai đoạn lịch sử trước khi thay nguồn góc.

- Sau đổi nút, trước HWT: `../ROBOT_V25_SAU_DOI_NUT_TRUOC_HWT_20261107.zip`.
- Trước đổi nút: `../ROBOT_V25_FIX_M2_LED_TRUOC_DOI_NUT_20261107.zip`.
