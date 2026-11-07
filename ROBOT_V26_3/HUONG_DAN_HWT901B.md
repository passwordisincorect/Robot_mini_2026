# Mega + HWT901B qua STM32 — 07/11/2026

## Mở và nạp mã

Mở `MEGA_PS3_UART_V26_3/MEGA_PS3_UART_V26_3.ino`, giữ `Hwt901b.h` và `YawPid.h` cùng thư mục; chọn Arduino Mega 2560. ESP32 PS3 giữ nguyên. Bản này thay hoàn toàn việc Mega đọc MPU6050 qua I2C bằng góc yaw HWT901B qua UART3.

Không có thao tác nạp tự động trong lần sửa này. Khi nạp thủ công, ngắt nguồn động lực và kiểm tra tín hiệu trên giá đỡ trước khi thử sàn.

## UART đã thống nhất

| Đường truyền | Thiết lập |
|---|---|
| ESP32 TX GPIO17 → Mega RX2 D17 | 115200, 8N1, giao thức PS3 hiện tại |
| STM32 TX → Mega RX3 D15 | 115200, 8N1, nguyên khung góc WIT 11 byte |
| GND các bo | Nối chung |

Không dùng RX1 D19 vì D18/D19 đang là công tắc hành trình. Mega chỉ nhận từ STM32; không cần nối TX3 D14 về STM32. Chọn nguồn và mức logic theo đúng phần cứng thực tế; không lấy điện áp cấp HWT901B từ tên module hay ví dụ UART.

Giao thức nhận:

```text
55 53 RL RH PL PH YL YH VL VH SUM
```

Roll/pitch/yaw là int16 little-endian, góc = raw × 180 / 32768 độ. SUM là 8 bit thấp của tổng 10 byte trước đó. Khung ví dụ yaw +45°:

```text
55 53 00 00 00 00 00 20 00 00 C8
```

Đây là khung giả lập để kiểm tra giao thức. Công thức và checksum theo [WITMOTION Standard Communication Protocol](https://wit-motion.gitbook.io/witmotion-sdk/wit-standard-protocol/wit-standard-communication-protocol).

Mega chỉ cập nhật hướng bằng khung `0x53` checksum đúng. Khung loại khác không làm mới yaw. Bộ đọc xử lý nhiễu/mất byte, giới hạn 64 byte mỗi vòng và trả từng mẫu góc cho PID; không chờ đọc đủ khung bằng vòng lặp chặn. Mảnh khung gián đoạn 30 ms bị bỏ; yaw quá 200 ms bị coi là cũ.

STM32 cần chuyển tiếp góc mới đều đặn, khuyến nghị cấu hình HWT xuất góc 50 Hz. Không gửi lặp khung lưu đệm khi cảm biến ngừng cập nhật. Không dùng khung raw MPU V26 20 byte cho bản Mega này.

Mã HAL cầu nối đi kèm nằm trong `STM32_HWT901B_BRIDGE/`; xem `INTEGRATION.md` để ghép vào dự án CubeMX. Đây là module bổ sung cho dự án STM32, không phải dự án CubeMX hoàn chỉnh hay firmware STM32 đã nạp lên bo.

## Nút điều khiển

| Nút | Chức năng |
|---|---|
| Joystick trái | Tiến/lùi/đi ngang theo ánh xạ đã xác nhận |
| Joystick phải X | Xoay, ưu tiên hơn joystick trái |
| D-pad phải/trái | Giữ để trượt cơ cấu phải/trái, thả để dừng |
| D-pad lên/xuống | Giữ để nâng/hạ cơ cấu; thả để dừng, hủy lệnh tự động đang chạy |
| Vuông/X | Bấm một lần nâng/hạ tới công tắc hành trình, PWM255 |
| Vuông + X | Hủy lệnh nâng/hạ; cần lần bấm mới để chạy lại |
| R1 | Bấm bật hút; thả rồi bấm lần nữa tắt hút và xả D40 trong 0,5 giây |
| L1/L2/R2 | Để trống, không đổi tốc độ hay xả van |
| R3 | Dừng bánh/cơ cấu, hủy nâng/hạ, tắt hút, van D40 và PID |

D28 bơm và LED D50/D52 cùng theo trạng thái hút. Van D40 active-HIGH: R1 chuyển từ hút ON sang OFF sẽ kích HIGH 500 ms rồi về LOW. Giữ nút không lặp xả. Bấm R1 bật hút lại sẽ đóng van trước khi bật bơm; R3/mất PS3 hủy xả ngay. Timer không dùng delay, không chặn điều khiển khác. Công tắc NC mở sẽ chặn chiều đi vào hành trình ngay; phải đóng ổn định 20 ms mới cho phép chạy lại. Lệnh nâng/hạ đã dừng tại hành trình không tự chạy lại do tiếp điểm rung.

V26.3 giữ tốc độ chạy/trượt/nâng mặc định PWM255; xoay 70% ≈ PWM179/255. Khi đi ngang hai chiều, thêm bù tiến ban đầu 3% để giảm trôi về đuôi. `STRAFE_FORWARD_TRIM_PERCENT` ở đầu sketch là mức hiệu chỉnh theo phép đo thực tế; không phải phản hồi vị trí tự động. Khi bù hướng, phần mềm dành khoảng PWM để tăng/giảm hai nhóm bánh mà không cắt cụt lệnh bù. Vì thế một số bánh giảm PWM khi PID đang chỉnh lệch.

## Phanh V26.3

Phanh thường giữ PWM85 → 170 trong 120 ms như V26.2 (V26.1 là 70 → 150). Hai chân hướng motor giữ LOW; không phát xung đảo chiều để phanh. R3/mất PS3 giữ PWM255 như trước, đã là giới hạn tối đa.

Cần thử dòng/nhiệt driver và quãng dừng thực tế sau khi tăng phanh; chưa có phép đo phần cứng trong lần sửa này. [Datasheet ST L298](https://www.st.com/resource/en/datasheet/l298.pdf) nêu trạng thái enable HIGH với hai input bằng nhau là fast motor stop và quy định giới hạn dòng khi phanh.

## Giữ hướng

- HWT901B tính góc. Mega lấy yaw, tính sai lệch ngắn nhất qua ±180°, tốc độ đổi yaw và PID. Không tích phân gyro MPU thô nữa.
- PID tự bật một lần sau khi có ít nhất hai mẫu yaw theo thời gian, yaw còn mới, robot ARM, nút/joystick trung tính và tốc độ đổi yaw nhỏ. Đây là điều kiện sẵn sàng dữ liệu, không phải hiệu chuẩn cảm biến trên Mega.
- Bắt đầu chạy thẳng/ngang sẽ chốt yaw đích. Giữ cùng đích khi đổi tiến/lùi/ngang liên tục; dừng, phanh hoặc xoay tay sẽ xóa đích để lần chạy sau lấy hướng mới.
- Khâu I có giới hạn và chống tích lũy khi bão hòa; lệnh bù có giới hạn và tốc độ tăng/giảm. Giới hạn auto 10 PWM, thủ công 25 PWM. Thông số nằm ở đầu `YawPid.h`.
- Quá 200 ms không có góc mới hợp lệ: tắt PID, xóa đích/I và điều kiện sẵn sàng. Dữ liệu trở lại không tự bật lại PID nếu lượt auto đã dùng; đưa robot về trạng thái đứng yên rồi dùng `PID ON` để bật lại.
- R3, mất PS3 hoặc `PID OFF` cũng hủy lượt auto. Mất PS3 200 ms vẫn dừng toàn bộ và tắt hút như trước.

Khi PID tắt/chưa sẵn sàng, joystick vẫn điều khiển bánh theo lệnh tay, không có bù hướng. Không xem trạng thái UART PS3 còn kết nối là bằng chứng yaw còn hợp lệ.

Serial Monitor 115200 nhận `PID STATUS`, `PID OFF`, `PID ON`; theo dõi `IMU=1 TYPE=53 YAW=...`, `YAW_OK`, `C` (PWM bù) và `H10` (sai lệch góc ×10).

## Kiểm tra dấu và hiệu chỉnh trên robot

`YAW_SIGN=1` giả định quay trái làm yaw tăng. Bánh đã giữ nguyên ánh xạ chiều cũ; PWM bù dương tạo xoay phải. Trên giá đỡ, kiểm tra quay trái/phải, dấu yaw và phản ứng bù có kéo về hướng đích. Nếu hướng lắp cảm biến làm dấu ngược, đổi `YAW_SIGN` sau khi đo xác nhận; không đổi đồng thời chiều motor và dấu yaw khi chưa xác định nguyên nhân.

Hiệu chuẩn HWT bằng quy trình của đúng cảm biến; kiểm tra yaw khi động cơ và bơm chạy vì từ trường có thể làm thay đổi yaw. Thông số PID hiện tại là cấu hình có giới hạn để thử nghiệm, chưa được chỉnh theo tải/độ bám robot. Giữ hướng yaw không đo được sai lệch vị trí ngang do bánh trượt; không thể cam kết đường đi không lệch chỉ từ kiểm thử phần mềm.

## Hoàn tác

Bản sau đổi nút nhưng trước HWT: `../ROBOT_V25_SAU_DOI_NUT_TRUOC_HWT_20261107.zip`. Bản trước đổi nút: `../ROBOT_V25_FIX_M2_LED_TRUOC_DOI_NUT_20261107.zip`. Giải nén vào thư mục riêng để so sánh/khôi phục, không ghi đè thiết kế đang dùng một cách mù quáng.
