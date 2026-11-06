# ROBOT V26.1 — nâng/hạ D-pad, xoay 70%, bù trôi khi đi ngang

Mở [MEGA_PS3_UART_V26_1.ino](MEGA_PS3_UART_V26_1/MEGA_PS3_UART_V26_1.ino) cùng `Hwt901b.h` và `YawPid.h`; board Arduino Mega 2560. ESP32 và giao thức yaw STM32 giữ như V26.

| Điều khiển | V26.1 |
|---|---|
| D-pad lên/xuống | Giữ để nâng/hạ PWM255; thả dừng |
| D-pad phải/trái | Giữ để trượt cơ cấu phải/trái |
| Vuông/X | Bấm một lần nâng/hạ tới hành trình |
| R1 | Bấm bật/tắt hút |
| Joystick phải X | Xoay 70%, PWM179/255 |
| Joystick trái X | Đi ngang với bù tiến ban đầu 3% ở cả hai chiều |
| L1/L2/R2 | Chưa gán chức năng |

D-pad lên/xuống ưu tiên điều khiển tay và hủy lệnh nâng/hạ tự động; thả nút không tự chạy tiếp lệnh cũ. Giữ cả lên và xuống sẽ dừng nâng/hạ. Hành trình NC, R3 và timeout PS3 vẫn có hiệu lực. Van D40 luôn OFF như V26.

Robot được báo giữ nguyên đầu nhưng trôi chéo về đuôi khi đi ngang trái/phải. Bản này thêm một thành phần tiến, không đổi PID yaw. Giá trị khởi đầu `STRAFE_FORWARD_TRIM_PERCENT=3` cho khoảng 8 PWM bù tiến ở tốc độ đặt 255. Dành khoảng PWM cho cả bù tiến và bù yaw để không cắt cụt lệnh ở giới hạn 255. Đi tiến/lùi và xoay không dùng bù này.

Mức bù 3% cần thử và chỉnh theo quãng đường đo được; PWM không tỉ lệ chính xác với tốc độ trên mọi tải/sàn. Nếu còn trôi về đuôi tăng nhẹ mức bù; nếu trôi về đầu giảm mức bù, có thể đặt 0 để tắt. Không suy ra đã hết lệch từ kiểm thử giả lập.

[Đấu UART và PID](HUONG_DAN_HWT901B.md) · [LED hút](HUONG_DAN_LED_HUT_D50_D52.md) · [Báo cáo V26.1](KET_QUA_V26_1_20261107.md).

```sh
python ROBOT_V26_1/tests/run_tests.py
arduino-cli compile --fqbn arduino:avr:mega ROBOT_V26_1/MEGA_PS3_UART_V26_1
```

V26 và V25 được giữ nguyên trong repository để đối chiếu/hoàn tác. GitHub chỉ lưu mã, test và tài liệu theo `.gitignore`; HEX/log ở bản phân phối trên máy. Chưa nạp hoặc thử robot trong lần sửa này.
