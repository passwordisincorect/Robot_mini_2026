# ROBOT V26 — HWT901B UART + PID giữ hướng

Mở [MEGA_PS3_UART_V26.ino](MEGA_PS3_UART_V26/MEGA_PS3_UART_V26.ino), giữ `Hwt901b.h` và `YawPid.h` cùng thư mục; chọn Arduino Mega 2560. ESP32 giữ bản bridge V11.

- STM32 chuyển tiếp nguyên khung góc WIT 11 byte `55 53 ... SUM` → Mega D15/RX3, 115200 8N1.
- Mega kiểm checksum, xử lý vòng góc ±180°, chốt hướng đích và PID khi tiến/lùi/đi ngang.
- D-pad phải/trái trượt; R1 bấm bật/tắt hút; vuông/X bấm một lần nâng/hạ tới hành trình.
- L1/L2/R2 để trống; van OFF. Tốc độ đặt PWM255; PID dành khoảng PWM để bù hướng.
- R3/mất PS3 dừng toàn bộ. Mất góc 200 ms tắt PID; điều khiển tay vẫn hoạt động.

[Hướng dẫn UART, nút và PID](HUONG_DAN_HWT901B.md) · [LED báo hút](HUONG_DAN_LED_HUT_D50_D52.md) · [Module STM32 HAL](STM32_HWT901B_BRIDGE/INTEGRATION.md) · [Báo cáo kiểm thử](KET_QUA_HWT901B_20261107.md).

Kiểm thử từ thư mục gốc repository:

```sh
python ROBOT_V26/tests/run_tests.py
arduino-cli compile --fqbn arduino:avr:mega ROBOT_V26/MEGA_PS3_UART_V26
```

Kết quả kiểm chứng: 80/80 nhóm giả lập PASS, Mega 2560 build PASS. Đổi tên V26 không đổi thuật toán điều khiển. File HEX/log sinh khi chạy kiểm chứng được giữ tại bản phân phối trên máy; repository tuân theo `.gitignore` hiện có, chỉ chứa mã nguồn, test và tài liệu. Manifest SHA256 trong thư mục này áp dụng cho các file nguồn/tài liệu được đưa lên GitHub.

V25 trong repository là bản trước để đối chiếu và hoàn tác. Chưa nạp/chạy robot; cần xác nhận dấu yaw và tinh chỉnh PID theo tải thật. Module HAL STM32 cần ghép vào dự án CubeMX, không phải project hoặc HEX STM32 hoàn chỉnh.
