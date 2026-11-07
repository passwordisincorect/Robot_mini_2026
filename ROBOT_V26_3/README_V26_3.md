# ROBOT V26.3 — xả van 500 ms, tiến/lùi PWM255

Mở [MEGA_PS3_UART_V26_3.ino](MEGA_PS3_UART_V26_3/MEGA_PS3_UART_V26_3.ino) cùng `Hwt901b.h` và `YawPid.h`; chọn Arduino Mega 2560. ESP32/STM32 giữ giao thức V26/V26.1.

- R1 lần đầu bật hút: D28 và LED D50/D52 HIGH, van D40 LOW.
- Thả rồi bấm R1 lần hai: tắt hút/LED và kích van D40 HIGH trong 500 ms, sau đó tự LOW. Giữ R1 không kéo dài hoặc lặp xả.
- Bấm R1 bật hút lại trong lúc xả: đóng van trước khi bật bơm. R3, mất PS3 hoặc UART PS3 hết hạn 200 ms hủy xả và tắt hút; không tự chạy lại sau ARM.
- Bộ đếm thời gian không chặn: các thao tác di chuyển, hành trình và UART tiếp tục được xử lý trong lúc xả.
- Phanh thường tăng nhẹ từ 70 → 150 lên **85 → 170 PWM trong 120 ms**. Không đảo chiều motor; phanh R3/mất PS3 vẫn PWM255.

Tiến/lùi giữ `driveSpeed=255` (100% PWM), đã là giới hạn tối đa; không có mức giảm tốc tiến/lùi bằng nút. Khi cần giữ hướng, PID vẫn điều chỉnh từng bánh. Chưa đo vận tốc thực.

Giữ thay đổi V26.1: D-pad lên/xuống giữ nâng/hạ, trái/phải giữ trượt; vuông/X bấm một lần tới hành trình; xoay 70% (PWM179); đi ngang bù tiến ban đầu 3%. L1/L2/R2 vẫn chưa gán chức năng.

**D40 → IN relay van active-HIGH**, nối chung GND và giữ điện trở kéo xuống D40 10k như cấu hình cũ. Khi có lệnh xả, mức D40 phải HIGH rồi LOW sau 0,5 giây. `VAN` trên Serial Monitor phản ánh thời điểm in; dòng trạng thái được luân phiên nên có thể không bắt trúng mọi xung 0,5 giây.

[UART, nút và PID](HUONG_DAN_HWT901B.md) · [LED hút](HUONG_DAN_LED_HUT_D50_D52.md) · [Báo cáo V26.3](KET_QUA_V26_3_20261107.md).

```sh
python ROBOT_V26_3/tests/run_tests.py
arduino-cli compile --fqbn arduino:avr:mega ROBOT_V26_3/MEGA_PS3_UART_V26_3
```

V26.2 và các phiên bản trước được giữ để hoàn tác. GitHub chỉ chứa nguồn/test/tài liệu theo `.gitignore`; HEX/log ở bản phân phối trên máy. Chưa nạp/thử relay, bơm, van hoặc đo dòng/nhiệt phanh trên robot; mức bù ngang 3% vẫn cần hiệu chỉnh thực tế.
