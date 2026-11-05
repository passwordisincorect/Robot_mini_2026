# Kiểm tra V25 — LED hút D50 + D52

## Phạm vi thay đổi
1. Định nghĩa `SUCTION_LED1=50`, `SUCTION_LED2=52`, active HIGH.
2. Khởi động hai chân OUTPUT/LOW, không gây sáng khi chưa ARM.
3. Hai chân D50/D52 luôn theo lệnh `suctionOn` cùng D28.
4. `stopTools()` tắt đồng thời bơm D28, LED D50/D52 và van D40.
5. Giữ nguyên cấu hình 4 bánh `REVERSE={false,false,false,true}`, PID V25 và phanh V24.
6. Bổ sung một nhóm test `suction_leds` cho R1/L1, R3, UART invalid, mất liên lạc và mức chân.

## Bằng chứng đã chạy
- Host C++ biên dịch 5/5: Mega, ESP32, yaw, auto, pid_v25.
- Mega 32/32 PASS (verification/mega_group1.log, mega_group2.log).
- ESP32 17/17 PASS; Yaw PID 5/5 PASS; Auto PID 9/9 PASS; PID V25 10/10 PASS.
- **73/73 nhóm phần mềm PASS**, bridge giả lập ESP32→Mega PASS.
- Arduino CLI compile Mega2560: PASS, Flash 20.866/253.952 byte (8%), SRAM tĩnh 1.125/8.192 (13%).
- Cảnh báo unused parameter từ Arduino core không chặn biên dịch.
- Manifest và ZIP kiểm định SHA256 khi đóng gói.

## Giới hạn
Không nạp lên board, không đo điện áp D50/D52, chưa thử LED/bơm thực tế. LED báo trạng thái lệnh hút, không phải phản hồi trạng thái máy bơm. Không push GitHub.
