# Đổi nút và tốc độ — 07/11/2026

Báo cáo lịch sử trước V26.3; bản hiện tại xem [KET_QUA_V26_3_20261107.md](KET_QUA_V26_3_20261107.md).

Báo cáo lịch sử trước V26.2; bản hiện tại xem [KET_QUA_V26_2_20261107.md](KET_QUA_V26_2_20261107.md).

Báo cáo lịch sử trước V26.1; bản hiện tại xem [KET_QUA_V26_1_20261107.md](KET_QUA_V26_1_20261107.md).

Báo cáo giai đoạn trước HWT901B. Giao thức đã được người dùng chốt và trạng thái BLOCKED dưới đây đã được giải quyết bằng bản UART góc; xem [KET_QUA_HWT901B_20261107.md](KET_QUA_HWT901B_20261107.md) và hướng dẫn HWT901B hiện tại.

Trạng thái: **PASS phần điều khiển nút/tốc độ; BLOCKED phần UART STM32 do cần chốt loại dữ liệu.**

## Mã đã sửa

- D-pad phải/trái: giữ để trượt phải/trái, thả để dừng.
- Vuông/X: bấm một lần nâng/hạ tới công tắc hành trình, PWM255. Nhấn cả hai hủy lệnh; sau khi chạm hành trình phải có lần bấm mới để chạy lại.
- R1: lần bấm đầu bật hút, lần tiếp theo tắt; giữ không lặp. D28/D50/D52 đồng bộ trạng thái hút.
- L1/L2/R2 để trống; bỏ chỉnh tốc độ và van xả L1. Tạm hiểu yêu cầu ghi L1 hai lần là L1/L2/R2. Van D40 luôn OFF; R1 tắt bơm không mở van.
- Chạy/xoay/trượt mặc định PWM255; nâng/hạ PWM255. PID giữ nguyên có thể giảm PWM một số bánh để bù hướng.
- Giữ hành trình NC với chống rung đóng 20 ms, R3, timeout PS3 200 ms và phanh 120 ms. Không thay chiều bốn motor.

## Kiểm chứng thực sự đã chạy

- Bản gốc trước sửa: 73/73 nhóm giả lập PASS.
- Bản đã sửa: `python tests/run_tests.py` — **74/74 nhóm PASS**, gồm Mega 33, ESP32 17, Yaw 5, auto PID 9, PID V25 10.
- Kiểm tra tích hợp ESP32 → Mega và CRC bằng Python độc lập: PASS.
- Bằng chứng toàn bộ lần chạy cuối: `verification/native_tests.log`.
- Arduino CLI `compile --fqbn arduino:avr:mega --warnings all`: PASS sau thay đổi cuối cùng của sketch. Flash 20.266/253.952 byte, RAM tĩnh 1.109/8.192 byte.
- Đã đọc soát thay đổi nguồn và các kỳ vọng test; sửa lại kỳ vọng tốc độ/PWM và ánh xạ nút còn cũ, rồi chạy lại toàn bộ suite tới PASS.

Phân công: worker explorer đọc soát điều khiển và mã UART V26; worker implementer cập nhật `test_mega.cpp`/`run_tests.py`; manager sửa nguồn, rà diff, hoàn thiện test và tự chạy xác minh toàn bộ.

## UART STM32 còn chờ

Đã đối chiếu mã V26 tại `Robot_mini_2026_V26_UART_ENCODER_PS3/Robot_mini_2026-main/ROBOT_V26`: STM32 gửi 6 số MPU6050 thô qua USART1 PA9, 115200 8N1, khoảng 50 Hz; khung binary 20 byte `C3 3C`, sequence và CRC16-CCITT-FALSE. Khung này không có trường góc yaw.

Người dùng đồng thời mô tả STM32 chỉ gửi góc. Cần chốt dùng raw V26 (Mega tự tính góc/PID), hay một firmware gửi yaw đã tính (cần mã/khung mẫu, đơn vị, dấu và quy tắc reset góc). **Chưa thêm bộ đọc UART STM32; Mega hiện vẫn dùng MPU6050 trực tiếp và PID cũ.** Không coi test MPU/PID hiện tại là bằng chứng cho giao tiếp STM32.

## Giới hạn và hoàn tác

- Chưa nạp Mega/STM32, chưa thử robot, công tắc, bơm/van hay dấu bù thực tế. Chưa xác nhận robot đi thẳng ngoài thực địa.
- `firmware/` vẫn giữ HEX bản gốc. HEX biên dịch của lần sửa nút nằm trong `verification/build_doi_nut/`; nó chưa có UART STM32.
- Không thêm encoder, chức năng van mới hay firmware STM32 mới.
- Bản sao trước sửa: `../ROBOT_V25_FIX_M2_LED_TRUOC_DOI_NUT_20261107.zip`. Giải nén bản sao vào thư mục riêng để đối chiếu/khôi phục; không tự nạp firmware.
