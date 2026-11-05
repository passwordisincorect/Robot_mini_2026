# Robot_mini_2026

Firmware điều khiển **robot 4 bánh mecanum** dùng **Arduino Mega 2560** (động cơ, cơ cấu, bơm/van, MPU6050) và **ESP32-WROOM-32** (nhận tay cầm PS3 qua Bluepad32 rồi gửi dữ liệu UART2 cho Mega).

**Phiên bản trong repository:** [V25](ROBOT_V25/) — sửa chiều bánh M2, giữ điều khiển PID giữ hướng V25 và phanh V24, bổ sung hai LED báo **lệnh hút** ở Mega D50/D52. Chưa xác minh hoạt động trên robot thật.

## Yêu cầu

- **Phần cứng:** Arduino Mega 2560, ESP32-WROOM-32 (ESP32 nguyên bản), tay cầm PS3 tương thích Bluepad32, MPU6050, bốn động cơ/bánh mecanum, driver L298N cho các tải được cấu hình và công tắc hành trình thường đóng (NC).
- **Nạp chương trình:** Arduino IDE, bộ board Arduino AVR cho Mega 2560; môi trường board `esp32_bluepad32` với lựa chọn **ESP32 Dev Module** cho sketch ESP32. Chú thích trong firmware ghi **core 4.1.0**; repository chưa có manifest khóa phiên bản này.
- **Nguồn và an toàn:** cấp nguồn logic đúng định mức module và nguồn động lực phù hợp từng tải; kiểm tra điện áp/dòng thực tế và khả năng chịu tải của driver trước khi bật nguồn động lực. Cần **công tắc cắt nguồn động lực độc lập**; lệnh dừng R3 không thay thế được nút cắt nguồn vật lý.

> **Lưu ý:** Sketch ESP32 kiểm tra chip ESP32 nguyên bản; **không dùng nguyên firmware này cho ESP32-C3/S3**. Không nối TX 5 V của Mega trực tiếp về chân RX 3,3 V của ESP32. Firmware hiện tại chỉ cần đường UART một chiều ESP32 → Mega.

## Cài đặt

1. Tải repository và mở thư mục [`ROBOT_V25/`](ROBOT_V25/).
2. Trong Arduino IDE, mở [`MEGA_PS3_UART_V25.ino`](ROBOT_V25/MEGA_PS3_UART_V25/MEGA_PS3_UART_V25.ino). Giữ [`Mpu6050.h`](ROBOT_V25/MEGA_PS3_UART_V25/Mpu6050.h) và [`YawPid.h`](ROBOT_V25/MEGA_PS3_UART_V25/YawPid.h) **cùng thư mục sketch**. Chọn **Arduino Mega 2560**, kiểm tra biên dịch rồi nạp khi đã ngắt nguồn tải.
3. Trong môi trường `esp32_bluepad32`, chọn **ESP32 Dev Module**, mở và nạp [`ESP32_PS3_BRIDGE_V11.ino`](ROBOT_V25/ESP32_PS3_BRIDGE_V11/ESP32_PS3_BRIDGE_V11.ino) cho ESP32-WROOM-32.
4. Mở Serial Monitor của từng board ở **115200 baud** để đối chiếu thông báo khởi động và trạng thái trước khi cấp nguồn cho tải.

Không có file HEX phát hành hoặc script cài đặt tự động trong repository; biên dịch lại từ mã nguồn. **Quy trình ghép đôi PS3 bằng USB chưa được đóng gói đầy đủ**: sketch ESP32 xuất `Bluetooth MAC:` để phục vụ quá trình ghép tay cầm với đúng địa chỉ Bluetooth, nhưng repository chưa cung cấp công cụ/hướng dẫn chi tiết cho từng loại tay PS3.

## Cấu hình và đấu nối

### Bố trí bánh và chân Mega

Nhìn từ trên xuống, đầu robot ở phía trên:

```text
    ĐẦU ROBOT
     M1   M4
     M2   M3
    ĐUÔI ROBOT
```

| Bánh | Vị trí | PWM/EN | IN A | IN B | `REVERSE` |
| --- | --- | ---: | ---: | ---: | --- |
| M1 | Trước trái | D5 | D30 | D31 | `false` |
| M2 | Sau trái | D6 | D32 | D33 | `false` |
| M3 | Sau phải | D7 | D34 | D35 | `false` |
| M4 | Trước phải | D8 | D37 | D36 | `true` |

Đây là **cấu hình logic trong code**, không phải xác nhận chiều quay ngoài thực tế. Nâng bánh khỏi sàn và kiểm tra từng bánh trước khi cho robot chạy.

### Cơ cấu, hành trình, bơm và van

| Chức năng | Chân Mega | Chi tiết trong firmware |
| --- | --- | --- |
| Nâng/hạ | EN D10, A D39, B D41 | Điều khiển động cơ hai chiều |
| Trượt trái/phải | EN D9, A D43, B D45 | Điều khiển động cơ hai chiều |
| Hành trình trên / dưới | D18 / D19 | Tiếp điểm **NC** tới GND, `INPUT_PULLUP` |
| Hành trình trái / phải | D26 / D27 | Tiếp điểm **NC** tới GND, `INPUT_PULLUP` |
| Bơm hút | D28 | Điều khiển **ENA** kênh A của L298N, active-HIGH |
| Relay van xả | D40 | Lệnh active-HIGH trong phiên bản này |
| LED báo lệnh hút | D50 / D52 | Active-HIGH, cùng trạng thái lệnh D28 |
| MPU6050 | SDA D20 / SCL D21 | I²C; kiểm tra mức điện áp module và chuyển mức phù hợp |
| UART từ ESP32 | RX2 D17 | Nhận ESP32 GPIO17 TX, 115200 8N1 |

- Nối **ESP32 GPIO17 TX → Mega D17 RX2**, đồng thời nối GND tín hiệu hai board. Không đấu TX Mega sang ESP32 nếu chỉ dùng firmware một chiều hiện tại.
- Công tắc hành trình dùng **COM → GND, NC → chân Mega** tương ứng. Khi tiếp điểm mở/dây đứt, chương trình coi hướng tương ứng là bị chặn; chỉ cho phép trở lại sau khi tiếp điểm đóng ổn định.
- Theo cấu hình bơm trong sketch: **Mega D28 → ENA L298N** (tháo jumper ENA), **IN1 → 5 V logic**, **IN2 → GND logic**, thêm **10 kΩ từ ENA xuống GND** để giữ OFF lúc reset. Nếu cấp 5 V logic ngoài cho module, kiểm tra và tháo jumper `5V-EN` đúng theo loại module trước khi đấu.
- Relay van dùng D40 active-HIGH trong code; kiểm tra mức kích thực tế và thêm **10 kΩ từ D40 xuống GND** theo ghi chú firmware. Phần tải của van đi qua tiếp điểm relay phù hợp điện áp và dòng tải.
- LED rời: **D50 → 330 Ω → anode LED1**, cathode → GND; **D52 → 330 Ω → anode LED2**, cathode → GND. Mỗi LED có điện trở riêng. D50/D52 cũng là chân SPI (MISO/SCK), cần đổi cách đấu nếu sau này dùng SPI. Không nối trực tiếp đèn 12/24 V hay cuộn relay vào chân Mega.

Xem [hướng dẫn chi tiết LED và kiểm tra tín hiệu](ROBOT_V25/HUONG_DAN_LED_HUT_D50_D52.md). **Không** coi LED sáng là bằng chứng bơm đã có nguồn, chạy hoặc tạo được lực hút.

## Bắt đầu nhanh và kiểm tra an toàn

1. **Ngắt nguồn động lực**, kiểm tra cực nguồn, dây chung GND tín hiệu, jumper L298N, các công tắc NC và đường UART; giữ bánh khỏi sàn.
2. Nạp hai sketch. Trên ESP32, xem `Bluetooth MAC:` ở Serial Monitor 115200; thực hiện ghép tay PS3 bằng công cụ USB tương thích của tay cầm. Chỉ tiếp tục khi ESP32 hiển thị trạng thái `PS3=1` hoặc có báo cáo nút/joystick hợp lệ.
3. Trên Mega, xem các dòng giám sát `UART_PS3=` và `ARM=` ở 115200. Khi đã nhận khung hợp lệ và thả các nút/joystick về trung tính liên tục tối thiểu **100 ms**, firmware có thể ARM.
4. **Trước lần thử chiều bánh, gửi `PID OFF` qua Serial Monitor Mega** và kiểm tra `PID STATUS` hiển thị `PID=0`. Việc này ngăn cơ chế tự bật PID trong chu kỳ nguồn hiện tại. Không giả định PID luôn OFF chỉ vì lúc boot biến `enabled` khởi tạo `false`.
5. Sau khi kiểm tra chiều bánh, hành trình, relay và bơm ở trạng thái không tải/được cô lập phù hợp, mới cấp nguồn động lực và thử từng chức năng có kiểm soát. Thử R3 và ngắt liên lạc để xác nhận tải ngừng theo logic phần mềm; nút cắt nguồn vật lý vẫn phải hoạt động độc lập.

**Mốc an toàn trong code:** Mega hủy lệnh điều khiển nếu không nhận khung UART hợp lệ trong khoảng **200 ms**; đây là điều kiện dừng phần mềm, **không** thay thế ngắt nguồn động lực.

## Điều khiển bằng tay PS3

| Điều khiển | Chức năng theo firmware |
| --- | --- |
| Joystick trái | Tiến/lùi, dịch ngang mecanum |
| Joystick phải (trục X) | Xoay robot |
| D-pad lên / xuống (giữ) | Nâng/hạ bằng tay; hủy tự động |
| D-pad phải / trái (nhấn) | Tự nâng đến công tắc trên / tự hạ đến công tắc dưới |
| Tròn / Vuông | Trượt phải / trượt trái |
| R1 | Nhấn để bật **lệnh hút duy trì**; thả R1 không tự tắt |
| L1 | Tắt lệnh hút, bật lệnh van xả khoảng **1 giây** |
| R3 | Khóa/dừng điều khiển, tắt bơm/van; thả nút và trả joystick về trung tính để ARM lại |
| L2 / R2 | Điều chỉnh tốc độ theo logic trong sketch |

Chân **D28** và LED **D50/D52** cùng phản ánh biến lệnh `suctionOn`. Không dùng LED làm phản hồi cảm biến cho trạng thái bơm thật.

## PID giữ hướng và phanh

- **PID khởi động OFF, nhưng có thể tự ON:** sau khi MPU6050 đủ mẫu hiệu chuẩn, kết nối PS3 đã ARM và các điều khiển trung tính, firmware có thể tự bật PID một lần trong chu kỳ cấp nguồn. Giới hạn hiệu chỉnh ở chế độ tự động là **10 PWM**; chế độ bật thủ công giới hạn **25 PWM** trước các giới hạn khác trong code.
- Dùng Serial Monitor **115200** trên Mega để gửi đúng các lệnh sau (mỗi lệnh một dòng):

  ```text
  PID STATUS
  PID OFF
  PID ON
  ```

  `PID OFF` vô hiệu hóa PID và hủy lần tự bật còn chờ; `PID ON` chỉ được chấp nhận khi điều kiện ARM, trung tính, hiệu chuẩn và cảm biến an toàn thỏa mãn. `PID STATUS` giúp kiểm tra `PID`, `AUTO`, `WAIT`, `CAL` và mức hiệu chỉnh.
- Code dự kiến **trục Z của MPU6050 hướng lên** và dùng dấu tốc độ góc để bù yaw. **Chưa kiểm chứng dấu phản hồi trên robot thật**: chỉ thử PID sau khi xác nhận chiều bánh và chiều lệch, khi robot được kê an toàn.
- Khi nhả joystick, firmware tạo nhịp phanh điện **120 ms**, tăng mức PWM EN từ **70 lên 150** khi hai chân hướng ở LOW; không phát lệnh đảo chiều chủ động để phanh. Mức phanh và dòng/tỏa nhiệt thực tế của L298N chưa được đo. R3 hoặc lỗi liên lạc đưa lệnh về trạng thái dừng theo firmware nhưng không bảo đảm dừng cơ khí tức thì.

## Kiểm thử

[Báo cáo đi kèm V25](ROBOT_V25/KET_QUA_KIEM_THU_LED_HUT.md) ghi **73/73 nhóm test host PASS** (Mega 32, ESP32 17, Yaw 5, Auto PID 9, PID V25 10), kiểm tra bridge UART mô phỏng và biên dịch Mega 2560 bằng Arduino CLI PASS. Đây là **kết quả được ghi trong báo cáo**, không phải kết quả CI hoặc lần chạy độc lập mới nhất.

Bộ kiểm thử dùng C++ stubs/mock, không cần kết nối board: [`tests/run_tests.py`](ROBOT_V25/tests/run_tests.py). Có thể chạy từ thư mục gốc repository nếu máy có **Python** và `g++` trong PATH:

```bash
python ROBOT_V25/tests/run_tests.py
```

Script sinh chương trình test và log tại `ROBOT_V25/verification/` (được `.gitignore` bỏ qua). Repository **không chứa log/HEX đầu ra** để xác minh lại tuyên bố PASS chỉ bằng tải mã nguồn. Chưa có CI workflow được lưu trong nhánh `main`.

**Chưa xác minh trên thiết bị thật:** ghép đôi PS3 cụ thể, điện áp D50/D52, chức năng bơm/van, chiều thực tế bốn bánh, hướng phản hồi PID, lực/dòng phanh và giới hạn dòng/nhiệt driver.

## Tài liệu và phiên bản

- [Firmware Mega — V25](ROBOT_V25/MEGA_PS3_UART_V25/MEGA_PS3_UART_V25.ino)
- [Firmware ESP32 PS3 bridge — V11](ROBOT_V25/ESP32_PS3_BRIDGE_V11/ESP32_PS3_BRIDGE_V11.ino)
- [Hướng dẫn LED hút, sơ đồ đấu và cách thử](ROBOT_V25/HUONG_DAN_LED_HUT_D50_D52.md)
- [Báo cáo kiểm thử V25](ROBOT_V25/KET_QUA_KIEM_THU_LED_HUT.md)
- [Bộ test host và các mock](ROBOT_V25/tests/)

**Giới hạn khả năng khôi phục:** các chú thích mã nguồn có nhắc `HUONG_DAN.md`, nhưng file này không được đưa lên repository. Bản firmware cũ `V25_FIX_M2` được nhắc trong tài liệu LED cũng **không có trong nhánh `main`**; đường dẫn trên máy phát triển trong hướng dẫn đó không phải phương án rollback khả dụng cho người tải repository. Trước khi nạp, hãy giữ bản firmware đang chạy ổn định của robot để có thể khôi phục nếu cần.

## Giấy phép

Dự án sử dụng [MIT License](LICENSE).
