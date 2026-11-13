# Phần cứng và sơ đồ chân

## Sơ đồ chân ESP32

| Chức năng | Chân |
|---|---|
| Motor phải: EN / IN1 / IN2 | 4 / 16 / 17 |
| Motor trái: EN / IN3 / IN4 | 14 / 26 / 27 |
| Encoder trái: A / B | 32 / 33 |
| Encoder phải: A / B | 19 / 18 |
| Sonar trái: TRIG / ECHO | 13 / 35 |
| Sonar giữa: TRIG / ECHO | 25 / 34 |
| Sonar phải: TRIG / ECHO | 23 / 39 |
| IMU BMI160: SDA / SCL | 21 / 22 (địa chỉ 0x68) |

Các giá trị này nằm trong `include/Config.h`.

## Lưu ý đấu nối

- **ECHO của HC-SR04 xuất 5 V.** Phải chia áp xuống 3.3 V trước khi vào ESP32.
- GPIO 34, 35, 39 chỉ là chân vào, nên phù hợp làm chân ECHO.
- **GPIO 14** (EN motor trái) có xung PWM khi ESP32 khởi động nên bánh trái có thể giật nhẹ. Nếu gặp hiện tượng này, nối điện trở kéo xuống (khoảng 10 kΩ) hoặc đổi chân.
- **L298N sụt khoảng 2 V**, nên điện áp thật sự đặt lên motor thấp hơn pin. Giá trị dùng trong tính toán là `L298N_DROP_V` trong `Config.h`.
- Mật khẩu Access Point phải có ít nhất 8 ký tự, nếu không xe không phát được Wi-Fi.

## Hằng số hình học

| Đại lượng | Giá trị |
|---|---|
| Bán kính bánh R | 0.0325 m |
| Khoảng cách hai bánh 2L | 0.184 m |
| Độ phân giải encoder | 937.2 xung/vòng bánh (11 xung × 21.3 × 4) |

Hãy đo lại các giá trị này trên xe thật trước khi tune PID (xem `docs/tuning.md`).
