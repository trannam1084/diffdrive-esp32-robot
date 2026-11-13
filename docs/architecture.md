# Kiến trúc phần mềm

## Các lớp

| Lớp (`lib/`) | Vai trò |
|---|---|
| `Motor` | PWM và chiều quay cho L298N, bù dead-zone `MIN_PWM`, tắt motor |
| `Encoder` | ISR giải mã vuông pha 4X, tính ω và V của từng bánh (có lọc thông thấp) |
| `PID` | PID có chống windup, khâu D tính trên giá trị đo, `setGains()` để đổi khi chạy |
| `IMU` | Gyro Z của BMI160: hiệu chuẩn tĩnh, deadband, tích phân góc yaw |
| `HCSR04` | Đọc siêu âm. Hàm đọc **chặn tới 30 ms**, chưa dùng trong vòng điều khiển |
| `Chassis` | Động học, động lực học, odometry, máy trạng thái chuyển động |
| `Telemetry` | Ghi từng chu kỳ điều khiển vào RAM (tối đa 30 s), tải về qua `/log` sau khi xe dừng |

`include/Config.h` chứa mọi hằng số vật lý, chân, hệ số PID và giới hạn lệnh.

## Luồng và đa nhiệm

- **Core 0:** Wi-Fi, AsyncTCP, web server. Handler của `/run` kiểm tra lệnh rồi đặt vào hộp thư.
- **Core 1:** `ControlTask`, chạy `Chassis::update(dt)` mỗi 20 ms (50 Hz) bằng `vTaskDelayUntil`. Mỗi chu kỳ nó lấy lệnh mới (nếu có) rồi cập nhật.
- Hộp thư lệnh được bảo vệ bằng khóa để web không đọc ghi dở dang trên `ControlTask`.
- Hàm của `Chassis` chỉ được gọi từ `ControlTask`.

## Trạng thái của `Chassis`

| Trạng thái | Ô web | Cách điều khiển |
|---|---|---|
| `IDLE` | | Motor tắt |
| `KINEMATIC_CONTROL` | Động học thuận và nghịch | Profile tăng tốc, PID vận tốc bánh có feedforward |
| `VELOCITY_CONTROL` | Động lực học nghịch | Profile, feedforward động lực học, PID bù |
| `TORQUE_CONTROL` | Động lực học thuận | Mô phỏng (9), (11) rồi PID bám theo (`TORQUE_USE_MODEL`) |
| `OPENLOOP_PWM` | | Chỉ để đo đường cong PWM đến vận tốc |
| `DRIVING_DIST`, `TURNING_ANGLE` | | Đi một quãng đường, xoay một góc (API nội bộ) |

## Một chu kỳ điều khiển

1. Encoder và IMU cập nhật.
2. Kiểm tra hết thời gian chạy, nếu hết thì `stop()`.
3. Odometry: V_m từ động học thuận, θ từ IMU, cộng dồn X và Y.
4. Rẽ nhánh theo trạng thái, tính PWM, cấp cho motor.

## Telemetry

Mỗi lệnh chạy bắt đầu một lần ghi log: `ControlTask` ghi một mẫu sau mỗi lần `chassis.update()` và dừng ghi khi xe về `IDLE`. Dữ liệu nằm trong RAM (khoảng 48 KB) và được tải về qua `/log` khi xe đã dừng, nên không cần cáp USB lúc chạy. Cột CSV: `ms, state, mục tiêu R, đo R, mục tiêu L, đo L, PWM R, PWM L, yaw(độ)`. Dùng `tools/analyze_run.py` và `tools/plot_telemetry.py` để phân tích.

Môi trường build `telemetry` còn in cùng dữ liệu ra Serial, chỉ dùng được khi nối cáp.
