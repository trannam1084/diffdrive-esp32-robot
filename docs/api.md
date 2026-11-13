# Giao diện HTTP

Mặc định xe phát Wi-Fi và địa chỉ là `http://192.168.4.1`. Mọi endpoint dùng `GET`.

| Endpoint | Mục đích |
|---|---|
| `/` | Trang điều khiển |
| `/run` | Chạy một lệnh có thời hạn |
| `/stop` | Dừng ngay |
| `/status` | Trạng thái xe (JSON), trang web gọi mỗi 300 ms |
| `/log` | Tải CSV của lần chạy gần nhất (xem dưới) |

## `/run`

```
/run?panel=fk|ik|fd|id&mode=straight|spin|arc&a=<số>&b=<số>&t=<giây>
```

| `panel` | `a` | `b` |
|---|---|---|
| `fk` | ω_R (rad/s) | ω_L (rad/s) |
| `ik` | V_m (m/s) | ω_m (rad/s) |
| `fd` | τ_R (N·m) | τ_L (N·m) |
| `id` | V_m (m/s) | ω_m (rad/s) |

Thành công trả về `{"ok":true,"out":[["tên",giá trị,"đơn vị"], ...]}` gồm các đại lượng xe tính từ công thức (v_R, v_L, τ, gia tốc, PWM...). Bị từ chối trả về mã 400 kèm `{"ok":false,"err":"..."}`.

## `/status`

Các khóa: `run` (0/1), `left` (giây còn lại), `theta` (độ, từ IMU), `V`, `W`, `vr`, `vl`, `x`, `y`, `log` (số mẫu đã ghi), `rec` (1 nếu đang ghi).

## `/log`

Trả về file CSV `run.csv` của lần chạy gần nhất (cột: `ms,state,tgtR,vR,tgtL,vL,pwmR,pwmL,yaw`). Trả về 409 nếu xe đang ghi, 404 nếu chưa có log. Mỗi lệnh `/run` hoặc `/pwm` mới sẽ xóa log cũ.

## Giới hạn nhập (mặc định trong `Config.h`)

| Giới hạn | Giá trị | Lý do |
|---|---|---|
| Thời gian | 0.1 đến 30 s | An toàn: mất kết nối thì xe vẫn tự dừng |
| Tốc độ góc bánh (`fk`) | tối đa 18 rad/s | Gần tốc độ tối đa có tải của motor |
| V_m | tối đa 0.6 m/s | Cùng lý do |
| Vận tốc mỗi bánh | tối đa 0.8 m/s | Vượt thì PWM bão hòa 255, xe không đạt giá trị đặt |
| ω_m | tối đa 3.5 rad/s | Gyro đặt dải ±500 dps (8.7 rad/s), xoay nhanh làm bánh trượt |
| Mô-men (`fd`) | tối đa 0.15 N·m | Gần mô-men kẹt của motor |

Việc kiểm tra hiện chia làm hai tầng:

- **Trang web (`WebPage.h`)** kiểm tra đầy đủ: khoảng giá trị, quy tắc dấu theo từng chế độ, và giới hạn tốc độ bánh.
- **Firmware (`validateRun` trong `src/main.cpp`)** chỉ kiểm tra khoảng giá trị (cho phép dư 0.05) và thời gian. Hàm đã được nới lỏng nên gọi `/run` trực tiếp sẽ **không** bị chặn khi sai quy tắc dấu, và giới hạn `MAX_WHEEL_V` không được kiểm tra ở phía xe. Ví dụ V_m = 0.6 cùng ω_m = 3.5 cho vận tốc bánh ngoài 0.92 m/s, vượt mức PWM đạt được.

Đổi giới hạn thì phải đổi đồng bộ: `MAX_*` trong `include/Config.h`, `validateRun` trong `src/main.cpp`, và hằng số JavaScript trong `include/WebPage.h`.

## Endpoint tùy chọn khi tune

Có sẵn trong `src/main.cpp`: `/pid?kp=&ki=&kd=` (đổi PID khi chạy) và `/pwm?r=&l=&t=` (chạy PWM thô tối đa 5 giây để hiệu chuẩn, \|PWM\| tối đa 255). Cách dùng nằm trong [docs/tuning.md](tuning.md). Hai endpoint này không có xác thực, nên gỡ khi nộp bản cuối.
