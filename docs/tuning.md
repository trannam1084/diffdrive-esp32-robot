# Hiệu chuẩn và tune PID

Hệ số trong `Config.h` chỉ là điểm xuất phát (Kp 120, Ki 400, Kd 0) và phải tune lại trên xe thật. Làm theo thứ tự dưới đây.

## Cách đo khi xe chạy mà không nối cáp USB

Xe tự ghi dữ liệu mỗi chu kỳ điều khiển (50 Hz) vào RAM trong lúc chạy. Bạn chỉ cần Wi-Fi, không cần cáp:

1. Cấp nguồn bằng pin, nối laptop hoặc điện thoại vào Wi-Fi của xe, mở `http://192.168.4.1`.
2. Chạy một lệnh bất kỳ (ô điều khiển, `/pwm` hoặc `/pid`). Xe bắt đầu ghi ngay khi lệnh bắt đầu.
3. Khi xe dừng, dòng "Tải log lần chạy gần nhất" trên trang hiện số mẫu đã ghi. Bấm vào đó, hoặc mở `http://192.168.4.1/log`, để tải file `run.csv`.
4. Đổi tên file theo hệ số đã dùng (ví dụ `kp120_ki400.csv`) rồi phân tích bằng các script trong `tools/`.

Lưu ý:
- Chỉ giữ **lần chạy gần nhất**. Chạy lệnh mới là log cũ mất, và log cũng mất khi khởi động lại xe. Hãy tải về trước khi chạy tiếp.
- Tối đa 30 giây mỗi lần (`TELEM_MAX_SAMPLES`, khoảng 48 KB RAM). Đầy thì các mẫu sau bị bỏ.
- Chỉ tải được khi xe đã dừng (đang ghi sẽ trả về mã 409).
- Laptop mất Internet khi nối vào Wi-Fi của xe, đó là bình thường.

## Bước 0: kiểm tra trước khi đụng PID

- Đẩy xe tay đúng 1 m rồi so với `x` trên web. Sai thì chỉnh `WHEEL_RADIUS` hoặc `ENCODER_RESOLUTION`.
- Quay xe 360° rồi so với θ của IMU. Sai thì chỉnh `WHEEL_BASE`.
- Cho xe tiến thì `vr` và `vl` phải cùng dương. Quay ngược chiều kim đồng hồ thì θ phải tăng.

PID chỉ bám vận tốc do encoder đo, nên encoder sai tỷ lệ thì PID bám chính xác một vận tốc sai.

## Bước 1: hiệu chuẩn feedforward

PWM nền của mỗi bánh là `WHEEL_FF_OFFSET + WHEEL_FF_GAIN * |v|`. Đặt xe trên sàn như khi chạy thật rồi chạy PWM thô ở 4 mức, mỗi mức 3 giây, và **tải log sau mỗi lần chạy**:

```
/pwm?r=120&l=120&t=3   ->  pwm120.csv
/pwm?r=160&l=160&t=3   ->  pwm160.csv
/pwm?r=200&l=200&t=3   ->  pwm200.csv
/pwm?r=255&l=255&t=3   ->  pwm255.csv
```

Mỗi mức tốn khoảng 0.5 đến 1.6 m đường đi nên cần chỗ rộng, hoặc nâng bánh lên khỏi mặt sàn nếu chấp nhận kết quả không tải (OFFSET sẽ thấp hơn khi có tải). Rồi:

```bash
python tools/fit_feedforward.py pwm120.csv pwm160.csv pwm200.csv pwm255.csv
```

Script in `OFFSET` và `GAIN` cho từng bánh và trung bình hai bánh. Ghi hai giá trị trung bình vào `include/Config.h`. Nếu GAIN hai bánh lệch nhau quá 10%, script sẽ cảnh báo và nên tách hệ số theo từng bánh.

## Bước 2: chỉ Kp

Đặt Ki = 0, Kd = 0. Dùng ô động học nghịch, đi thẳng V_m = 0.3 m/s trong 3 giây. Tăng Kp tới khi vận tốc dao động đều, rồi lấy khoảng 50–60% giá trị đó.

## Bước 3: Ki

Điểm xuất phát Ki ≈ 3.3·Kp (thời gian tích phân khoảng 0.3 s). Tăng dần tới khi sai số xác lập về 0 trong chừng 0.5 s mà vọt lố dưới 10%.

## Bước 4: Kd

Giữ Kd = 0, trừ khi còn vọt lố rõ. Vận tốc encoder có nhiễu lượng tử (1 xung ở 50 Hz là khoảng 0.011 m/s).

## Bước 5: kiểm tra bốn ô

Chạy lần lượt cả bốn ô ở ba chế độ và xem từng bánh có bám mục tiêu không.

## Đọc kết quả của mỗi lần chạy

```bash
python tools/analyze_run.py run.csv       # sai số xác lập, dao động, vọt lố, thời gian tăng
python tools/plot_telemetry.py run.csv    # đồ thị vận tốc, PWM, yaw
```

Cột CSV: `ms, state, tgtR, vR, tgtL, vL, pwmR, pwmL, yaw`. `analyze_run.py` đưa gợi ý thô (ví dụ "vọt lố cao: giảm Kp hoặc Ki"), hãy luôn xem đồ thị để xác nhận.

Quy trình tune mỗi vòng: đổi hệ số bằng `/pid`, chạy lệnh thử (ví dụ ô động học nghịch, đi thẳng V_m = 0.3 m/s trong 3 giây), tải log, chạy `analyze_run.py`.

Cách thứ hai nếu vẫn muốn log qua cáp: build môi trường `telemetry` (`pio run -e telemetry -t upload`) để in CSV ra Serial. Cách này chỉ dùng được khi xe còn nối với máy tính.

## Giới hạn vật lý cần biết

- Mô-men hoặc tốc độ thấp: do `MIN_PWM = 100`, mỗi bánh khó giữ ổn định dưới khoảng 0.1 m/s.
- Quãng đường thực tế luôn ngắn hơn V·t một đoạn khoảng V²/(2·`MAX_ACC_V`) do giai đoạn tăng tốc.
- Xe thả trôi khi dừng (motor không phanh), nên quãng đường thực có thể dài hơn dự kiến một chút.

## Endpoint tune

Hai endpoint đã có sẵn trong `src/main.cpp`:

```
/pid?kp=120&ki=400&kd=0      đổi PID của cả hai bánh khi đang chạy (không cần nạp lại)
/pwm?r=160&l=160&t=2         chạy PWM thô 2 giây (tối đa 5 giây, |PWM| tối đa 255)
```

Ví dụ trên điện thoại đang nối Wi-Fi của xe: `http://192.168.4.1/pid?kp=120&ki=400&kd=0`. Hệ số đổi qua `/pid` chỉ có tác dụng đến khi khởi động lại, nên khi chọn được giá trị tốt hãy ghi vào `PID_KP`, `PID_KI`, `PID_KD` trong `Config.h`.
