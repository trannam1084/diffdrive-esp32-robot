#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// ==========================================
// 1. PINOUT ESP32
// ==========================================
// Motor & L298N
#define PIN_MOTOR_R_EN      4
#define PIN_MOTOR_R_IN1     16
#define PIN_MOTOR_R_IN2     17
#define PIN_MOTOR_L_EN      14
#define PIN_MOTOR_L_IN3     26
#define PIN_MOTOR_L_IN4     27

// Encoder
#define PIN_ENC_L_A         32
#define PIN_ENC_L_B         33
#define PIN_ENC_R_A         19
#define PIN_ENC_R_B         18

// Sonar HC-SR04
#define PIN_SONAR_L_TRIG    13
#define PIN_SONAR_L_ECHO    35
#define PIN_SONAR_M_TRIG    25
#define PIN_SONAR_M_ECHO    34
#define PIN_SONAR_R_TRIG    23
#define PIN_SONAR_R_ECHO    39

// IMU BMI160
#define PIN_IMU_SDA         21
#define PIN_IMU_SCL         22
#define IMU_I2C_ADDR        0x68

// ==========================================
// 2. THÔNG SỐ HÌNH HỌC
// ==========================================
#define WHEEL_RADIUS        0.0325f                  // R (m) Bán kính bánh = 65mm / 2
#define WHEEL_CIRCUMFERENCE (2.0f * PI * WHEEL_RADIUS) // (m) Chu vi bánh
#define WHEEL_BASE          0.1840f                  // 2L (m) Khoảng cách 2 bánh
#define L_HALF              (WHEEL_BASE / 2.0f)      // L (m) Từ tâm xe đến 1 bánh
#define ENCODER_RESOLUTION  937.2f                   // 11 xung * 21.3 tỉ số truyền * 4 (chế độ 4X)
#define ENCODER_LPF_ALPHA   0.4f                     // Lọc thông thấp vận tốc encoder (0..1, nhỏ = mượt hơn nhưng trễ hơn)

// ==========================================
// 3. CHU KỲ VÒNG LẶP
// ==========================================
#define CONTROL_LOOP_HZ     50                       // Vòng điều khiển 50Hz (20ms)

// ==========================================
// 4. ĐỘNG CƠ & PWM
// ==========================================
#define PWM_FREQ            5000                     // 5kHz: giảm tiếng rít, ít suy hao
#define PWM_RESOLUTION      8                        // 8-bit (0-255)
#define PWM_CHANNEL_R       0
#define PWM_CHANNEL_L       1
#define MAX_PWM             255                      // L298N sụt ~2V nên 100% vẫn an toàn cho motor 12V
#define MIN_PWM             100                      // Dead-zone: dưới mức này motor chỉ rít, không quay

// ==========================================
// 5. PID VẬN TỐC BÁNH XE
// ==========================================
// Lưu ý: ở chế độ động lực học nghịch, PID chỉ bù sai số quanh feedforward,
// nên nếu xe dao động hãy giảm PID_KP / PID_KI (ví dụ KP ~150, KI ~100).
// Điểm xuất phát đề xuất: đã có feedforward vận tốc (WHEEL_FF_*) nên PID chỉ bù sai số.
// Ki tương đối lớn vì sai lệch giữa 2 bánh rất nhỏ (~0.01-0.02 m/s) nhưng quyết định bán kính vòng cung.
// Cần tune lại trên xe thật theo hướng dẫn.
#define PID_KP              120.0f
#define PID_KI              400.0f
#define PID_KD              0.0f
#define PID_MAX_OUT         (MAX_PWM)
#define PID_MIN_OUT         (-MAX_PWM)

// ==========================================
// 6. ĐIỀU KHIỂN GÓC XOAY & QUÃNG ĐƯỜNG
// ==========================================
#define TURN_MAX_SPEED      3.5f                     // (rad/s) Vận tốc góc tối đa khi xoay xe
#define PID_TURN_KP         0.12f                    // PID góc: sai số (độ) -> W (rad/s)
#define PID_TURN_KI         0.0f
#define PID_TURN_KD         0.0f
#define TURN_TOLERANCE_DEG  1.5f                     // (độ) Sai số góc chấp nhận được
#define TURN_SETTLE_GYRO    0.1f                     // (rad/s) Xe coi là đã dừng xoay
#define DRIVE_TIMEOUT_MS    5000                     // Quá thời gian này -> coi là kẹt, dừng xe
#define TURN_TIMEOUT_MS     4000

// Giữ hướng khi đi thẳng (theta = 0 theo IMU): sai số góc (rad) -> W (rad/s)
#define HEADING_KP          2.0f

// Giới hạn gia tốc của profile vận tốc (động lực học nghịch) - cần tuning
#define MAX_ACC_V           1.0f                     // (m/s^2)
#define MAX_ACC_W           10.0f                    // (rad/s^2)

// ==========================================
// 7. ĐỘNG LỰC HỌC
// ==========================================
#define ROBOT_M_TONG        0.81f                    // (kg) Khối lượng tương đương
#define ROBOT_I_TONG        0.00447f                 // (kg.m^2) Momen quán tính tương đương
#define F_MS                0.302f                   // (N) Ma sát lăn cơ bản
#define B_V                 0.15f                    // (N.s/m) Cản nhớt tịnh tiến
#define B_OMEGA             0.002f                   // (Nm.s/rad) Cản nhớt góc

// Động cơ JGA25-370 280RPM & pin (mô hình điện, dùng để quy đổi mô-men -> PWM)
#define MOTOR_KT            0.128f                   // (Nm/A)
#define MOTOR_KE            0.409f                   // (V/(rad/s))
#define MOTOR_RA            9.23f                    // (Ohm)
#define BATT_VOLTAGE        11.1f                    // (V) Điện áp pin trung bình

// Sụt áp của L298N (~2V): điện áp thật sự đặt lên motor = pin - sụt áp.
// Dùng trong torqueToPWM/pwmToTorque (nếu bỏ qua, PWM feedforward bị thấp ~18%).
#define L298N_DROP_V        2.0f
#define VBATT_EFF           (BATT_VOLTAGE - L298N_DROP_V)

// Feedforward vận tốc bánh (chế độ động học): PWM = OFFSET + GAIN * |v|
//  - GAIN lấy từ Ke (PWM trên mỗi m/s); OFFSET bù ma sát/dead-zone.
//  - Cả hai chỉ là ước lượng ban đầu: hãy ĐO LẠI bằng /pwm (xem hướng dẫn hiệu chuẩn).
#define WHEEL_FF_OFFSET     60.0f
#define WHEEL_FF_GAIN       (MOTOR_KE / WHEEL_RADIUS / VBATT_EFF * MAX_PWM)
#define WHEEL_V_DEADBAND    0.03f                    // (m/s) mục tiêu bánh nhỏ hơn mức này -> tắt motor

// Ghi log vào RAM mỗi chu kỳ điều khiển để tải về sau khi xe dừng (không cần cáp USB lúc chạy).
// 1500 mẫu = 30 s ở 50 Hz, khoảng 48 KB heap.
#define TELEM_MAX_SAMPLES   1500

// Chế độ động lực học thuận (đặt mô-men)
//  1 = mô phỏng phương trình (9), (11) trong phần mềm rồi để PID bánh bám theo (đi được vòng tròn ổn định)
//  0 = chạy hở: mô-men -> PWM trực tiếp (bão hòa nhanh, xe phóng thẳng)
#define TORQUE_USE_MODEL    1
#define TORQUE_MODE_VMAX    0.5f                     // (m/s) tốc độ bánh tối đa trong chế độ này

// ==========================================
// 8. HC-SR04
// ==========================================
#define DISTANCE_THRESHOLD  0.25f                    // (m) Gần hơn mức này -> nên dừng xe

// ==========================================
// 9. GIỚI HẠN LỆNH TỪ WEB
// ==========================================
#define MAX_V               0.6f                     // (m/s) V_m tối đa
#define MAX_W               TURN_MAX_SPEED           // (rad/s) omega_m tối đa
#define MAX_WHEEL_V         0.8f                     // (m/s) Vận tốc tối đa mỗi bánh
#define MAX_WHEEL_OMEGA     18.0f                    // (rad/s) Tốc độ góc tối đa mỗi bánh
#define MAX_TAU             0.15f                    // (N.m) Mô-men tối đa mỗi bánh (~ mô-men kẹt motor)
#define MAX_RUN_TIME_S      30.0f                    // (s) Thời gian chạy tối đa cho một lệnh

// ==========================================
// 10. WI-FI & WEB SERVER
// ==========================================
// WIFI_AP_MODE, AP_SSID, AP_PASS, STA_SSID, STA_PASS nằm trong include/secrets.h
// (file này KHÔNG được commit). Lần đầu: sao chép include/secrets.example.h thành include/secrets.h.
#if __has_include("secrets.h")
#include "secrets.h"
#else
#error "Thieu include/secrets.h - sao chep include/secrets.example.h thanh include/secrets.h roi dien thong tin Wi-Fi"
#endif

#endif // CONFIG_H
