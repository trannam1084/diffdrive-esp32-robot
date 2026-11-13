#ifndef TELEMETRY_H
#define TELEMETRY_H

#include <Arduino.h>

// Ghi dữ liệu mỗi chu kỳ điều khiển vào RAM trong lúc xe chạy, tải về qua Wi-Fi sau khi xe dừng.
// Chỉ giữ lần chạy gần nhất; log mất khi chạy lệnh mới hoặc khởi động lại.
struct TelemetrySample {
    uint32_t ms;                    // thời gian từ lúc bắt đầu lệnh
    float tgtR, vR, tgtL, vL;       // vận tốc mục tiêu và đo của từng bánh (m/s)
    float yawDeg;                   // góc yaw từ IMU (độ)
    int16_t pwmR, pwmL;
    uint8_t state;
};

class Telemetry {
private:
    TelemetrySample* buf = nullptr;
    uint16_t capacity = 0;
    volatile uint16_t used = 0;
    volatile bool rec = false;
    uint32_t t0 = 0;

public:
    // Cấp phát bộ nhớ một lần ở setup(). Trả về false nếu không đủ heap.
    bool begin(uint16_t maxSamples);

    // Xóa log cũ và bắt đầu ghi (gọi cùng lúc bắt đầu một lệnh chạy)
    void start();
    void stop();

    // Gọi mỗi chu kỳ khi đang ghi. Bộ nhớ đầy thì bỏ qua phần còn lại.
    void record(uint8_t state, float tgtR, float vR, float tgtL, float vL,
                float pwmR, float pwmL, float yawDeg);

    bool recording() const { return rec; }
    uint16_t count() const { return used; }

    // Viết dòng CSV thứ i vào out, trả về số byte (0 nếu i vượt quá số mẫu)
    size_t formatRow(uint16_t i, char* out, size_t maxLen) const;
    static const char* header();
};

#endif
