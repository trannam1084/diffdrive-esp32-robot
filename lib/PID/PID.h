#ifndef PID_H
#define PID_H

#include <Arduino.h>

class PID {
private:
    float kp, ki, kd;
    float minOut, maxOut;
    float integral;
    float prevMeasured;
    bool  hasPrev;       // false ở lần tính đầu tiên -> bỏ qua khâu D để tránh xung đột biến

public:
    PID(float p, float i, float d, float min_out, float max_out);

    // Xóa trạng thái (gọi khi dừng xe / đổi lệnh để tránh windup)
    void reset();

    // Trả về giá trị điều khiển đã kẹp trong [minOut, maxOut]
    float compute(float setpoint, float measured, float dt);

    // Đổi hệ số khi đang chạy (dùng cho tune qua web). Gán float là thao tác nguyên tử trên ESP32.
    void setGains(float p, float i, float d) { kp = p; ki = i; kd = d; }
};

#endif // PID_H
