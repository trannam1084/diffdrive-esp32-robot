#include "PID.h"

PID::PID(float p, float i, float d, float min_out, float max_out)
    : kp(p), ki(i), kd(d), minOut(min_out), maxOut(max_out),
      integral(0.0f), prevMeasured(0.0f), hasPrev(false) {}

void PID::reset() {
    integral = 0.0f;
    prevMeasured = 0.0f;
    hasPrev = false;
}

float PID::compute(float setpoint, float measured, float dt) {
    if (dt <= 0.0f) return 0.0f;

    const float error = setpoint - measured;

    // Khâu I: kẹp chính biến tích phân (không chỉ kẹp đầu ra) để không bị windup,
    // nhờ vậy khi hết bão hòa PID phản ứng ngay.
    integral += error * dt;
    if (ki > 0.0f) {
        integral = constrain(integral, minOut / ki, maxOut / ki);
    }

    // Khâu D tính trên giá trị đo (không phải sai số) -> không bị "derivative kick"
    // khi setpoint thay đổi đột ngột.
    float dOut = 0.0f;
    if (hasPrev) dOut = -kd * (measured - prevMeasured) / dt;
    prevMeasured = measured;
    hasPrev = true;

    return constrain(kp * error + ki * integral + dOut, minOut, maxOut);
}
