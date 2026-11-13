#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>

class Motor {
private:
    uint8_t pinEN, pinIN1, pinIN2;
    uint8_t pwmChannel;
    bool isReversed;   // Đảo chiều quay bằng phần mềm (không cần đổi dây)

public:
    Motor(uint8_t en, uint8_t in1, uint8_t in2, uint8_t channel, bool reverse = false);

    // Khởi tạo GPIO và kênh PWM (LEDC)
    void begin();

    // pwm_value từ -255 đến 255 (đã tự bù dead-zone)
    void setPWM(int16_t pwm_value);

    // Tắt động cơ (IN1 = IN2 = LOW)
    void stop();
};

#endif // MOTOR_H
