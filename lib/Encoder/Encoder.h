#ifndef ENCODER_H
#define ENCODER_H

#include <Arduino.h>

// Cài đặt ngắt cho 2 encoder (gọi 1 lần trong setup)
void initEncoders();

// Biến đếm xung cập nhật trong ISR
extern volatile int32_t ticksRight;
extern volatile int32_t ticksLeft;

class Encoder {
private:
    volatile int32_t* tickCount;  // Trỏ tới ticksRight hoặc ticksLeft
    int32_t lastTicks;
    float angularVelocity;        // rad/s
    float linearVelocity;         // m/s

public:
    explicit Encoder(volatile int32_t* counterPtr);

    // Gọi mỗi chu kỳ điều khiển để tính vận tốc
    void update(float dt);

    int32_t getTicks();
    float getAngularVelocity();
    float getLinearVelocity();
    float getDistance();          // Tổng quãng đường bánh đã đi (m)
};

#endif // ENCODER_H
