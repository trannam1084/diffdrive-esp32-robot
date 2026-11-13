#ifndef IMU_H
#define IMU_H

#include <Arduino.h>
#include <Wire.h>

class IMU {
private:
    uint8_t i2cAddr;
    float gyroZ_offset;   // Offset tĩnh (raw LSB)
    float currentYaw;     // rad
    float currentGyroZ;   // rad/s
    bool  ready;          // true sau khi begin() thành công
    uint32_t lastUs;      // mốc micros() của lần update trước (0 = chưa có)

    // Đọc raw Gyro Z (16-bit). Trả về false nếu lỗi I2C.
    bool readRawGyroZ(int16_t& raw);

public:
    explicit IMU(uint8_t addr);

    // Khởi tạo I2C và bật Gyro. Trả về false nếu không thấy BMI160.
    bool begin(uint8_t sda, uint8_t scl);
    bool isReady() const { return ready; }

    // Lấy trung bình n mẫu khi xe đứng yên để tính offset tĩnh
    void calibrateGyro(uint16_t samples = 200);

    // Đọc Gyro Z và tích phân góc Yaw (gọi mỗi chu kỳ điều khiển)
    void update(float dt);

    float getYaw();       // rad
    float getGyroZ();     // rad/s
    void resetYaw();
};

#endif // IMU_H
