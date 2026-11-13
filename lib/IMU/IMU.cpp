#include "IMU.h"

#define BMI160_CMD_REG        0x7E
#define BMI160_CHIP_ID_REG    0x00
#define BMI160_CHIP_ID        0xD1
#define BMI160_GYRO_Z_L_REG   0x10   // GYR_Z[7:0]; GYR_Z[15:8] ở 0x11 (đọc liên tiếp 2 byte)
#define BMI160_GYR_RANGE_REG  0x43
#define BMI160_GYR_RANGE_500  0x02   // ±500 deg/s
#define BMI160_CMD_GYR_NORMAL 0x15

// Dải đo ±500 deg/s -> 500 / 32768 deg/s trên mỗi LSB (độ phân giải gấp 4 so với ±2000).
// Xe xoay tối đa 3.5 rad/s ~ 200 deg/s nên vẫn còn dư.
#define GYRO_DPS_PER_LSB      (500.0f / 32768.0f)
// Bỏ qua nhiễu nền < 0.5 deg/s để chống trôi góc khi xe đứng yên
#define GYRO_DEADBAND_DPS     0.5f

IMU::IMU(uint8_t addr)
    : i2cAddr(addr), gyroZ_offset(0), currentYaw(0), currentGyroZ(0), ready(false), lastUs(0) {}

bool IMU::begin(uint8_t sda, uint8_t scl) {
    ready = false;
    Wire.begin(sda, scl);
    Wire.setClock(400000);  // Fast mode để đọc nhanh, không làm trễ vòng điều khiển
    Wire.setTimeOut(5);     // ms: I2C treo (mất dây) không làm kẹt vòng điều khiển 20ms

    // Kiểm tra Chip ID
    Wire.beginTransmission(i2cAddr);
    Wire.write(BMI160_CHIP_ID_REG);
    if (Wire.endTransmission() != 0) return false;
    if (Wire.requestFrom((uint16_t)i2cAddr, (uint8_t)1) != 1) return false;
    if (Wire.read() != BMI160_CHIP_ID) return false;

    // Bật Gyro ở Normal mode
    Wire.beginTransmission(i2cAddr);
    Wire.write(BMI160_CMD_REG);
    Wire.write(BMI160_CMD_GYR_NORMAL);
    Wire.endTransmission();
    delay(100);  // Chờ Gyro khởi động hoàn toàn

    // Đặt dải đo ±500 deg/s
    Wire.beginTransmission(i2cAddr);
    Wire.write(BMI160_GYR_RANGE_REG);
    Wire.write(BMI160_GYR_RANGE_500);
    if (Wire.endTransmission() != 0) return false;
    delay(10);

    ready = true;
    return true;
}

bool IMU::readRawGyroZ(int16_t& raw) {
    Wire.beginTransmission(i2cAddr);
    Wire.write(BMI160_GYRO_Z_L_REG);
    if (Wire.endTransmission(false) != 0) return false;
    if (Wire.requestFrom((uint16_t)i2cAddr, (uint8_t)2) != 2) return false;

    // Đọc tách từng byte: thứ tự đánh giá của "read() | read() << 8" không được đảm bảo
    const uint8_t lo = Wire.read();
    const uint8_t hi = Wire.read();
    raw = (int16_t)((hi << 8) | lo);
    return true;
}

void IMU::calibrateGyro(uint16_t samples) {
    if (!ready) return;
    long sum = 0;
    uint16_t valid = 0;
    int16_t raw;
    for (uint16_t i = 0; i < samples; i++) {
        if (readRawGyroZ(raw)) { sum += raw; valid++; }
        delay(10);  // Khớp ODR mặc định 100Hz của BMI160
    }
    if (valid > 0) gyroZ_offset = (float)sum / valid;
    lastUs = 0;     // tránh dt khổng lồ ở lần update đầu sau khi hiệu chuẩn
}

void IMU::update(float dt) {
    if (!ready || dt <= 0.0f) return;

    // Dùng dt thực đo bằng micros() (vòng lặp có thể bị trễ nhẹ); kẹp để chống giá trị bất thường
    const uint32_t now = micros();
    const float realDt = lastUs ? constrain((now - lastUs) * 1e-6f, 0.005f, 0.1f) : dt;
    lastUs = now;

    int16_t raw;
    if (!readRawGyroZ(raw)) return;  // Lỗi I2C: giữ giá trị cũ

    float dps = ((float)raw - gyroZ_offset) * GYRO_DPS_PER_LSB;
    if (fabsf(dps) < GYRO_DEADBAND_DPS) dps = 0.0f;

    currentGyroZ = dps * DEG_TO_RAD;
    currentYaw  += currentGyroZ * realDt;  // Tích phân Euler
}

float IMU::getYaw()   { return currentYaw; }
float IMU::getGyroZ() { return currentGyroZ; }
void  IMU::resetYaw() { currentYaw = 0.0f; }
