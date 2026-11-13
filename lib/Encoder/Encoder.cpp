#include "Encoder.h"
#include "Config.h"
#include "soc/gpio_struct.h"

volatile int32_t ticksRight = 0;
volatile int32_t ticksLeft = 0;

// ==========================================
// ISR giải mã vuông pha 4X bằng bảng tra.
// - Đọc thẳng thanh ghi GPIO (nhanh, nằm trong IRAM): digitalRead có thể nằm trong
//   flash, gây crash "Cache disabled" khi Wi-Fi/flash đang ghi.
// - Bảng phải đặt trong DRAM_ATTR vì lý do tương tự.
// - Chuyển trạng thái không hợp lệ (nhiễu, bỏ lỡ xung) cho kết quả 0 thay vì đếm sai.
// Trạng thái = (A<<1)|B; chỉ số = (trạng thái cũ << 2) | trạng thái mới.
// Chiều đếm giống bản ISR cũ: chuỗi 00 -> 01 -> 11 -> 10 là +1.
// Nếu một bánh đếm ngược chiều mong muốn, đổi dấu cả bảng cho bánh đó.
// ==========================================
static const DRAM_ATTR int8_t QEM[16] = {
     0, +1, -1,  0,
    -1,  0,  0, +1,
    +1,  0,  0, -1,
     0, -1, +1,  0
};

static inline uint8_t IRAM_ATTR readPin(uint8_t pin) {
    return pin < 32 ? (GPIO.in >> pin) & 1 : (GPIO.in1.val >> (pin - 32)) & 1;
}

void IRAM_ATTR isrRight() {
    static uint8_t s = 0;
    s = ((s << 2) | (readPin(PIN_ENC_R_A) << 1) | readPin(PIN_ENC_R_B)) & 0x0F;
    ticksRight = ticksRight + QEM[s];
}

void IRAM_ATTR isrLeft() {
    static uint8_t s = 0;
    s = ((s << 2) | (readPin(PIN_ENC_L_A) << 1) | readPin(PIN_ENC_L_B)) & 0x0F;
    ticksLeft = ticksLeft + QEM[s];
}

void initEncoders() {
    pinMode(PIN_ENC_R_A, INPUT_PULLUP);
    pinMode(PIN_ENC_R_B, INPUT_PULLUP);
    pinMode(PIN_ENC_L_A, INPUT_PULLUP);
    pinMode(PIN_ENC_L_B, INPUT_PULLUP);

    // Cả kênh A và B cùng vào một ISR cho mỗi bánh
    attachInterrupt(digitalPinToInterrupt(PIN_ENC_R_A), isrRight, CHANGE);
    attachInterrupt(digitalPinToInterrupt(PIN_ENC_R_B), isrRight, CHANGE);
    attachInterrupt(digitalPinToInterrupt(PIN_ENC_L_A), isrLeft, CHANGE);
    attachInterrupt(digitalPinToInterrupt(PIN_ENC_L_B), isrLeft, CHANGE);
}

// ==========================================
// Encoder class
// ==========================================
Encoder::Encoder(volatile int32_t* counterPtr)
    : tickCount(counterPtr), lastTicks(0), angularVelocity(0.0f), linearVelocity(0.0f) {}

void Encoder::update(float dt) {
    if (dt <= 0.0f) return;

    // Đọc int32 căn hàng 4 byte trên ESP32 là thao tác nguyên tử,
    // không cần khóa critical section.
    const int32_t current = *tickCount;
    const int32_t delta = current - lastTicks;
    lastTicks = current;

    // ω_i (rad/s) từ số xung trong dt; lọc thông thấp vì 1 xung ở 50Hz ~ 0.011 m/s
    const float raw = ((float)delta / ENCODER_RESOLUTION) * TWO_PI / dt;
    angularVelocity += ENCODER_LPF_ALPHA * (raw - angularVelocity);

    // [ĐH-T] V_i = R * ω_i  (PDF mục 2, phương trình (2))
    linearVelocity = angularVelocity * WHEEL_RADIUS;
}

int32_t Encoder::getTicks()            { return *tickCount; }
float   Encoder::getAngularVelocity()  { return angularVelocity; }
float   Encoder::getLinearVelocity()   { return linearVelocity; }

float Encoder::getDistance() {
    return ((float)getTicks() / ENCODER_RESOLUTION) * WHEEL_CIRCUMFERENCE;
}
