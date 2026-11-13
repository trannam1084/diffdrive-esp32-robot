#ifndef HCSR04_H
#define HCSR04_H

#include <Arduino.h>

class HCSR04 {
private:
    uint8_t trigPin;
    uint8_t echoPin;

public:
    static constexpr float NO_ECHO = 9.9f;  // Giá trị trả về khi không có phản hồi

    HCSR04(uint8_t trig, uint8_t echo);
    void begin();

    // Khoảng cách (m). Lưu ý: hàm CHẶN tới 30ms nếu không có vật cản,
    // không nên gọi trực tiếp trong vòng điều khiển 50Hz.
    float getDistance();
};

#endif
