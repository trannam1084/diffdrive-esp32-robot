#include "Motor.h"
#include "Config.h"

Motor::Motor(uint8_t en, uint8_t in1, uint8_t in2, uint8_t channel, bool reverse)
    : pinEN(en), pinIN1(in1), pinIN2(in2), pwmChannel(channel), isReversed(reverse) {}

void Motor::begin() {
    pinMode(pinIN1, OUTPUT);
    pinMode(pinIN2, OUTPUT);

    // API LEDC của Arduino-ESP32 core 2.x
    ledcSetup(pwmChannel, PWM_FREQ, PWM_RESOLUTION);
    ledcAttachPin(pinEN, pwmChannel);

    stop();
}

void Motor::setPWM(int16_t pwm_value) {
    if (isReversed) pwm_value = -pwm_value;
    pwm_value = constrain(pwm_value, -MAX_PWM, MAX_PWM);

    if (pwm_value == 0) {
        stop();
        return;
    }

    // Dead-zone: lệnh khác 0 nhưng quá nhỏ thì nâng lên MIN_PWM để motor thực sự quay
    int16_t duty = abs(pwm_value);
    if (duty < MIN_PWM) duty = MIN_PWM;

    const bool forward = pwm_value > 0;
    digitalWrite(pinIN1, forward ? HIGH : LOW);
    digitalWrite(pinIN2, forward ? LOW : HIGH);
    ledcWrite(pwmChannel, duty);
}

void Motor::stop() {
    digitalWrite(pinIN1, LOW);
    digitalWrite(pinIN2, LOW);
    ledcWrite(pwmChannel, 0);
}
