#include "HCSR04.h"

// Tốc độ âm ~343 m/s = 0.0343 cm/us; quãng đường đi-về nên chia 2; đổi sang mét
#define SOUND_M_PER_US_HALF  (343.0f / 1e6f / 2.0f)
#define ECHO_TIMEOUT_US      30000UL  // ~5m

HCSR04::HCSR04(uint8_t trig, uint8_t echo) : trigPin(trig), echoPin(echo) {}

void HCSR04::begin() {
    pinMode(trigPin, OUTPUT);
    pinMode(echoPin, INPUT);
}

float HCSR04::getDistance() {
    // Xung kích 10us
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    const unsigned long duration = pulseIn(echoPin, HIGH, ECHO_TIMEOUT_US);
    if (duration == 0) return NO_ECHO;

    return duration * SOUND_M_PER_US_HALF;
}
