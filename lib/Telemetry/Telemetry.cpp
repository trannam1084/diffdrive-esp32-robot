#include "Telemetry.h"

bool Telemetry::begin(uint16_t maxSamples) {
    buf = (TelemetrySample*)malloc(sizeof(TelemetrySample) * maxSamples);
    capacity = buf ? maxSamples : 0;
    return buf != nullptr;
}

void Telemetry::start() {
    used = 0;
    t0 = millis();
    rec = (buf != nullptr);
}

void Telemetry::stop() {
    rec = false;
}

void Telemetry::record(uint8_t state, float tgtR, float vR, float tgtL, float vL,
                       float pwmR, float pwmL, float yawDeg) {
    if (!rec || used >= capacity) return;

    TelemetrySample& s = buf[used];
    s.ms = millis() - t0;
    s.tgtR = tgtR; s.vR = vR; s.tgtL = tgtL; s.vL = vL;
    s.yawDeg = yawDeg;
    s.pwmR = (int16_t)constrain(pwmR, -255.0f, 255.0f);
    s.pwmL = (int16_t)constrain(pwmL, -255.0f, 255.0f);
    s.state = state;
    used = used + 1;   // tăng sau khi mẫu đã ghi xong để phía web không đọc dở
}

size_t Telemetry::formatRow(uint16_t i, char* out, size_t maxLen) const {
    if (i >= used) return 0;
    const TelemetrySample& s = buf[i];
    const int n = snprintf(out, maxLen, "%lu,%u,%.3f,%.3f,%.3f,%.3f,%d,%d,%.2f\n",
                           (unsigned long)s.ms, (unsigned)s.state,
                           s.tgtR, s.vR, s.tgtL, s.vL, s.pwmR, s.pwmL, s.yawDeg);
    return (n > 0 && (size_t)n < maxLen) ? (size_t)n : 0;
}

const char* Telemetry::header() {
    return "ms,state,tgtR,vR,tgtL,vL,pwmR,pwmL,yaw\n";
}
