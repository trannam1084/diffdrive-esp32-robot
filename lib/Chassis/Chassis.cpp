#include "Chassis.h"

// Ký hiệu nhãn công thức trong comment (tham chiếu PDF "ĐH và ĐLH xe"):
//   [ĐH-T]  động học thuận     [ĐH-N]  động học nghịch
//   [ĐLH-T] động lực học thuận [ĐLH-N] động lực học nghịch

// Hàm dấu: trả về 0 khi giá trị gần 0 (tránh sinh lực ma sát khi mục tiêu = 0)
static inline float sgn(float x) {
    return x > 0.001f ? 1.0f : (x < -0.001f ? -1.0f : 0.0f);
}

// Dịch dần giá trị hiện tại về mục tiêu, mỗi bước tối đa maxStep
static float slewTo(float cur, float target, float maxStep) {
    float d = target - cur;
    if (d >  maxStep) d =  maxStep;
    if (d < -maxStep) d = -maxStep;
    return cur + d;
}

// [ĐLH] PDF mục 1.2e: lực cản Fc = Fms + bv*|V|, momen cản Mc = Fms*L + bω*|ω|,
// luôn ngược chiều chuyển động (bằng 0 khi V = 0 hoặc ω = 0)
static inline void resistance(float V, float W, float& Fc, float& Mc) {
    Fc = sgn(V) * (F_MS + B_V * fabsf(V));
    Mc = sgn(W) * (F_MS * L_HALF + B_OMEGA * fabsf(W));
}

Chassis::Chassis(Motor* mR, Motor* mL, Encoder* eR, Encoder* eL, PID* pR, PID* pL, IMU* i)
    : motorR(mR), motorL(mL), encR(eR), encL(eL), pidVelR(pR), pidVelL(pL), imu(i),
      pidTurn(PID_TURN_KP, PID_TURN_KI, PID_TURN_KD, -TURN_MAX_SPEED, TURN_MAX_SPEED),
      state(IDLE), targetVelR(0), targetVelL(0),
      startDistR(0), startDistL(0), targetDistance(0), targetYaw(0),
      actionStartTime(0), x_pos(0), y_pos(0), theta(0),
      target_v_final(0), target_w_final(0),
      current_v_profile(0), current_w_profile(0),
      prev_target_Vm(0), prev_target_Wm(0) {}

void Chassis::begin() {
    stop();
}

// ==========================================
// Hàm phụ trợ
// ==========================================
void Chassis::applyVW(float v, float w) {
    // [ĐH-N] (V_m, ω_m) -> (v_R, v_L)
    WheelVelocity wheel_cmd = computeInverseKinematics(v, w);
    targetVelR = wheel_cmd.v_R;
    targetVelL = wheel_cmd.v_L;
}

bool Chassis::timedOut(uint32_t timeout_ms) {
    return (millis() - actionStartTime) > timeout_ms;
}

float Chassis::traveledDistance() {
    return ((encR->getDistance() - startDistR) + (encL->getDistance() - startDistL)) / 2.0f;
}

// PWM nền: bù ma sát và sức phản điện động theo vận tốc mong muốn của bánh
float Chassis::wheelFeedforward(float v) {
    if (fabsf(v) < WHEEL_V_DEADBAND) return 0.0f;
    return sgn(v) * (WHEEL_FF_OFFSET + WHEEL_FF_GAIN * fabsf(v));
}

float Chassis::runWheelPID(PID* pid, Motor* motor, float target, float measured, float dt) {
    // Mục tiêu ~ 0: tắt hẳn motor. Nếu để PID chạy, đầu ra nhỏ bị dead-zone
    // nâng lên MIN_PWM sẽ làm bánh giật quanh vị trí dừng.
    if (fabsf(target) < WHEEL_V_DEADBAND) {
        pid->reset();
        motor->stop();
        return 0.0f;
    }
    // Feedforward lo phần lớn PWM, PID chỉ bù sai số nhỏ quanh đó
    float u = wheelFeedforward(target) + pid->compute(target, measured, dt);

    // Không đảo chiều motor khi mục tiêu cùng một hướng: PWM âm nhỏ bị Motor::setPWM nâng
    // lên -MIN_PWM, tức là đạp ngược 100/255 mỗi lần vượt mục tiêu -> bánh giật, không bám được.
    if ((target > 0.0f && u < 0.0f) || (target < 0.0f && u > 0.0f)) u = 0.0f;

    u = constrain(u, -(float)MAX_PWM, (float)MAX_PWM);
    motor->setPWM((int16_t)u);
    return u;
}

void Chassis::applyDynamicsProfile(float dt) {
    current_v_profile = slewTo(current_v_profile, target_v_final, MAX_ACC_V * dt);
    current_w_profile = slewTo(current_w_profile, target_w_final, MAX_ACC_W * dt);
}

// ==========================================
// Lệnh điều khiển
// ==========================================
void Chassis::stop() {
    targetVelR = targetVelL = 0.0f;
    target_v_final = target_w_final = 0.0f;
    current_v_profile = current_w_profile = 0.0f;
    prev_target_Vm = prev_target_Wm = 0.0f;
    runDurationMs = 0;
    holdHeading = false;
    kinV = kinW = 0.0f;
    cmdTauR = cmdTauL = 0.0f;
    cmdPwmR = cmdPwmL = 0.0f;
    lastPwmR = lastPwmL = 0.0f;
    pidVelR->reset();
    pidVelL->reset();
    pidTurn.reset();
    motorR->stop();
    motorL->stop();
    state = IDLE;
}

void Chassis::beginRun(uint32_t ms, bool hold) {
    resetOdom();               // theta = 0 từ IMU tại thời điểm bắt đầu
    runStartMs = millis();
    runDurationMs = ms;
    holdHeading = hold;
}

float Chassis::getRemainingS() {
    if (state == IDLE || runDurationMs == 0) return 0.0f;
    const uint32_t elapsed = millis() - runStartMs;
    return elapsed >= runDurationMs ? 0.0f : (runDurationMs - elapsed) / 1000.0f;
}

// Động học: đặt (V, W), PID vận tốc bánh bám theo
void Chassis::setKinematicVW(float v, float w) {
    if (state != KINEMATIC_CONTROL) {
        // Bắt đầu profile từ vận tốc thực tế để không bị giật
        const KinematicsState m = getMeasuredKinematics();
        current_v_profile = m.V_m;
        current_w_profile = m.W_m;
        pidVelR->reset();
        pidVelL->reset();
    }
    kinV = v;
    kinW = w;
    target_v_final = v;
    target_w_final = w;
    applyVW(current_v_profile, current_w_profile);
    state = KINEMATIC_CONTROL;
}

void Chassis::setVelocityVW(float v, float w) {
    setKinematicVW(v, w);
}

void Chassis::setWheelVelocities(float vR, float vL) {
    // [ĐH-T] (v_R, v_L) -> (V_m, ω_m)
    const KinematicsState k = computeForwardKinematics(vR, vL);
    setKinematicVW(k.V_m, k.W_m);
}

// Động lực học nghịch: profile gia tốc + feedforward + PID bù
void Chassis::setDynamicsVW(float v, float w) {
    if (state != VELOCITY_CONTROL) {
        // Bắt đầu profile từ vận tốc thực tế để không bị giật
        const KinematicsState m = getMeasuredKinematics();
        current_v_profile = prev_target_Vm = m.V_m;
        current_w_profile = prev_target_Wm = m.W_m;
        pidVelR->reset();
        pidVelL->reset();
    }
    target_v_final = v;
    target_w_final = w;
    state = VELOCITY_CONTROL;
}

// Động lực học thuận: đặt mô-men, chạy hở
void Chassis::setTorques(float tauR, float tauL) {
    if (state != TORQUE_CONTROL) {
        // Mô hình bắt đầu từ trạng thái thực tế của xe
        const KinematicsState m = getMeasuredKinematics();
        current_v_profile = m.V_m;
        current_w_profile = m.W_m;
        pidVelR->reset();
        pidVelL->reset();
    }
    cmdTauR = tauR;
    cmdTauL = tauL;
    state = TORQUE_CONTROL;
}

// Hiệu chuẩn: đặt PWM thô, đọc vận tốc ổn định ở web để suy ra WHEEL_FF_OFFSET / WHEEL_FF_GAIN
void Chassis::setOpenLoopPWM(float pwmR, float pwmL) {
    cmdPwmR = pwmR;
    cmdPwmL = pwmL;
    state = OPENLOOP_PWM;
}

void Chassis::driveDistance(float distance_m, float speed) {
    startDistR = encR->getDistance();
    startDistL = encL->getDistance();
    targetDistance = distance_m;

    // Chiều đi do dấu của distance_m quyết định, không phải dấu của speed
    speed = fabsf(speed);
    if (distance_m < 0) speed = -speed;

    targetVelR = targetVelL = speed;
    state = DRIVING_DIST;
    actionStartTime = millis();
}

void Chassis::turnAngle(float angle_deg) {
    targetYaw = imu->getYaw() * RAD_TO_DEG + angle_deg;  // Xoay tương đối so với hướng hiện tại
    pidTurn.reset();
    state = TURNING_ANGLE;
    actionStartTime = millis();
}

bool Chassis::isIdle() { return state == IDLE; }

// ==========================================
// 1. ĐỘNG HỌC
// ==========================================
// [ĐH-T] PDF mục 2:  V_m = (V_r + V_l)/2     ω_m = (V_r - V_l)/(2L)
Chassis::KinematicsState Chassis::computeForwardKinematics(float v_R, float v_L) {
    KinematicsState ks;
    ks.V_m = (v_R + v_L) / 2.0f;
    ks.W_m = (v_R - v_L) / WHEEL_BASE;     // WHEEL_BASE = 2L
    return ks;
}

// Lấy (V, W) thực tế: encoder -> V_r = R*ω_r (trong Encoder) -> động học thuận
Chassis::KinematicsState Chassis::getMeasuredKinematics() {
    return computeForwardKinematics(encR->getLinearVelocity(), encL->getLinearVelocity());
}

// [ĐH-N] PDF phần "Động học nghịch":  V_r = V_m + L*ω_m     V_l = V_m - L*ω_m
// (vận tốc góc bánh: ω_i = V_i / R, tính ở nơi gọi)
Chassis::WheelVelocity Chassis::computeInverseKinematics(float v_m, float w_m) {
    WheelVelocity wheel_vel;
    wheel_vel.v_R = v_m + (w_m * L_HALF);
    wheel_vel.v_L = v_m - (w_m * L_HALF);
    return wheel_vel;
}

// ==========================================
// Vòng cập nhật chính
// ==========================================
void Chassis::update(float dt) {
    encR->update(dt);
    encL->update(dt);
    imu->update(dt);

    const float vR = encR->getLinearVelocity();
    const float vL = encL->getLinearVelocity();

#ifdef TELEMETRY
    // CSV 50Hz để vẽ đồ thị tune PID: ms, state, mục tiêu R, đo R, mục tiêu L, đo L, PWM R, PWM L, yaw(độ)
    Serial.printf("%lu,%d,%.3f,%.3f,%.3f,%.3f,%.0f,%.0f,%.2f\n", millis(), (int)state,
                  targetVelR, vR, targetVelL, vL, lastPwmR, lastPwmL, imu->getYaw() * RAD_TO_DEG);
#endif

    // Hết thời gian chạy của lệnh từ web -> tự dừng
    if (state != IDLE && runDurationMs > 0 && (millis() - runStartMs) >= runDurationMs) {
        stop();
        return;
    }

    // [ĐH-T] Odometry, PDF mục 3:  Ẋ = V_m*cosθ,  Ẏ = V_m*sinθ
    // V_m lấy từ động học thuận; θ lấy từ IMU (chính xác hơn tích phân ω_m của encoder khi bánh trượt)
    const KinematicsState km = computeForwardKinematics(vR, vL);
    theta = imu->getYaw();
    const float dS = km.V_m * dt;
    x_pos += dS * cosf(theta);
    y_pos += dS * sinf(theta);

    switch (state) {
        case IDLE:
            return;  // Motor đã được tắt trong stop()

        case DRIVING_DIST:
            if (fabsf(traveledDistance()) >= fabsf(targetDistance) || timedOut(DRIVE_TIMEOUT_MS)) {
                stop();
            }
            break;

        case TURNING_ANGLE: {
            const float yawDeg = imu->getYaw() * RAD_TO_DEG;
            // PID góc (độ) -> vận tốc góc W (rad/s), giới hạn bởi TURN_MAX_SPEED
            applyVW(0.0f, pidTurn.compute(targetYaw, yawDeg, dt));   // [ĐH-N] qua applyVW

            const bool reached = fabsf(targetYaw - yawDeg) < TURN_TOLERANCE_DEG &&
                                 fabsf(imu->getGyroZ()) < TURN_SETTLE_GYRO;
            if (reached || timedOut(TURN_TIMEOUT_MS)) stop();
            break;
        }

        case KINEMATIC_CONTROL:
            // Đi thẳng: bù W nhỏ để giữ theta = 0 theo IMU
            target_v_final = kinV;
            target_w_final = holdHeading ? constrain(HEADING_KP * (0.0f - theta), -1.0f, 1.0f) : kinW;
            // Tăng/giảm tốc có giới hạn: lệnh bậc thang làm PID bão hòa 255, bánh trượt, vọt lố
            applyDynamicsProfile(dt);
            applyVW(current_v_profile, current_w_profile);   // [ĐH-N] (V, W) -> vận tốc 2 bánh
            break;                    // rơi xuống PID vận tốc ở cuối hàm

        case TORQUE_CONTROL: {
#if TORQUE_USE_MODEL
            // [ĐLH-T] Chạy mô hình trong phần mềm: τ -> (V̇, ω̇) theo (9), (11) -> tích phân ra (V, ω).
            // Động cơ có sức phản điện động nên không giữ được mô-men không đổi khi bánh quay nhanh:
            // cả hai bánh cùng chạm 255 và chênh lệch biến mất. Do đó thay vì cấp PWM hở, ta dùng
            // (V, ω) của mô hình làm mục tiêu cho PID bánh.
            const StateAccel a = computeAccelFromTorque(cmdTauR, cmdTauL, current_v_profile, current_w_profile);
            current_v_profile += a.a_V * dt;
            current_w_profile += a.a_W * dt;

            // Giới hạn tốc độ bánh: nhân CẢ V và ω với cùng một hệ số -> giữ nguyên bán kính quay V/ω
            const WheelVelocity wm = computeInverseKinematics(current_v_profile, current_w_profile);
            const float vMax = fmaxf(fabsf(wm.v_R), fabsf(wm.v_L));
            if (vMax > TORQUE_MODE_VMAX) {
                const float k = TORQUE_MODE_VMAX / vMax;
                current_v_profile *= k;
                current_w_profile *= k;
            }
            applyVW(current_v_profile, current_w_profile);   // [ĐH-N]
            break;                // rơi xuống PID vận tốc ở cuối hàm
#else
            // [ĐLH-T] Chạy hở: τ -> PWM (có bù sức phản điện động theo tốc độ bánh)
            lastPwmR = torqueToPWM(cmdTauR, encR->getAngularVelocity());
            lastPwmL = torqueToPWM(cmdTauL, encL->getAngularVelocity());
            motorR->setPWM((int16_t)constrain(lastPwmR, -(float)MAX_PWM, (float)MAX_PWM));
            motorL->setPWM((int16_t)constrain(lastPwmL, -(float)MAX_PWM, (float)MAX_PWM));
            return;
#endif
        }

        case OPENLOOP_PWM:
            lastPwmR = cmdPwmR;
            lastPwmL = cmdPwmL;
            motorR->setPWM((int16_t)cmdPwmR);
            motorL->setPWM((int16_t)cmdPwmL);
            return;

        case VELOCITY_CONTROL: {
            // Đi thẳng: giữ theta = 0 theo IMU
            if (holdHeading) target_w_final = constrain(HEADING_KP * (0.0f - theta), -1.0f, 1.0f);

            applyDynamicsProfile(dt);

            // Đã dừng hẳn -> tắt motor, tránh bị nâng lên MIN_PWM
            if (fabsf(target_v_final) < 1e-3f && fabsf(target_w_final) < 1e-3f &&
                fabsf(current_v_profile) < 1e-3f && fabsf(current_w_profile) < 1e-3f) {
                pidVelR->reset();
                pidVelL->reset();
                prev_target_Vm = prev_target_Wm = 0.0f;
                lastPwmR = lastPwmL = 0.0f;
                targetVelR = targetVelL = 0.0f;
                motorR->stop();
                motorL->stop();
                return;
            }

            // [ĐLH-N] Feedforward: (V, W) profile -> (V̇, ω̇) -> τ_R, τ_L -> PWM
            const MotorCommand ff = computeInverseDynamics(current_v_profile, current_w_profile, dt);

            // [ĐH-N] Vận tốc 2 bánh mong muốn để PID bù sai số
            const WheelVelocity ref = computeInverseKinematics(current_v_profile, current_w_profile);
            targetVelR = ref.v_R;      // chỉ để log/hiển thị, trạng thái này không dùng targetVel để điều khiển
            targetVelL = ref.v_L;
            const float fbR = pidVelR->compute(ref.v_R, vR, dt);
            const float fbL = pidVelL->compute(ref.v_L, vL, dt);

            float uR = ff.pwmR + fbR;
            float uL = ff.pwmL + fbL;
            // Không đảo chiều khi mục tiêu cùng một hướng (xem runWheelPID)
            if ((ref.v_R > 0.0f && uR < 0.0f) || (ref.v_R < 0.0f && uR > 0.0f)) uR = 0.0f;
            if ((ref.v_L > 0.0f && uL < 0.0f) || (ref.v_L < 0.0f && uL > 0.0f)) uL = 0.0f;
            lastPwmR = constrain(uR, -(float)MAX_PWM, (float)MAX_PWM);
            lastPwmL = constrain(uL, -(float)MAX_PWM, (float)MAX_PWM);
            motorR->setPWM((int16_t)lastPwmR);
            motorL->setPWM((int16_t)lastPwmL);
            return;
        }
    }

    if (state == IDLE) return;  // Vừa stop() ở trên

    // PID vận tốc (low-level) cho các trạng thái dùng targetVelR/L
    lastPwmR = runWheelPID(pidVelR, motorR, targetVelR, vR, dt);
    lastPwmL = runWheelPID(pidVelL, motorL, targetVelL, vL, dt);
}

// ==========================================
// Odometry
// ==========================================
float Chassis::getX() { return x_pos; }
float Chassis::getY() { return y_pos; }
float Chassis::getTheta() { return theta; }

void Chassis::resetOdom() {
    x_pos = y_pos = theta = 0.0f;
    imu->resetYaw();
}

// Dữ liệu một chu kỳ cho log: mục tiêu và vận tốc đo của từng bánh, PWM, yaw
Chassis::Sample Chassis::getSample() {
    return { (uint8_t)state, targetVelR, encR->getLinearVelocity(), targetVelL, encL->getLinearVelocity(),
             lastPwmR, lastPwmL, imu->getYaw() * RAD_TO_DEG };
}

// ==========================================
// 2. MÔ HÌNH ĐỘNG CƠ (mô-men <-> PWM), nằm ngoài PDF
// ==========================================
float Chassis::torqueToPWM(float torque, float omega_wheel) {
    // U = (Ra/Kt)*Tau + Ke*Omega
    const float voltage = (MOTOR_RA / MOTOR_KT) * torque + MOTOR_KE * omega_wheel;
    return (voltage / VBATT_EFF) * MAX_PWM;       // VBATT_EFF = pin - sụt áp L298N
}

float Chassis::pwmToTorque(float pwm, float omega_wheel) {
    // Tau = (Kt/Ra) * (U - Ke*Omega)
    const float voltage = (pwm / MAX_PWM) * VBATT_EFF;
    return (MOTOR_KT / MOTOR_RA) * (voltage - MOTOR_KE * omega_wheel);
}

// ==========================================
// 3. ĐỘNG LỰC HỌC THUẬN  (PDF mục 3.1, phương trình (9) và (11))
//    V̇_m = (τr + τl)/(m_tong*R) - Fc/m_tong
//    ω̇_m = L*(τr - τl)/(I_tong*R) - Mc/I_tong
// ==========================================
Chassis::StateAccel Chassis::computeAccelFromTorque(float tauR, float tauL, float Vm, float Wm) {
    const float F = (tauR + tauL) / WHEEL_RADIUS;            // Fr + Fl, với F_i = τ_i / R
    const float M = (tauR - tauL) * L_HALF / WHEEL_RADIUS;   // (Fr - Fl) * L

    float Fc, Mc;
    resistance(Vm, Wm, Fc, Mc);

    StateAccel a;
    if (fabsf(Vm) > 0.001f) {
        a.a_V = (F - Fc) / ROBOT_M_TONG;                     // (9)
    } else {
        // Xe đang đứng yên: lực kéo chưa thắng ma sát tĩnh thì không có gia tốc
        a.a_V = fabsf(F) > F_MS ? (F - sgn(F) * F_MS) / ROBOT_M_TONG : 0.0f;
    }
    if (fabsf(Wm) > 0.001f) {
        a.a_W = (M - Mc) / ROBOT_I_TONG;                     // (11)
    } else {
        const float Mc0 = F_MS * L_HALF;
        a.a_W = fabsf(M) > Mc0 ? (M - sgn(M) * Mc0) / ROBOT_I_TONG : 0.0f;
    }
    return a;
}

// PWM -> τ (mô hình động cơ) -> gia tốc theo (9), (11)
Chassis::StateAccel Chassis::computeForwardDynamics(float pwmR, float pwmL, float Vm, float Wm) {
    // [ĐH-N] ω của từng bánh từ (V, W): ω_i = V_i / R
    const WheelVelocity wv = computeInverseKinematics(Vm, Wm);
    const float tauR = pwmToTorque(pwmR, wv.v_R / WHEEL_RADIUS);
    const float tauL = pwmToTorque(pwmL, wv.v_L / WHEEL_RADIUS);
    return computeAccelFromTorque(tauR, tauL, Vm, Wm);
}

// ==========================================
// 4. ĐỘNG LỰC HỌC NGHỊCH  (PDF mục 4.1, phương trình (5) và (6))
//    τr = R/2 * [ (m_tong*V̇ + Fc) + (1/L)*(I_tong*ω̇ + Mc) ]
//    τl = R/2 * [ (m_tong*V̇ + Fc) - (1/L)*(I_tong*ω̇ + Mc) ]
// ==========================================
Chassis::DynPreview Chassis::solveInverseDynamics(float V, float W, float aV, float aW) {
    float Fc, Mc;
    resistance(V, W, Fc, Mc);

    const float forceTerm  = ROBOT_M_TONG * aV + Fc;              // m_tong*V̇ + Fc
    const float momentTerm = (ROBOT_I_TONG * aW + Mc) / L_HALF;   // (I_tong*ω̇ + Mc)/L

    DynPreview d;
    d.tauR = (WHEEL_RADIUS / 2.0f) * (forceTerm + momentTerm);    // (5)
    d.tauL = (WHEEL_RADIUS / 2.0f) * (forceTerm - momentTerm);    // (6)

    // [ĐH-N] ω của từng bánh để quy đổi τ -> PWM (có bù sức phản điện động)
    const WheelVelocity wv = computeInverseKinematics(V, W);
    d.pwmR = torqueToPWM(d.tauR, wv.v_R / WHEEL_RADIUS);
    d.pwmL = torqueToPWM(d.tauL, wv.v_L / WHEEL_RADIUS);
    return d;
}

// Vòng điều khiển: gia tốc mong muốn lấy từ sự thay đổi của (V, W) mục tiêu giữa 2 chu kỳ
Chassis::MotorCommand Chassis::computeInverseDynamics(float target_Vm, float target_Wm, float dt) {
    if (dt <= 0.0001f) return {0, 0};

    const float aV = (target_Vm - prev_target_Vm) / dt;
    const float aW = (target_Wm - prev_target_Wm) / dt;
    prev_target_Vm = target_Vm;
    prev_target_Wm = target_Wm;

    const DynPreview d = solveInverseDynamics(target_Vm, target_Wm, aV, aW);
    return { d.pwmR, d.pwmL };
}

// Trạng thái duy trì (V̇ = ω̇ = 0), không đụng tới prev_target_*
Chassis::DynPreview Chassis::previewInverseDynamics(float V, float W) {
    return solveInverseDynamics(V, W, 0.0f, 0.0f);
}
