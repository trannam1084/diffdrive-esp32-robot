#ifndef CHASSIS_H
#define CHASSIS_H

#include <Arduino.h>
#include "Motor.h"
#include "Encoder.h"
#include "PID.h"
#include "IMU.h"
#include "Config.h"

enum ChassisState {
    IDLE,
    DRIVING_DIST,       // Đi thẳng một quãng đường
    TURNING_ANGLE,      // Xoay một góc
    VELOCITY_CONTROL,   // Động lực học nghịch: profile + feedforward + PID bù
    KINEMATIC_CONTROL,  // Động học: đặt vận tốc, PID bánh bám theo
    TORQUE_CONTROL,     // Động lực học thuận: đặt mô-men (mô phỏng mô hình, PID bám theo)
    OPENLOOP_PWM        // Chỉ để đo đường cong PWM -> vận tốc (hiệu chuẩn)
};

class Chassis {
private:
    Motor* motorR;
    Motor* motorL;
    Encoder* encR;
    Encoder* encL;
    PID* pidVelR;      // PID vận tốc bánh phải (low-level)
    PID* pidVelL;
    IMU* imu;
    PID pidTurn;       // PID góc xoay (high-level)

    ChassisState state;

    // Vận tốc mục tiêu 2 bánh (m/s)
    float targetVelR, targetVelL;

    // Điều khiển quãng đường
    float startDistR, startDistL, targetDistance;

    // Điều khiển góc
    float targetYaw;   // độ

    uint32_t actionStartTime;

    // Odometry
    float x_pos, y_pos, theta;

    // Profile vận tốc (động lực học nghịch)
    float target_v_final;
    float target_w_final;
    float current_v_profile;
    float current_w_profile;

    // Mục tiêu vận tốc trước đó (để tính gia tốc mong muốn cho động lực học nghịch)
    float prev_target_Vm, prev_target_Wm;

    // Lệnh chạy có thời hạn từ web
    uint32_t runStartMs = 0, runDurationMs = 0;
    bool  holdHeading = false;      // true: giữ theta = 0 theo IMU (đi thẳng)
    float kinV = 0, kinW = 0;       // (V, W) của chế độ động học
    float cmdTauR = 0, cmdTauL = 0; // mô-men đặt (chế độ động lực học thuận)
    float lastPwmR = 0, lastPwmL = 0;
    float cmdPwmR = 0, cmdPwmL = 0; // PWM đặt trực tiếp (chế độ OPENLOOP_PWM)

    // Đổi (V, W) -> vận tốc 2 bánh mà KHÔNG đổi state
    void applyVW(float v, float w);
    // Chạy 1 vòng PID vận tốc cho một bánh
    // Trả về PWM đã cấp cho motor (để ghi log)
    float runWheelPID(PID* pid, Motor* motor, float target, float measured, float dt);
    // PWM nền theo vận tốc mục tiêu của bánh: OFFSET + GAIN*|v|
    float wheelFeedforward(float v);
    float traveledDistance();
    bool timedOut(uint32_t timeout_ms);

    void applyDynamicsProfile(float dt);

public:
    Chassis(Motor* mR, Motor* mL, Encoder* eR, Encoder* eL, PID* pR, PID* pL, IMU* i);

    void begin();

    // Gọi mỗi chu kỳ điều khiển (20ms)
    void update(float dt);

    // Dừng xe, về IDLE
    void stop();

    // Bắt đầu một lệnh có thời hạn: gọi SAU stop(), TRƯỚC các hàm set*.
    // Reset odometry + yaw (theta = 0), hết thời gian thì tự stop().
    void beginRun(uint32_t ms, bool hold);
    float getRemainingS();

    // Động học: đặt (V, W) hoặc vận tốc 2 bánh, PID vận tốc bánh bám theo
    void setKinematicVW(float v, float w);
    void setVelocityVW(float v, float w);
    void setWheelVelocities(float vR, float vL);

    // Động lực học nghịch: (V, W) mục tiêu -> profile + feedforward + PID bù
    void setDynamicsVW(float v_target, float w_target);

    // Động lực học thuận: đặt mô-men từng bánh (N.m), chạy hở
    void setTorques(float tauR, float tauL);

    // Đặt PWM hai bánh trực tiếp (hiệu chuẩn: đo vận tốc ổn định ứng với từng mức PWM)
    void setOpenLoopPWM(float pwmR, float pwmL);

    // Đi một quãng đường (m, âm = lùi) với tốc độ (m/s)
    void driveDistance(float distance_m, float speed);

    // Xoay tương đối một góc (độ, dương = ngược chiều kim đồng hồ)
    void turnAngle(float angle_deg);

    bool isIdle();

    // Odometry
    float getX();
    float getY();
    float getTheta();   // rad, lấy từ IMU
    void resetOdom();

    // Mẫu dữ liệu của một chu kỳ, dùng để ghi log (Telemetry)
    struct Sample { uint8_t state; float tgtR, vR, tgtL, vL, pwmR, pwmL, yawDeg; };
    Sample getSample();

    // ==========================================
    // 1. ĐỘNG HỌC (PDF: "Động học thuận" / "Động học nghịch")
    // ==========================================
    struct KinematicsState { float V_m; float W_m; };
    // Động học thuận: (v_R, v_L) -> (V_m, ω_m)
    KinematicsState computeForwardKinematics(float v_R, float v_L);

    struct WheelVelocity { float v_R; float v_L; };
    // Động học nghịch: (V_m, ω_m) -> (v_R, v_L)
    WheelVelocity computeInverseKinematics(float v_m, float w_m);

    // (V, W) thực tế từ Encoder
    KinematicsState getMeasuredKinematics();

    // ==========================================
    // 2. ĐỘNG LỰC HỌC (PDF mục 3 và 4)
    // ==========================================
    // Mô hình động cơ điện: mô-men <-> PWM
    float torqueToPWM(float torque, float omega_wheel);
    float pwmToTorque(float pwm, float omega_wheel);

    // Động lực học thuận: (τ_R, τ_L) -> gia tốc (V̇_m, ω̇_m).
    // Vm, Wm = vận tốc hiện tại; mặc định 0 = xe đang đứng yên (xét ma sát tĩnh).
    struct StateAccel { float a_V; float a_W; };
    StateAccel computeAccelFromTorque(float tauR, float tauL, float Vm = 0.0f, float Wm = 0.0f);
    // Như trên nhưng đầu vào là PWM
    StateAccel computeForwardDynamics(float pwmR, float pwmL, float Vm, float Wm);

    // Động lực học nghịch: (V, W, V̇, ω̇) -> (τ_R, τ_L) và PWM tương ứng
    struct DynPreview { float tauR, tauL, pwmR, pwmL; };
    DynPreview solveInverseDynamics(float V, float W, float aV, float aW);

    // Dùng trong vòng điều khiển: tự tính V̇, ω̇ từ sự thay đổi của (V, W) mục tiêu
    struct MotorCommand { float pwmR; float pwmL; };
    MotorCommand computeInverseDynamics(float target_Vm, float target_Wm, float dt);

    // Mô-men và PWM cần để DUY TRÌ (V, W) (gia tốc = 0), không đổi trạng thái nội bộ
    DynPreview previewInverseDynamics(float V, float W);
};

#endif
