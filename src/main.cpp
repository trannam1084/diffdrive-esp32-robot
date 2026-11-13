#include <Arduino.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include "Config.h"
#include "Motor.h"
#include "Encoder.h"
#include "PID.h"
#include "IMU.h"
#include "Chassis.h"
#include "Telemetry.h"
#include "WebPage.h"

// ==========================================
// Đối tượng phần cứng
// ==========================================
Motor motorRight(PIN_MOTOR_R_EN, PIN_MOTOR_R_IN1, PIN_MOTOR_R_IN2, PWM_CHANNEL_R, false);
Motor motorLeft(PIN_MOTOR_L_EN, PIN_MOTOR_L_IN3, PIN_MOTOR_L_IN4, PWM_CHANNEL_L, false);

Encoder encRight(&ticksRight);
Encoder encLeft(&ticksLeft);

PID pidRight(PID_KP, PID_KI, PID_KD, PID_MIN_OUT, PID_MAX_OUT);
PID pidLeft(PID_KP, PID_KI, PID_KD, PID_MIN_OUT, PID_MAX_OUT);

IMU imu(IMU_I2C_ADDR);
Chassis chassis(&motorRight, &motorLeft, &encRight, &encLeft, &pidRight, &pidLeft, &imu);

AsyncWebServer server(80);
Telemetry telem;   // ghi log từng lần chạy vào RAM, tải về qua /log

// ==========================================
// Ô điều khiển và chế độ di chuyển
// ==========================================
enum Panel  : uint8_t { P_FK, P_IK, P_FD, P_ID };
enum Motion : uint8_t { M_STRAIGHT, M_SPIN, M_ARC };

static const char* const PANEL_NAMES[]  = { "fk", "ik", "fd", "id" };
static const char* const MOTION_NAMES[] = { "straight", "spin", "arc" };

static int indexOfName(const String& s, const char* const* names, int n) {
    for (int i = 0; i < n; i++) if (s == names[i]) return i;
    return -1;
}

// ==========================================
// Hộp thư lệnh: Web task (ghi) -> ControlTask (đọc)
// ==========================================
enum CmdMode : uint8_t { CMD_NONE, CMD_STOP, CMD_RUN, CMD_PWM };   // CMD_PWM: chỉ dùng để hiệu chuẩn (docs/tuning.md)

struct WebCommand {
    CmdMode mode;
    uint8_t panel;
    uint8_t motion;
    float a, b, t;
};

static portMUX_TYPE cmdMux = portMUX_INITIALIZER_UNLOCKED;
static WebCommand pendingCmd = { CMD_NONE, 0, 0, 0, 0, 0 };

static void postCommand(const WebCommand& c) {
    portENTER_CRITICAL(&cmdMux);
    pendingCmd = c;
    portEXIT_CRITICAL(&cmdMux);
}

static WebCommand takeCommand() {
    portENTER_CRITICAL(&cmdMux);
    const WebCommand c = pendingCmd;
    pendingCmd.mode = CMD_NONE;
    portEXIT_CRITICAL(&cmdMux);
    return c;
}

// ==========================================
// Kiểm tra hợp lệ (Đã nới lỏng để không bị nghẽn lệnh)
// ==========================================
static bool validateRun(int p, int m, float a, float b, float t, const char*& err) {
    const float maxA = (p == P_FK) ? MAX_WHEEL_OMEGA : (p == P_FD) ? MAX_TAU : MAX_V;
    const float maxB = (p == P_FK) ? MAX_WHEEL_OMEGA : (p == P_FD) ? MAX_TAU : MAX_W;

    if (isnan(a) || isnan(b) || isnan(t))            { err = "Gia tri khong hop le";          return false; }
    if (t < 0.1f || t > MAX_RUN_TIME_S)              { err = "Thoi gian ngoai khoang cho phep"; return false; }
    if (fabsf(a) > maxA + 0.05f || fabsf(b) > maxB + 0.05f) { err = "Vuot gioi han cho phep"; return false; }

    return true;
}

// ==========================================
// Tính các đại lượng còn lại
// ==========================================
static void addOut(String& js, const char* name, float v, const char* unit) {
    if (js.length() > 1) js += ',';
    js += "[\""; js += name; js += "\","; js += String(v, 4); js += ",\""; js += unit; js += "\"]";
}

static String computeOutputs(int p, float a, float b, float t) {
    String js = "[";
    if (p == P_FK) {
        const float vR = a * WHEEL_RADIUS, vL = b * WHEEL_RADIUS;
        const auto k = chassis.computeForwardKinematics(vR, vL);
        addOut(js, "v_R", vR, "m/s");
        addOut(js, "v_L", vL, "m/s");
        addOut(js, "V_m", k.V_m, "m/s");
        addOut(js, "omega_m", k.W_m, "rad/s");
        addOut(js, "Quang duong du kien", k.V_m * t, "m");
        addOut(js, "Goc quay du kien", k.W_m * t * RAD_TO_DEG, "do");
    } else if (p == P_IK) {
        const auto w = chassis.computeInverseKinematics(a, b);
        addOut(js, "v_R", w.v_R, "m/s");
        addOut(js, "v_L", w.v_L, "m/s");
        addOut(js, "omega_R", w.v_R / WHEEL_RADIUS, "rad/s");
        addOut(js, "omega_L", w.v_L / WHEEL_RADIUS, "rad/s");
        addOut(js, "Quang duong du kien", a * t, "m");
        addOut(js, "Goc quay du kien", b * t * RAD_TO_DEG, "do");
    } else if (p == P_FD) {
        const auto ac = chassis.computeAccelFromTorque(a, b);
        addOut(js, "Luc keo F", (a + b) / WHEEL_RADIUS, "N");
        addOut(js, "Momen M", (a - b) * L_HALF / WHEEL_RADIUS, "N.m");
        addOut(js, "Gia toc a_V", ac.a_V, "m/s2");
        addOut(js, "Gia toc goc a_W", ac.a_W, "rad/s2");
        addOut(js, "PWM_R luc khoi dong", chassis.torqueToPWM(a, 0), "");
        addOut(js, "PWM_L luc khoi dong", chassis.torqueToPWM(b, 0), "");
    } else {
        const auto w  = chassis.computeInverseKinematics(a, b);
        const auto pv = chassis.previewInverseDynamics(a, b);
        addOut(js, "v_R", w.v_R, "m/s");
        addOut(js, "v_L", w.v_L, "m/s");
        addOut(js, "tau_R", pv.tauR, "N.m");
        addOut(js, "tau_L", pv.tauL, "N.m");
        addOut(js, "PWM_R duy tri", pv.pwmR, "");
        addOut(js, "PWM_L duy tri", pv.pwmL, "");
    }
    js += ']';
    return js;
}

// ==========================================
// Wi-Fi & Web API
// ==========================================
static void sendErr(AsyncWebServerRequest* req, const char* msg) {
    req->send(400, "application/json", String("{\"ok\":false,\"err\":\"") + msg + "\"}");
}

static void setupWiFiAndWeb() {
    if (WIFI_AP_MODE) {
        WiFi.softAP(AP_SSID, AP_PASS);
        Serial.print("Da phat Wi-Fi. IP: ");
        Serial.println(WiFi.softAPIP());
    } else {
        Serial.print("Dang ket noi Wi-Fi: ");
        Serial.println(STA_SSID);
        WiFi.begin(STA_SSID, STA_PASS);
        while (WiFi.status() != WL_CONNECTED) {
            delay(500);
            Serial.print(".");
        }
        Serial.print("\nKet noi thanh cong! IP: ");
        Serial.println(WiFi.localIP());
    }

    server.on("/", HTTP_GET, [](AsyncWebServerRequest* req) {
        req->send_P(200, "text/html", index_html);
    });

    server.on("/run", HTTP_GET, [](AsyncWebServerRequest* req) {
        if (!req->hasParam("panel") || !req->hasParam("mode") || !req->hasParam("a") ||
            !req->hasParam("b") || !req->hasParam("t")) {
            sendErr(req, "Thieu tham so");
            return;
        }
        const int p = indexOfName(req->getParam("panel")->value(), PANEL_NAMES, 4);
        const int m = indexOfName(req->getParam("mode")->value(), MOTION_NAMES, 3);
        if (p < 0 || m < 0) { sendErr(req, "O hoac che do khong ton tai"); return; }

        const float a = req->getParam("a")->value().toFloat();
        const float b = req->getParam("b")->value().toFloat();
        const float t = req->getParam("t")->value().toFloat();

        const char* err = "";
        if (!validateRun(p, m, a, b, t, err)) { sendErr(req, err); return; }

        postCommand({ CMD_RUN, (uint8_t)p, (uint8_t)m, a, b, t });
        req->send(200, "application/json",
                  String("{\"ok\":true,\"out\":") + computeOutputs(p, a, b, t) + "}");
    });

    server.on("/stop", HTTP_GET, [](AsyncWebServerRequest* req) {
        postCommand({ CMD_STOP, 0, 0, 0, 0, 0 });
        req->send(200, "text/plain", "Da dung robot.");
    });

    // ---- Công cụ tune (docs/tuning.md). Nên gỡ khi nộp bản cuối ----
    // Đổi PID khi đang chạy: /pid?kp=120&ki=400&kd=0
    server.on("/pid", HTTP_GET, [](AsyncWebServerRequest* req) {
        if (!req->hasParam("kp") || !req->hasParam("ki") || !req->hasParam("kd")) {
            req->send(400, "text/plain", "Thieu kp, ki, kd");
            return;
        }
        const float kp = req->getParam("kp")->value().toFloat();
        const float ki = req->getParam("ki")->value().toFloat();
        const float kd = req->getParam("kd")->value().toFloat();
        if (kp < 0 || kp > 1000 || ki < 0 || ki > 2000 || kd < 0 || kd > 50) {
            req->send(400, "text/plain", "He so ngoai khoang");
            return;
        }
        pidRight.setGains(kp, ki, kd);
        pidLeft.setGains(kp, ki, kd);
        req->send(200, "text/plain", "OK");
    });

    // Chạy PWM thô để đo đường cong PWM -> vận tốc: /pwm?r=160&l=160&t=2
    server.on("/pwm", HTTP_GET, [](AsyncWebServerRequest* req) {
        if (!req->hasParam("r") || !req->hasParam("l") || !req->hasParam("t")) {
            req->send(400, "text/plain", "Thieu r, l, t");
            return;
        }
        const float r = req->getParam("r")->value().toFloat();
        const float l = req->getParam("l")->value().toFloat();
        const float t = req->getParam("t")->value().toFloat();
        if (fabsf(r) > 255 || fabsf(l) > 255 || t < 0.5f || t > 5.0f) {
            req->send(400, "text/plain", "Ngoai khoang");
            return;
        }
        postCommand({ CMD_PWM, 0, 0, r, l, t });
        req->send(200, "text/plain", "OK");
    });

    // Tải log của lần chạy gần nhất (CSV): http://192.168.4.1/log
    // Chỉ tải được khi xe đã dừng. Phản hồi theo từng khối nên không cần giữ cả file trong RAM.
    server.on("/log", HTTP_GET, [](AsyncWebServerRequest* req) {
        if (telem.recording()) { req->send(409, "text/plain", "Dang ghi, doi xe dung roi tai lai"); return; }
        if (telem.count() == 0) { req->send(404, "text/plain", "Chua co log"); return; }

        AsyncWebServerResponse* res = req->beginChunkedResponse("text/csv",
            [](uint8_t* buffer, size_t maxLen, size_t index) -> size_t {
                static uint16_t row;
                static bool headerSent;
                if (index == 0) { row = 0; headerSent = false; }

                size_t w = 0;
                if (!headerSent) {
                    const char* h = Telemetry::header();
                    const size_t hl = strlen(h);
                    if (hl > maxLen) return 0;
                    memcpy(buffer, h, hl);
                    w = hl;
                    headerSent = true;
                }
                while (row < telem.count()) {
                    char line[96];
                    const size_t l = telem.formatRow(row, line, sizeof(line));
                    if (l == 0 || w + l > maxLen) break;   // dòng sau để lượt gọi kế tiếp
                    memcpy(buffer + w, line, l);
                    w += l;
                    row++;
                }
                return w;    // 0 = hết dữ liệu
            });
        res->addHeader("Content-Disposition", "attachment; filename=run.csv");
        req->send(res);
    });

    server.on("/status", HTTP_GET, [](AsyncWebServerRequest* req) {
        const auto k = chassis.getMeasuredKinematics();
        char buf[260];
        snprintf(buf, sizeof(buf),
                 "{\"run\":%d,\"left\":%.2f,\"theta\":%.2f,\"V\":%.3f,\"W\":%.3f,"
                 "\"vr\":%.3f,\"vl\":%.3f,\"x\":%.3f,\"y\":%.3f,\"log\":%u,\"rec\":%d}",
                 chassis.isIdle() ? 0 : 1, chassis.getRemainingS(),
                 chassis.getTheta() * RAD_TO_DEG, k.V_m, k.W_m,
                 k.V_m + k.W_m * L_HALF, k.V_m - k.W_m * L_HALF,
                 chassis.getX(), chassis.getY(),
                 (unsigned)telem.count(), telem.recording() ? 1 : 0);
        req->send(200, "application/json", buf);
    });

    server.begin();
    Serial.println("Web Server da san sang.");
}

// ==========================================
// Vòng điều khiển thời gian thực
// ==========================================
static void applyRun(const WebCommand& c) {
    chassis.stop();

    const bool hold = (c.motion == M_STRAIGHT) && (c.panel != P_FD);
    chassis.beginRun((uint32_t)(c.t * 1000.0f), hold);

    switch (c.panel) {
        case P_FK: chassis.setWheelVelocities(c.a * WHEEL_RADIUS, c.b * WHEEL_RADIUS); break;
        case P_IK: chassis.setVelocityVW(c.a, c.b);   break;
        case P_FD: chassis.setTorques(c.a, c.b);      break;
        case P_ID: chassis.setDynamicsVW(c.a, c.b);   break;
    }
    telem.start();   // bắt đầu ghi log cho lần chạy này
}

static void ControlTask(void* pvParameters) {
    TickType_t lastWake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(1000 / CONTROL_LOOP_HZ);
    const float dt = 1.0f / CONTROL_LOOP_HZ;

    for (;;) {
        const WebCommand cmd = takeCommand();
        if (cmd.mode == CMD_RUN)       applyRun(cmd);
        else if (cmd.mode == CMD_STOP) chassis.stop();
        else if (cmd.mode == CMD_PWM) {
            chassis.stop();
            chassis.beginRun((uint32_t)(cmd.t * 1000.0f), false);
            chassis.setOpenLoopPWM(cmd.a, cmd.b);
            telem.start();
        }

        chassis.update(dt);

        // Ghi log mỗi chu kỳ; dừng ghi khi xe đã dừng (mẫu cuối là lúc vừa dừng)
        if (telem.recording()) {
            const Chassis::Sample s = chassis.getSample();
            telem.record(s.state, s.tgtR, s.vR, s.tgtL, s.vL, s.pwmR, s.pwmL, s.yawDeg);
            if (chassis.isIdle()) telem.stop();
        }
        vTaskDelayUntil(&lastWake, period);
    }
}

void setup() {
    Serial.begin(115200);
    delay(1000);

    motorRight.begin();
    motorLeft.begin();
    initEncoders();
    chassis.begin();

    if (telem.begin(TELEM_MAX_SAMPLES)) {
        Serial.printf("Telemetry: %d mau (heap trong: %u byte)\n", TELEM_MAX_SAMPLES, (unsigned)ESP.getFreeHeap());
    } else {
        Serial.println("CANH BAO: khong du bo nho cho Telemetry, /log se khong dung duoc.");
    }

    // Khởi động Wi-Fi và Web Server trước
    setupWiFiAndWeb();

    // Khởi tạo IMU an toàn, khớp với file IMU.cpp của bạn (mặc định lấy 200 mẫu)
    if (imu.begin(PIN_IMU_SDA, PIN_IMU_SCL)) {
        Serial.println("Dang hieu chuan Gyro - GIU XE DUNG YEN...");
        delay(500);
        imu.calibrateGyro(200);
        Serial.println("Hieu chuan IMU xong!");
    } else {
        Serial.println("LOI: Khong tim thay IMU BMI160!");
    }

    // Tạo Task điều khiển sau cùng
    xTaskCreatePinnedToCore(ControlTask, "ControlTask", 8192, NULL, 1, NULL, 1);
}

void loop() {
    vTaskDelete(NULL);
}
