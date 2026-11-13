#ifndef SECRETS_H
#define SECRETS_H

// Sao chép file này thành include/secrets.h rồi điền thông tin thật.
// secrets.h đã nằm trong .gitignore nên mật khẩu không bị đưa lên GitHub.

// true  = xe tự phát Wi-Fi (Access Point), vào http://192.168.4.1
// false = xe nối Wi-Fi nhà (Station), xem IP trên Serial Monitor
#define WIFI_AP_MODE        true

// Chế độ Access Point (mật khẩu tối thiểu 8 ký tự)
#define AP_SSID             "ROBOT_ESP32"
#define AP_PASS             "doi-mat-khau-nay"

// Chế độ Station (chỉ Wi-Fi 2.4 GHz)
#define STA_SSID            ""
#define STA_PASS            ""

#endif // SECRETS_H
