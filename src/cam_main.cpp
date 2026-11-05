#include <Arduino.h>
#include <WiFi.h>
#include "esp_camera.h"
#include "esp_http_server.h"

// ============================================================================
// Aideepen ESP32-CAM-MB (AI-Thinker OV2640) Companion Streamer Firmware
// Automatically joins the ESP32-S3 "RC-Rover" SoftAP at static IP 192.168.4.2
// and serves a low-latency CORS-enabled MJPEG stream at:
//   http://192.168.4.2:81/stream
// ============================================================================

namespace {

constexpr const char* ROVER_AP_SSID = "RC-Rover";
constexpr const char* ROVER_AP_PASS = "rover1234";

const IPAddress CAM_LOCAL_IP(192, 168, 4, 2);
const IPAddress CAM_GATEWAY(192, 168, 4, 1);
const IPAddress CAM_SUBNET(255, 255, 255, 0);

// AI-Thinker ESP32-CAM Pin Definition
constexpr int PWDN_GPIO_NUM  = 32;
constexpr int RESET_GPIO_NUM = -1;
constexpr int XCLK_GPIO_NUM  = 0;
constexpr int SIOD_GPIO_NUM  = 26;
constexpr int SIOC_GPIO_NUM  = 27;
constexpr int Y9_GPIO_NUM    = 35;
constexpr int Y8_GPIO_NUM    = 34;
constexpr int Y7_GPIO_NUM    = 39;
constexpr int Y6_GPIO_NUM    = 36;
constexpr int Y5_GPIO_NUM    = 21;
constexpr int Y4_GPIO_NUM    = 19;
constexpr int Y3_GPIO_NUM    = 18;
constexpr int Y2_GPIO_NUM    = 5;
constexpr int VSYNC_GPIO_NUM = 25;
constexpr int HREF_GPIO_NUM  = 23;
constexpr int PCLK_GPIO_NUM  = 22;

constexpr const char* STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=frame";
constexpr const char* STREAM_BOUNDARY = "\r\n--frame\r\n";
constexpr const char* STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

httpd_handle_t streamServer = nullptr;

esp_err_t streamHandler(httpd_req_t* req) {
    camera_fb_t* fb = nullptr;
    esp_err_t res = httpd_resp_set_type(req, STREAM_CONTENT_TYPE);
    if (res != ESP_OK) {
        return res;
    }

    // Critical: Allow the 192.168.4.1 dashboard <canvas> to inspect pixels for Target Lock
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store, no-cache, must-revalidate, max-age=0");

    char partBuf[64];

    while (true) {
        fb = esp_camera_fb_get();
        if (!fb) {
            delay(10);
            continue;
        }

        size_t hlen = snprintf(partBuf, sizeof(partBuf), STREAM_PART, fb->len);
        res = httpd_resp_send_chunk(req, STREAM_BOUNDARY, strlen(STREAM_BOUNDARY));
        if (res == ESP_OK) {
            res = httpd_resp_send_chunk(req, partBuf, hlen);
        }
        if (res == ESP_OK) {
            res = httpd_resp_send_chunk(req, reinterpret_cast<const char*>(fb->buf), fb->len);
        }

        esp_camera_fb_return(fb);
        fb = nullptr;

        if (res != ESP_OK) {
            break;
        }
    }

    return res;
}

void startCameraStreamServer() {
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = 81;
    config.ctrl_port = 32769;

    httpd_uri_t streamUri = {
        .uri       = "/stream",
        .method    = HTTP_GET,
        .handler   = streamHandler,
        .user_ctx  = nullptr
    };

    if (httpd_start(&streamServer, &config) == ESP_OK) {
        httpd_register_uri_handler(streamServer, &streamUri);
    }
}

bool initCamera() {
    camera_config_t config;
    config.ledc_channel = LEDC_CHANNEL_0;
    config.ledc_timer = LEDC_TIMER_0;
    config.pin_d0 = Y2_GPIO_NUM;
    config.pin_d1 = Y3_GPIO_NUM;
    config.pin_d2 = Y4_GPIO_NUM;
    config.pin_d3 = Y5_GPIO_NUM;
    config.pin_d4 = Y6_GPIO_NUM;
    config.pin_d5 = Y7_GPIO_NUM;
    config.pin_d6 = Y8_GPIO_NUM;
    config.pin_d7 = Y9_GPIO_NUM;
    config.pin_xclk = XCLK_GPIO_NUM;
    config.pin_pclk = PCLK_GPIO_NUM;
    config.pin_vsync = VSYNC_GPIO_NUM;
    config.pin_href = HREF_GPIO_NUM;
    config.pin_sccb_sda = SIOD_GPIO_NUM;
    config.pin_sccb_scl = SIOC_GPIO_NUM;
    config.pin_pwdn = PWDN_GPIO_NUM;
    config.pin_reset = RESET_GPIO_NUM;
    config.xclk_freq_hz = 20000000;
    config.pixel_format = PIXFORMAT_JPEG;

    // QVGA (320x240) gives ultra-low latency (~25fps) for real-time FPV & browser target tracking
    config.frame_size = FRAMESIZE_QVGA;
    config.jpeg_quality = 12;
    config.fb_count = psramFound() ? 2 : 1;
    config.grab_mode = CAMERA_GRAB_LATEST;

    return (esp_camera_init(&config) == ESP_OK);
}

void connectToRoverAp() {
    WiFi.mode(WIFI_STA);
    WiFi.config(CAM_LOCAL_IP, CAM_GATEWAY, CAM_SUBNET);
    WiFi.begin(ROVER_AP_SSID, ROVER_AP_PASS);

    Serial.printf("Connecting ESP32-CAM to %s", ROVER_AP_SSID);
    uint32_t startMs = millis();
    while (WiFi.status() != WL_CONNECTED && (millis() - startMs < 10000)) {
        delay(250);
        Serial.print(".");
    }
    Serial.println();

    if (WiFi.status() == WL_CONNECTED) {
        Serial.printf("ESP32-CAM Online! Stream URL: http://%s:81/stream\n", WiFi.localIP().toString().c_str());
    }
}

} // namespace

void setup() {
    Serial.begin(115200);
    Serial.println("\n--- RC-Rover ESP32-CAM-MB Companion Streamer ---");

    if (!initCamera()) {
        Serial.println("ERROR: OV2640 Camera initialization failed!");
        return;
    }

    connectToRoverAp();
    startCameraStreamServer();
}

void loop() {
    // Automatically reconnect if the ESP32-S3 Rover controller reboots
    if (WiFi.status() != WL_CONNECTED) {
        connectToRoverAp();
    }
    delay(1000);
}
