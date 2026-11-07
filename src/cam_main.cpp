#include <Arduino.h>
#include <WiFi.h>
#include <ArduinoOTA.h>
#include <math.h>
#include "esp_camera.h"
#include "esp_http_server.h"
#include "img_converters.h"
#include "esp_log.h"
#include "driver/ledc.h"
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"

// ============================================================================
// Aideepen ESP32-CAM-MB (AI-Thinker OV2640) Companion Streamer Firmware
// Automatically joins the ESP32-S3 "RC-Rover" SoftAP at static IP 192.168.4.2
// and serves a low-latency CORS-enabled MJPEG stream at:
//   http://192.168.4.2:81/stream
// and a single-frame JPEG snapshot endpoint (for home Wi-Fi relay) at:
//   http://192.168.4.2/capture
// ============================================================================

namespace {

constexpr const char* ROVER_AP_SSID = "RC-Rover";
constexpr const char* ROVER_AP_PASS = "rover1234";

const IPAddress CAM_LOCAL_IP(192, 168, 4, 2);
const IPAddress CAM_GATEWAY(192, 168, 4, 1);
const IPAddress CAM_SUBNET(255, 255, 255, 0);

// Onboard small red status LED on the back of AI-Thinker ESP32-CAM (active LOW)
constexpr int RED_LED_GPIO = 33;
// Onboard top-facing flash LED (active HIGH) - used for brief 8ms boot/Wi-Fi visual pulses
constexpr int FLASH_LED_GPIO = 4;

void pulseTopLed(int count) {
    pinMode(FLASH_LED_GPIO, OUTPUT);
    for (int i = 0; i < count; ++i) {
        digitalWrite(FLASH_LED_GPIO, HIGH);
        delay(8);
        digitalWrite(FLASH_LED_GPIO, LOW);
        if (i + 1 < count) {
            delay(140);
        }
    }
}

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
httpd_handle_t captureServer = nullptr;

bool cameraInitialized = false;
esp_err_t cameraInitErr = ESP_OK;

// On-chip 1/8th scale RGB888 buffer (40x30x3 = 3600 bytes) for headless optical floor-horizon analysis
constexpr int VIS_W = 40;
constexpr int VIS_H = 30;
uint8_t* g_visRgbBuf = nullptr;
volatile int g_visLeftPct = 100;
volatile int g_visCenterPct = 100;
volatile int g_visRightPct = 100;

void unpackRgb565(const uint8_t* buf, int pixelIdx, int& r, int& g, int& b) {
    const uint16_t px = static_cast<uint16_t>(buf[pixelIdx * 2]) |
                        (static_cast<uint16_t>(buf[pixelIdx * 2 + 1]) << 8);
    r = ((px >> 11) & 0x1F) << 3;
    g = ((px >> 5)  & 0x3F) << 2;
    b = (px & 0x1F) << 3;
}

void computeOpticalFloorHorizon(const camera_fb_t* fb) {
    if (!fb || !g_visRgbBuf) {
        return;
    }
    // Fast 1/8th scale JPEG decode (320x240 -> 40x30 RGB565)
    if (!jpg2rgb565(fb->buf, fb->len, g_visRgbBuf, JPG_SCALE_8X)) {
        return;
    }

    int refR = 0, refG = 0, refB = 0, count = 0;
    for (int ry = 2; ry <= 5; ++ry) {
        for (int rx = 15; rx <= 24; ++rx) {
            int pr = 0, pg = 0, pb = 0;
            unpackRgb565(g_visRgbBuf, ry * VIS_W + rx, pr, pg, pb);
            refR += pr;
            refG += pg;
            refB += pb;
            ++count;
        }
    }
    if (count > 0) {
        refR /= count;
        refG /= count;
        refB /= count;
    }

    constexpr int COLS = 16;
    float colNorm[COLS];
    for (int c = 0; c < COLS; ++c) {
        int rawX = VIS_W - 1 - ((c * VIS_W + (COLS / 2)) / COLS);
        if (rawX < 0) rawX = 0;
        if (rawX >= VIS_W) rawX = VIS_W - 1;

        int hitSteps = 22;
        for (int step = 0; step < 22; ++step) {
            const int rawY = 4 + step;
            int pr = 0, pg = 0, pb = 0;
            unpackRgb565(g_visRgbBuf, rawY * VIS_W + rawX, pr, pg, pb);
            const int dr = abs(pr - refR);
            const int dg = abs(pg - refG);
            const int db = abs(pb - refB);
            if (dr + dg + db > 75) {
                hitSteps = step;
                break;
            }
        }
        colNorm[c] = fmaxf(0.10f, static_cast<float>(hitSteps) / 22.0f);
    }

    float leftSum = 0.0f, centerSum = 0.0f, rightSum = 0.0f;
    for (int i = 0; i < 6; ++i) leftSum += colNorm[i];
    for (int i = 6; i <= 9; ++i) centerSum += colNorm[i];
    for (int i = 10; i < 16; ++i) rightSum += colNorm[i];

    g_visLeftPct   = static_cast<int>((leftSum / 6.0f) * 100.0f);
    g_visCenterPct = static_cast<int>((centerSum / 4.0f) * 100.0f);
    g_visRightPct  = static_cast<int>((rightSum / 6.0f) * 100.0f);
}

esp_err_t statusHandler(httpd_req_t* req) {
    char buf[220];
    snprintf(buf,
             sizeof(buf),
             "{\"camInit\":%s,\"err\":%d,\"psram\":%s,\"freeHeap\":%u,\"visL\":%d,\"visC\":%d,\"visR\":%d}",
             cameraInitialized ? "true" : "false",
             static_cast<int>(cameraInitErr),
             psramFound() ? "true" : "false",
             static_cast<unsigned>(ESP.getFreeHeap()),
             g_visLeftPct,
             g_visCenterPct,
             g_visRightPct);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(req, "Connection", "close");
    return httpd_resp_send(req, buf, strlen(buf));
}

esp_err_t captureHandler(httpd_req_t* req) {
    camera_fb_t* fb = esp_camera_fb_get();
    if (!fb) {
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    computeOpticalFloorHorizon(fb);

    char visHdr[40];
    snprintf(visHdr, sizeof(visHdr), "%d:%d:%d", g_visLeftPct, g_visCenterPct, g_visRightPct);

    httpd_resp_set_type(req, "image/jpeg");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store, no-cache, must-revalidate, max-age=0");
    httpd_resp_set_hdr(req, "X-Vis", visHdr);
    httpd_resp_set_hdr(req, "Connection", "close");

    esp_err_t res = httpd_resp_send(req, reinterpret_cast<const char*>(fb->buf), fb->len);
    esp_camera_fb_return(fb);
    return res;
}

esp_err_t streamHandler(httpd_req_t* req) {
    camera_fb_t* fb = nullptr;
    esp_err_t res = httpd_resp_set_type(req, STREAM_CONTENT_TYPE);
    if (res != ESP_OK) {
        return res;
    }

    // Critical: Allow the dashboard <canvas> to inspect pixels for Target Lock & Vision Avoidance
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
    httpd_uri_t streamUri = {
        .uri       = "/stream",
        .method    = HTTP_GET,
        .handler   = streamHandler,
        .user_ctx  = nullptr
    };

    httpd_uri_t captureUri = {
        .uri       = "/capture",
        .method    = HTTP_GET,
        .handler   = captureHandler,
        .user_ctx  = nullptr
    };

    httpd_uri_t statusUri = {
        .uri       = "/status",
        .method    = HTTP_GET,
        .handler   = statusHandler,
        .user_ctx  = nullptr
    };

    httpd_config_t streamConfig = HTTPD_DEFAULT_CONFIG();
    streamConfig.server_port = 81;
    streamConfig.ctrl_port = 32769;
    streamConfig.lru_purge_enable = true;

    if (httpd_start(&streamServer, &streamConfig) == ESP_OK) {
        httpd_register_uri_handler(streamServer, &streamUri);
        httpd_register_uri_handler(streamServer, &captureUri);
        httpd_register_uri_handler(streamServer, &statusUri);
    }

    httpd_config_t capConfig = HTTPD_DEFAULT_CONFIG();
    capConfig.server_port = 80;
    capConfig.ctrl_port = 32768;
    capConfig.lru_purge_enable = true;

    if (httpd_start(&captureServer, &capConfig) == ESP_OK) {
        httpd_register_uri_handler(captureServer, &captureUri);
        httpd_register_uri_handler(captureServer, &statusUri);
    }
}

bool initCamera() {
    if (cameraInitialized) {
        return true;
    }
    camera_config_t config = {};
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
    config.sccb_i2c_port = -1;
    config.jpeg_buffer_size = 0;

    // Official CameraWebServer.ino pattern:
    // Allocate UXGA-sized PSRAM frame buffers during init so DMA never overflows (FB-OVF),
    // then immediately drop sensor resolution to QVGA (320x240) after init!
    if (psramFound()) {
        config.frame_size = FRAMESIZE_UXGA;
        config.jpeg_quality = 12;
        config.fb_location = CAMERA_FB_IN_PSRAM;
        config.fb_count = 2;
        config.grab_mode = CAMERA_GRAB_LATEST;
    } else {
        config.frame_size = FRAMESIZE_SVGA;
        config.jpeg_quality = 14;
        config.fb_location = CAMERA_FB_IN_DRAM;
        config.fb_count = 1;
        config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
    }

    for (int attempt = 0; attempt < 3; ++attempt) {
        // Power-cycle the OV2640 sensor via PWDN (GPIO 32) so warm resets never leave SCCB stuck (0x106)
        pinMode(PWDN_GPIO_NUM, OUTPUT);
        digitalWrite(PWDN_GPIO_NUM, HIGH);
        delay(50);
        digitalWrite(PWDN_GPIO_NUM, LOW);
        delay(50);

        cameraInitErr = esp_camera_init(&config);
        if (cameraInitErr == ESP_OK) {
            sensor_t* s = esp_camera_sensor_get();
            if (s != nullptr) {
                s->set_framesize(s, FRAMESIZE_QVGA);
            }
            cameraInitialized = true;
            delay(150);
            camera_fb_t* testFb = esp_camera_fb_get();
            if (testFb != nullptr) {
                Serial.printf("OV2640 Test Frame OK: %u bytes!\n", static_cast<unsigned>(testFb->len));
                esp_camera_fb_return(testFb);
            } else {
                Serial.println("OV2640 Test Frame returned null");
            }
            return true;
        }
        esp_camera_deinit();
        delay(100);
    }
    return false;
}

void connectToRoverAp() {
    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);
    WiFi.config(CAM_LOCAL_IP, CAM_GATEWAY, CAM_SUBNET);
    WiFi.begin(ROVER_AP_SSID, ROVER_AP_PASS);
    // Low TX power spike for 3.3V jumper operation while still reaching across the room
    WiFi.setTxPower(WIFI_POWER_11dBm);

    Serial.printf("Connecting ESP32-CAM to %s", ROVER_AP_SSID);
    uint32_t startMs = millis();
    while (WiFi.status() != WL_CONNECTED && (millis() - startMs < 10000)) {
        digitalWrite(RED_LED_GPIO, !digitalRead(RED_LED_GPIO));
        delay(250);
        Serial.print(".");
    }
    Serial.println();

    if (WiFi.status() == WL_CONNECTED) {
        digitalWrite(RED_LED_GPIO, LOW); // Solid red LED on back = connected!
        pulseTopLed(2);                  // 2 quick flashes on top LED = Wi-Fi connected!
        Serial.printf("ESP32-CAM Online! Stream URL: http://%s:81/stream\n", WiFi.localIP().toString().c_str());
    }
}

} // namespace

void setup() {
    // CRITICAL: Disable brownout detector immediately so 3.3V jumper wire resistance
    // doesn't reset the ESP32-CAM when the 2.4GHz Wi-Fi radio initializes
    WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
    esp_log_level_set("cam_hal", ESP_LOG_NONE);

    pinMode(RED_LED_GPIO, OUTPUT);
    digitalWrite(RED_LED_GPIO, LOW); // Turn on rear red status LED on boot
    pulseTopLed(1);                  // 1 quick flash on top LED = Power ON!

    Serial.begin(115200);
    Serial.println("\n--- RC-Rover ESP32-CAM-MB Companion Streamer ---");

    g_visRgbBuf = static_cast<uint8_t*>(malloc(VIS_W * VIS_H * 3));

    // 1. Initialize OV2640 with GPIO32 power-cycle and compact JPEG quality (no FB-OVF)
    if (!initCamera()) {
        Serial.printf("ERROR: OV2640 Camera initialization failed (0x%x)!\n", cameraInitErr);
    } else {
        Serial.println("OV2640 Camera initialized OK!");
    }

    // 2. Connect to RC-Rover SoftAP and start HTTP + OTA servers
    connectToRoverAp();
    startCameraStreamServer();
    ArduinoOTA.setHostname("rc-rover-cam");
    ArduinoOTA.begin();
}

void loop() {
    static uint32_t lastCheckMs = 0;
    ArduinoOTA.handle();
    if (millis() - lastCheckMs >= 2000) {
        lastCheckMs = millis();
        // Automatically reconnect if the ESP32-S3 Rover controller reboots
        if (WiFi.status() != WL_CONNECTED) {
            connectToRoverAp();
        }
    }
    delay(20);
}
