#pragma once

#include <Arduino.h>
#include <WiFi.h>
#include <Preferences.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>

enum class RoverMode : uint8_t {
    STANDBY = 0,
    AUTONOMOUS = 1,
    FOLLOW = 2,
    MANUAL = 3
};

struct ManualCommand {
    int8_t drive = 0; // -1 = reverse, 0 = stop, 1 = forward
    int8_t steer = 0; // -1 = left, 0 = center, 1 = right
};

struct TelemetrySnapshot {
    float frontCm = 400.0f;
    float rearCm = 400.0f;
    float sectorLeftCm = 400.0f;
    float sectorCenterCm = 400.0f;
    float sectorRightCm = 400.0f;
    float headingDeg = 0.0f;
    int8_t goalSteer = 0;
    int8_t driveDir = 0;
    int8_t steerDir = 0;
    const char* stateLabel = "INIT";
    bool watchdogTripped = false;
};

class WebInterface {
public:
    // SoftAP Credentials & Static IP Configuration
    static constexpr const char* AP_SSID = "RC-Rover";
    static constexpr const char* AP_PASS = "rover1234"; // Minimum 8 chars for WPA2

    // 500ms Failsafe Watchdog Timeout for Manual Wi-Fi Control
    static constexpr uint32_t WATCHDOG_TIMEOUT_MS = 500;
    static constexpr uint32_t TELEMETRY_INTERVAL_MS = 90;

    WebInterface();

    void begin();
    void update(const TelemetrySnapshot& telemetry);

    RoverMode getMode() const { return _mode; }
    void activateAutonomousRoam() { _mode = RoverMode::AUTONOMOUS; }
    bool hasUserInteracted() const { return _userInteracted; }

    uint8_t getThrottleTrim() const { return _throttleTrim; }
    ManualCommand getManualCommand() const { return _manualCmd; }

    // Camera-assisted target steering (-1 = left, 0 = center, 1 = right) for Follow-Me mode
    int8_t getFollowTargetSteer() const;
    bool hasActiveFollowTarget() const;

    // Camera + Sonar sensor fusion clearance estimates from optical floor segmentation
    bool hasVisionSectorFusion() const;
    float getVisionLeftCm() const { return _visionLeftCm; }
    float getVisionRightCm() const { return _visionRightCm; }

    // Returns true if manual heartbeat has expired (> 500ms since last client packet)
    bool isManualWatchdogExpired() const;
    void clearManualCommand();

private:
    AsyncWebServer _server;
    AsyncWebSocket _ws;

    volatile RoverMode _mode = RoverMode::STANDBY;
    volatile bool _userInteracted = false;
    volatile uint8_t _throttleTrim = 135;
    ManualCommand _manualCmd;
    volatile int8_t _followTargetSteer = 0;
    volatile bool _followTargetLocked = false;
    volatile uint32_t _lastFollowUpdateMs = 0;
    volatile float _visionLeftCm = 150.0f;
    volatile float _visionRightCm = 150.0f;
    volatile uint32_t _lastVisionUpdateMs = 0;
    volatile uint32_t _lastHeartbeatMs = 0;
    uint32_t _lastTelemetryMs = 0;

    char _staSsid[33] = {0};
    char _staPass[65] = {0};
    bool _pendingWifiConnect = false;
    uint32_t _wifiConnectStartMs = 0;

    void connectSavedWifi();

    void onWsEvent(AsyncWebSocket* server,
                   AsyncWebSocketClient* client,
                   AwsEventType type,
                   void* arg,
                   uint8_t* data,
                   size_t len);
    void handleMessage(uint8_t* data, size_t len);
    void broadcastTelemetry(const TelemetrySnapshot& t);
};
