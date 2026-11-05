#pragma once

#include <Arduino.h>
#include <WiFi.h>
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

class WebInterface {
public:
    // SoftAP Credentials & Static IP Configuration
    static constexpr const char* AP_SSID = "RC-Rover";
    static constexpr const char* AP_PASS = "rover1234"; // Minimum 8 chars for WPA2

    // 500ms Failsafe Watchdog Timeout for Manual Wi-Fi Control
    static constexpr uint32_t WATCHDOG_TIMEOUT_MS = 500;
    static constexpr uint32_t TELEMETRY_INTERVAL_MS = 100;

    WebInterface();

    void begin();
    void update(float frontDistanceCm, float rearDistanceCm, const char* stateLabel, bool watchdogTripped);

    RoverMode getMode() const { return _mode; }
    void activateAutonomousRoam() { _mode = RoverMode::AUTONOMOUS; }
    bool hasUserInteracted() const { return _userInteracted; }

    uint8_t getThrottleTrim() const { return _throttleTrim; }
    ManualCommand getManualCommand() const { return _manualCmd; }

    // Camera-assisted target steering (-1 = left, 0 = center, 1 = right) for Follow-Me mode
    int8_t getFollowTargetSteer() const;
    bool hasActiveFollowTarget() const;

    // Returns true if manual heartbeat has expired (> 500ms since last client packet)
    bool isManualWatchdogExpired() const;
    void clearManualCommand();

private:
    AsyncWebServer _server;
    AsyncWebSocket _ws;

    volatile RoverMode _mode = RoverMode::STANDBY;
    volatile bool _userInteracted = false;
    volatile uint8_t _throttleTrim = 170;
    ManualCommand _manualCmd;
    volatile int8_t _followTargetSteer = 0;
    volatile bool _followTargetLocked = false;
    volatile uint32_t _lastFollowUpdateMs = 0;
    volatile uint32_t _lastHeartbeatMs = 0;
    uint32_t _lastTelemetryMs = 0;

    void onWsEvent(AsyncWebSocket* server,
                   AsyncWebSocketClient* client,
                   AwsEventType type,
                   void* arg,
                   uint8_t* data,
                   size_t len);
    void handleMessage(uint8_t* data, size_t len);
    void broadcastTelemetry(float frontDistanceCm, float rearDistanceCm, const char* stateLabel, bool watchdogTripped);
};
