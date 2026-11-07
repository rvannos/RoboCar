#pragma once

#include <Arduino.h>

class SonarDriver {
public:
    // Front HC-SR04P Pin Mapping (ESP32-S3)
    static constexpr uint8_t PIN_FRONT_TRIG = 12; // Front 10us trigger output
    static constexpr uint8_t PIN_FRONT_ECHO = 13; // Front echo input (3.3V native)

    // Rear HC-SR04P Pin Mapping (ESP32-S3)
    static constexpr uint8_t PIN_REAR_TRIG = 10;  // Rear 10us trigger output
    static constexpr uint8_t PIN_REAR_ECHO = 11;  // Rear echo input (3.3V native)

    // Timing & Filtering Constants
    static constexpr uint32_t PING_STEP_INTERVAL_MS = 35;
    static constexpr float MAX_DISTANCE_CM = 400.0f;
    static constexpr float EMERGENCY_MIN_CM = 7.0f;
    static constexpr float REAR_STOP_THRESHOLD_CM = 12.0f;
    static constexpr uint8_t CONSECUTIVE_FAULT_LIMIT = 4;

    SonarDriver() = default;

    void begin();

    // Non-blocking alternating ping loop; call every loop() iteration
    bool update();

    // Median-filtered distances in centimeters across 3 consecutive pings
    float getFrontDistanceCm() const { return _front.filteredCm; }
    float getRearDistanceCm() const { return _rear.filteredCm; }

    // Backwards-compatible alias for front distance
    float getDistanceCm() const { return _front.filteredCm; }

    // Watchdog & proximity hazard checks (require 3 consecutive <10cm readings to reject EMI spikes)
    bool isFrontEmergencyClose() const { return _front.consecutiveCloseCount >= 3; }
    bool isFrontEchoTimeoutFault() const { return false; } // Open-room timeouts (>4m) are treated as clear space
    bool isEmergencyHazard() const { return isFrontEmergencyClose(); }

    // True when rear sensor is connected and detects an obstacle within 15cm
    bool isRearObstacleClose() const {
        return _rear.hasValidReading && (_rear.filteredCm < REAR_STOP_THRESHOLD_CM);
    }

private:
    struct SensorChannel {
        uint8_t trigPin;
        uint8_t echoPin;
        float samples[3] = {MAX_DISTANCE_CM, MAX_DISTANCE_CM, MAX_DISTANCE_CM};
        uint8_t sampleIndex = 0;
        float filteredCm = MAX_DISTANCE_CM;
        float lastRawCm = MAX_DISTANCE_CM;
        uint8_t consecutiveCloseCount = 0;
        uint8_t consecutiveTimeoutCount = 0;
        bool hasValidReading = false;
    };

    SensorChannel _front{PIN_FRONT_TRIG, PIN_FRONT_ECHO};
    SensorChannel _rear{PIN_REAR_TRIG, PIN_REAR_ECHO};

    uint32_t _lastStepMs = 0;
    bool _activeIsFront = true;
    bool _awaitingEcho = false;

    static void IRAM_ATTR echoFrontIsr();
    static void IRAM_ATTR echoRearIsr();

    static volatile uint32_t _frontStartUs;
    static volatile uint32_t _frontEndUs;
    static volatile bool _frontCaptured;
    static volatile bool _frontSawRising;

    static volatile uint32_t _rearStartUs;
    static volatile uint32_t _rearEndUs;
    static volatile bool _rearCaptured;
    static volatile bool _rearSawRising;

    void triggerChannel(bool front);
    void processCompletedEcho(SensorChannel& ch, uint32_t durationUs);
    static float medianOfThree(float a, float b, float c);
};
