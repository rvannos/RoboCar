#include "sonar_driver.h"

volatile uint32_t SonarDriver::_frontStartUs = 0;
volatile uint32_t SonarDriver::_frontEndUs = 0;
volatile bool SonarDriver::_frontCaptured = false;
volatile bool SonarDriver::_frontSawRising = false;

volatile uint32_t SonarDriver::_rearStartUs = 0;
volatile uint32_t SonarDriver::_rearEndUs = 0;
volatile bool SonarDriver::_rearCaptured = false;
volatile bool SonarDriver::_rearSawRising = false;

void IRAM_ATTR SonarDriver::echoFrontIsr() {
    const uint32_t nowUs = micros();
    if (digitalRead(PIN_FRONT_ECHO) == HIGH) {
        _frontStartUs = nowUs;
        _frontSawRising = true;
    } else if (_frontSawRising) {
        const uint32_t dur = nowUs - _frontStartUs;
        if (dur >= 100) { // Reject <1.7cm floating-pin/EMI noise spikes
            _frontEndUs = nowUs;
            _frontCaptured = true;
        }
        _frontSawRising = false;
    }
}

void IRAM_ATTR SonarDriver::echoRearIsr() {
    const uint32_t nowUs = micros();
    if (digitalRead(PIN_REAR_ECHO) == HIGH) {
        _rearStartUs = nowUs;
        _rearSawRising = true;
    } else if (_rearSawRising) {
        const uint32_t dur = nowUs - _rearStartUs;
        if (dur >= 100) { // Reject <1.7cm floating-pin/EMI noise spikes
            _rearEndUs = nowUs;
            _rearCaptured = true;
        }
        _rearSawRising = false;
    }
}

void SonarDriver::begin() {
    pinMode(PIN_FRONT_TRIG, OUTPUT);
    digitalWrite(PIN_FRONT_TRIG, LOW);
    pinMode(PIN_FRONT_ECHO, INPUT_PULLDOWN);

    pinMode(PIN_REAR_TRIG, OUTPUT);
    digitalWrite(PIN_REAR_TRIG, LOW);
    pinMode(PIN_REAR_ECHO, INPUT_PULLDOWN);

    attachInterrupt(digitalPinToInterrupt(PIN_FRONT_ECHO), echoFrontIsr, CHANGE);
    attachInterrupt(digitalPinToInterrupt(PIN_REAR_ECHO), echoRearIsr, CHANGE);

    _lastStepMs = millis();
}

void SonarDriver::triggerChannel(bool front) {
    _awaitingEcho = true;
    if (front) {
        _frontCaptured = false;
        _frontSawRising = false;
        digitalWrite(PIN_FRONT_TRIG, LOW);
        delayMicroseconds(2);
        digitalWrite(PIN_FRONT_TRIG, HIGH);
        delayMicroseconds(10);
        digitalWrite(PIN_FRONT_TRIG, LOW);
    } else {
        _rearCaptured = false;
        _rearSawRising = false;
        digitalWrite(PIN_REAR_TRIG, LOW);
        delayMicroseconds(2);
        digitalWrite(PIN_REAR_TRIG, HIGH);
        delayMicroseconds(10);
        digitalWrite(PIN_REAR_TRIG, LOW);
    }
}

float SonarDriver::medianOfThree(float a, float b, float c) {
    if ((a <= b && b <= c) || (c <= b && b <= a)) return b;
    if ((b <= a && a <= c) || (c <= a && a <= b)) return a;
    return c;
}

void SonarDriver::processCompletedEcho(SensorChannel& ch, uint32_t durationUs) {
    float cm = static_cast<float>(durationUs) * 0.01715f;
    if (cm > MAX_DISTANCE_CM) {
        cm = MAX_DISTANCE_CM;
    }

    ch.hasValidReading = true;
    ch.lastRawCm = cm;
    ch.samples[ch.sampleIndex] = cm;
    ch.sampleIndex = (ch.sampleIndex + 1) % 3;
    const float medCm = medianOfThree(ch.samples[0], ch.samples[1], ch.samples[2]);
    // Pure 3-sample median + gentle low-pass smoothing completely rejects single-ping
    // ultrasonic floor reflections and brushed DC motor EMI spikes (no twitching!)
    ch.filteredCm = ch.filteredCm * 0.35f + medCm * 0.65f;

    ch.consecutiveTimeoutCount = 0;
    if (cm < EMERGENCY_MIN_CM) {
        if (ch.consecutiveCloseCount < 255) {
            ch.consecutiveCloseCount++;
        }
    } else {
        ch.consecutiveCloseCount = 0;
    }
}

bool SonarDriver::update() {
    const uint32_t nowMs = millis();
    bool newReading = false;

    // 1. Check if the currently active sensor finished capturing its echo pulse
    if (_awaitingEcho) {
        if (_activeIsFront && _frontCaptured) {
            _awaitingEcho = false;
            processCompletedEcho(_front, _frontEndUs - _frontStartUs);
            newReading = true;
        } else if (!_activeIsFront && _rearCaptured) {
            _awaitingEcho = false;
            processCompletedEcho(_rear, _rearEndUs - _rearStartUs);
            newReading = true;
        }
    }

    // 2. Wait until the 40ms alternating step interval completes before firing the other sensor
    if (nowMs - _lastStepMs < PING_STEP_INTERVAL_MS) {
        return newReading;
    }

    // 3. Handle timeout if the previous ping never returned within 40ms (open room > 4m)
    if (_awaitingEcho) {
        _awaitingEcho = false;
        SensorChannel& timedOutCh = _activeIsFront ? _front : _rear;
        if (timedOutCh.consecutiveTimeoutCount < 255) {
            timedOutCh.consecutiveTimeoutCount++;
        }
        // Only relax toward open space if the sensor wasn't already right against an obstacle
        if (timedOutCh.lastRawCm > 35.0f) {
            processCompletedEcho(timedOutCh, static_cast<uint32_t>(MAX_DISTANCE_CM / 0.01715f));
            newReading = true;
        }
    }

    // 4. Alternate between Front and Rear sensors to eliminate acoustic crosstalk
    _activeIsFront = !_activeIsFront;
    _lastStepMs = nowMs;
    triggerChannel(_activeIsFront);

    return newReading;
}
