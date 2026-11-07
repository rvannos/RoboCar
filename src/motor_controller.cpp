#include "motor_controller.h"

void MotorController::begin() {
    pinMode(PIN_IN1, OUTPUT);
    pinMode(PIN_IN2, OUTPUT);
    pinMode(PIN_IN3, OUTPUT);
    pinMode(PIN_IN4, OUTPUT);

    digitalWrite(PIN_IN1, LOW);
    digitalWrite(PIN_IN2, LOW);
    digitalWrite(PIN_IN3, LOW);
    digitalWrite(PIN_IN4, LOW);

#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
    // ESP32 Arduino Core v3.x LEDC API
    ledcAttach(PIN_ENA, PWM_FREQ_HZ, PWM_RESOLUTION_BITS);
    ledcAttach(PIN_ENB, PWM_FREQ_HZ, PWM_RESOLUTION_BITS);
#else
    // Fallback for ESP32 Arduino Core v2.x
    ledcSetup(LEDC_CHANNEL_ENA, PWM_FREQ_HZ, PWM_RESOLUTION_BITS);
    ledcAttachPin(PIN_ENA, LEDC_CHANNEL_ENA);
    ledcSetup(LEDC_CHANNEL_ENB, PWM_FREQ_HZ, PWM_RESOLUTION_BITS);
    ledcAttachPin(PIN_ENB, LEDC_CHANNEL_ENB);
#endif

    stop_all();
}

void MotorController::restoreDrivePwmConfig() {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
    ledcDetach(PIN_ENA);
    ledcAttach(PIN_ENA, PWM_FREQ_HZ, PWM_RESOLUTION_BITS);
#else
    ledcSetup(LEDC_CHANNEL_ENA, PWM_FREQ_HZ, PWM_RESOLUTION_BITS);
#endif
    writePwmA(0);
}

void MotorController::writePwmA(uint8_t duty) {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
    ledcWrite(PIN_ENA, duty);
#else
    ledcWrite(LEDC_CHANNEL_ENA, duty);
#endif
}

void MotorController::writePwmB(uint8_t duty) {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
    ledcWrite(PIN_ENB, duty);
#else
    ledcWrite(LEDC_CHANNEL_ENB, duty);
#endif
}

void MotorController::forward(uint8_t speed) {
    digitalWrite(PIN_IN1, HIGH);
    digitalWrite(PIN_IN2, LOW);
    writePwmA(speed);
    _currentDriveSpeed = speed;
    _driveDir = (speed > 0) ? 1 : 0;
}

void MotorController::reverse(uint8_t speed) {
    digitalWrite(PIN_IN1, LOW);
    digitalWrite(PIN_IN2, HIGH);
    writePwmA(speed);
    _currentDriveSpeed = speed;
    _driveDir = (speed > 0) ? -1 : 0;
}

void MotorController::stop_drive() {
    digitalWrite(PIN_IN1, LOW);
    digitalWrite(PIN_IN2, LOW);
    writePwmA(0);
    _currentDriveSpeed = 0;
    _driveDir = 0;
}

void MotorController::steer_left(uint8_t pwm) {
    digitalWrite(PIN_IN3, HIGH);
    digitalWrite(PIN_IN4, LOW);
    writePwmB(pwm);
    _steerDir = (pwm > 0) ? -1 : 0;
}

void MotorController::steer_right(uint8_t pwm) {
    digitalWrite(PIN_IN3, LOW);
    digitalWrite(PIN_IN4, HIGH);
    writePwmB(pwm);
    _steerDir = (pwm > 0) ? 1 : 0;
}

void MotorController::steer_center() {
    // Cutting power allows the front steering rack spring to return to center
    digitalWrite(PIN_IN3, LOW);
    digitalWrite(PIN_IN4, LOW);
    writePwmB(0);
    _steerDir = 0;
}

void MotorController::stop_all() {
    stop_drive();
    steer_center();
}

void MotorController::playTone(uint32_t freqHz, uint32_t durationMs) {
    stop_all();
    if (freqHz == 0 || durationMs == 0) {
        return;
    }

    // Energize H-bridge direction while pulsing ENA at audio frequency with sub-rolling duty (~55/255)
    digitalWrite(PIN_IN1, HIGH);
    digitalWrite(PIN_IN2, LOW);

#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
    ledcChangeFrequency(PIN_ENA, freqHz, PWM_RESOLUTION_BITS);
    ledcWrite(PIN_ENA, 55);
#else
    ledcSetup(LEDC_CHANNEL_ENA, freqHz, PWM_RESOLUTION_BITS);
    ledcWrite(LEDC_CHANNEL_ENA, 55);
#endif

    delay(durationMs);

    digitalWrite(PIN_IN1, LOW);
    digitalWrite(PIN_IN2, LOW);
    restoreDrivePwmConfig();
}

void MotorController::playBootChime() {
    // Rising 3-note ESC-style boot chime (C6 -> E6 -> G6)
    playTone(1047, 90);
    delay(30);
    playTone(1319, 90);
    delay(30);
    playTone(1568, 140);
}

void MotorController::playRoamLaunchChime() {
    // Two alert pips followed by high launch tone (A5 -> A5 -> A6)
    playTone(880, 100);
    delay(60);
    playTone(880, 100);
    delay(60);
    playTone(1760, 220);
}
