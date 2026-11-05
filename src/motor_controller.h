#pragma once

#include <Arduino.h>

class MotorController {
public:
    // L298N Dual H-Bridge Pin Mapping (ESP32-S3)
    static constexpr uint8_t PIN_IN1 = 4; // Drive motor direction forward
    static constexpr uint8_t PIN_IN2 = 5; // Drive motor direction reverse
    static constexpr uint8_t PIN_IN3 = 6; // Steering left command
    static constexpr uint8_t PIN_IN4 = 7; // Steering right command
    static constexpr uint8_t PIN_ENA = 8; // Rear motor PWM speed control
    static constexpr uint8_t PIN_ENB = 9; // Steering PWM power cap

    // PWM Configuration
    static constexpr uint32_t PWM_FREQ_HZ = 5000;
    static constexpr uint8_t PWM_RESOLUTION_BITS = 8; // 0-255 scale

    // Calibration Defaults (0-255 scale)
    // Toy 2-wire steering actuators run hot if held at 100% duty cycle against stops.
    // Capped in the 180-220 range (default: 200).
    static constexpr uint8_t DEFAULT_STEER_PWM = 200;
    static constexpr uint8_t DEFAULT_CRUISE_PWM = 170;
    static constexpr uint8_t DEFAULT_REVERSE_PWM = 190;

    MotorController() = default;

    void begin();

    // Drive motor routines
    void forward(uint8_t speed);
    void reverse(uint8_t speed);
    void stop_drive();

    // Front steering actuator routines (spring-centered)
    void steer_left(uint8_t pwm = DEFAULT_STEER_PWM);
    void steer_right(uint8_t pwm = DEFAULT_STEER_PWM);
    void steer_center();

    // Hard stop both drive and steering channels
    void stop_all();

    // Acoustic motor-winding chime routines (uses the DC motor coils like an ESC speaker!)
    void playTone(uint32_t freqHz, uint32_t durationMs);
    void playBootChime();
    void playRoamLaunchChime();

    // Telemetry getters
    uint8_t getDriveSpeed() const { return _currentDriveSpeed; }
    int8_t getDriveDirection() const { return _driveDir; } // 1 = fwd, -1 = rev, 0 = stopped
    int8_t getSteerDirection() const { return _steerDir; } // -1 = left, 1 = right, 0 = center

private:
    static constexpr uint8_t LEDC_CHANNEL_ENA = 0;
    static constexpr uint8_t LEDC_CHANNEL_ENB = 1;

    uint8_t _currentDriveSpeed = 0;
    int8_t _driveDir = 0;
    int8_t _steerDir = 0;

    void writePwmA(uint8_t duty);
    void writePwmB(uint8_t duty);
    void restoreDrivePwmConfig();
};
