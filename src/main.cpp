#include <Arduino.h>
#include "motor_controller.h"
#include "sonar_driver.h"
#include "web_interface.h"

namespace {

// 30-Second Boot Standby Window before Auto-Roam
constexpr uint32_t BOOT_STANDBY_TIMEOUT_MS = 30000;

// Steer-First Autonomous Thresholds & Timings
constexpr float STEER_AVOID_THRESHOLD_CM = 65.0f; // Begin steering around obstacle while driving forward
constexpr float BLOCKED_REVERSE_CM = 28.0f;       // Only reverse when blocked closer than 28cm
constexpr uint32_t REVERSE_DURATION_MS = 350;
constexpr uint32_t EVADE_RECOVERY_MS = 320;
constexpr uint8_t REVERSE_SPEED_PWM = 190;

// Follow-Me Standoff Distance Window
constexpr float FOLLOW_MIN_HOLD_CM = 30.0f;
constexpr float FOLLOW_MAX_HOLD_CM = 50.0f;
constexpr float FOLLOW_BACKUP_CM = 24.0f;
constexpr float FOLLOW_MAX_RANGE_CM = 160.0f;

// Onboard WS2812 RGB Status LED (GPIO 48, soldered by default on YD-ESP32-S3)
constexpr uint8_t PIN_RGB_LED = 48;

enum class NavigationState : uint8_t {
    STATE_CRUISING,
    STATE_STEER_AROUND,
    STATE_BLOCKED_REVERSE,
    STATE_POST_REVERSE_TURN,
    STATE_EMERGENCY_STOP
};

MotorController motors;
SonarDriver sonar;
WebInterface webUi;

NavigationState navState = NavigationState::STATE_CRUISING;
RoverMode lastMode = RoverMode::STANDBY;
uint32_t bootStartMs = 0;
uint32_t stateEnteredMs = 0;
float steerEntryDistanceCm = 400.0f;
bool steerLeftActive = true;
const char* followSubStateLabel = "FOLLOW: STANDBY";
char standbyLabelBuf[32] = "STANDBY (30s)";

void updateStatusLed(RoverMode mode, NavigationState state, bool watchdogTripped) {
    if (mode == RoverMode::STANDBY) {
        // Gentle cyan pulse while waiting in 30-second boot standby
        const uint8_t pulse = ((millis() / 300) & 1U) ? 35 : 8;
        neopixelWrite(PIN_RGB_LED, 0, pulse, pulse);
        return;
    }
    if (watchdogTripped || state == NavigationState::STATE_EMERGENCY_STOP) {
        neopixelWrite(PIN_RGB_LED, 40, 0, 0); // Red: Emergency / Watchdog Stop
        return;
    }
    if (mode == RoverMode::MANUAL) {
        neopixelWrite(PIN_RGB_LED, 0, 15, 40); // Blue: Manual D-Pad Mode
        return;
    }
    if (mode == RoverMode::FOLLOW) {
        neopixelWrite(PIN_RGB_LED, 30, 0, 35); // Purple: Follow-Me Mode
        return;
    }
    switch (state) {
        case NavigationState::STATE_CRUISING:
            neopixelWrite(PIN_RGB_LED, 0, 35, 0);  // Green: Clear Cruise
            break;
        case NavigationState::STATE_STEER_AROUND:
            neopixelWrite(PIN_RGB_LED, 40, 22, 0); // Amber: Steering Around Obstacle
            break;
        case NavigationState::STATE_BLOCKED_REVERSE:
        case NavigationState::STATE_POST_REVERSE_TURN:
            neopixelWrite(PIN_RGB_LED, 40, 5, 0);  // Orange-Red: Reversing / Escaping
            break;
        default:
            break;
    }
}

const char* getStateLabel(RoverMode mode, NavigationState state, bool manualWatchdogTripped) {
    if (mode == RoverMode::STANDBY) {
        return standbyLabelBuf;
    }
    if (mode == RoverMode::MANUAL) {
        return manualWatchdogTripped ? "MANUAL (WD STOP)" : "MANUAL OVERRIDE";
    }
    if (mode == RoverMode::FOLLOW) {
        return followSubStateLabel;
    }
    switch (state) {
        case NavigationState::STATE_CRUISING:
            return "AUTO: CRUISING";
        case NavigationState::STATE_STEER_AROUND:
            return steerLeftActive ? "AUTO: STEER LEFT" : "AUTO: STEER RIGHT";
        case NavigationState::STATE_BLOCKED_REVERSE:
            return "AUTO: NO PATH (REV)";
        case NavigationState::STATE_POST_REVERSE_TURN:
            return steerLeftActive ? "AUTO: ESCAPE LEFT" : "AUTO: ESCAPE RIGHT";
        case NavigationState::STATE_EMERGENCY_STOP:
            return "EMERGENCY STOP";
        default:
            return "UNKNOWN";
    }
}

void applySteerDirection(bool left) {
    if (left) {
        motors.steer_left(MotorController::DEFAULT_STEER_PWM);
    } else {
        motors.steer_right(MotorController::DEFAULT_STEER_PWM);
    }
}

void runAutonomousStateMachine(uint32_t nowMs, float frontDistCm, uint8_t cruiseSpeedPwm) {
    const uint8_t turnSpeedPwm = (cruiseSpeedPwm > 110) ? static_cast<uint8_t>(cruiseSpeedPwm - 15) : cruiseSpeedPwm;

    switch (navState) {
        case NavigationState::STATE_CRUISING: {
            if (sonar.isEmergencyHazard()) {
                motors.stop_all();
                navState = NavigationState::STATE_EMERGENCY_STOP;
                stateEnteredMs = nowMs;
                return;
            }

            // 1. No path forward (< 28cm) -> Must back up to create turning clearance
            if (frontDistCm < BLOCKED_REVERSE_CM) {
                motors.stop_all();
                steerLeftActive = (esp_random() & 1U) != 0;
                if (sonar.isRearObstacleClose()) {
                    applySteerDirection(steerLeftActive);
                    motors.forward(turnSpeedPwm);
                    navState = NavigationState::STATE_POST_REVERSE_TURN;
                } else {
                    applySteerDirection(!steerLeftActive);
                    motors.reverse(REVERSE_SPEED_PWM);
                    navState = NavigationState::STATE_BLOCKED_REVERSE;
                }
                stateEnteredMs = nowMs;
                return;
            }

            // 2. Obstacle ahead in mid-range (28cm - 65cm) -> Steer around on the fly without stopping!
            if (frontDistCm < STEER_AVOID_THRESHOLD_CM) {
                steerLeftActive = (esp_random() & 1U) != 0;
                steerEntryDistanceCm = frontDistCm;
                applySteerDirection(steerLeftActive);
                motors.forward(turnSpeedPwm);
                navState = NavigationState::STATE_STEER_AROUND;
                stateEnteredMs = nowMs;
                return;
            }

            // 3. Open path (>= 65cm) -> Cruise straight ahead
            motors.steer_center();
            motors.forward(cruiseSpeedPwm);
            break;
        }

        case NavigationState::STATE_STEER_AROUND: {
            if (sonar.isEmergencyHazard()) {
                motors.stop_all();
                navState = NavigationState::STATE_EMERGENCY_STOP;
                stateEnteredMs = nowMs;
                return;
            }

            // Path opened up (>= 65cm) -> Re-center steering and resume straight cruise!
            if (frontDistCm >= STEER_AVOID_THRESHOLD_CM) {
                motors.steer_center();
                motors.forward(cruiseSpeedPwm);
                navState = NavigationState::STATE_CRUISING;
                stateEnteredMs = nowMs;
                return;
            }

            // Corner closed in completely (< 28cm) -> No forward path left, reverse out
            if (frontDistCm < BLOCKED_REVERSE_CM) {
                motors.stop_all();
                if (sonar.isRearObstacleClose()) {
                    steerLeftActive = !steerLeftActive;
                    applySteerDirection(steerLeftActive);
                    motors.forward(turnSpeedPwm);
                    navState = NavigationState::STATE_POST_REVERSE_TURN;
                } else {
                    applySteerDirection(!steerLeftActive);
                    motors.reverse(REVERSE_SPEED_PWM);
                    navState = NavigationState::STATE_BLOCKED_REVERSE;
                }
                stateEnteredMs = nowMs;
                return;
            }

            // If steering in the chosen direction for 350ms+ is closing the gap, flip steering side on the fly
            if ((nowMs - stateEnteredMs > 350) && (frontDistCm + 6.0f < steerEntryDistanceCm)) {
                steerLeftActive = !steerLeftActive;
                steerEntryDistanceCm = frontDistCm;
                stateEnteredMs = nowMs;
            }

            applySteerDirection(steerLeftActive);
            motors.forward(turnSpeedPwm);
            break;
        }

        case NavigationState::STATE_BLOCKED_REVERSE: {
            if (sonar.isRearObstacleClose() || (nowMs - stateEnteredMs >= REVERSE_DURATION_MS)) {
                motors.stop_drive();
                applySteerDirection(steerLeftActive);
                motors.forward(turnSpeedPwm);
                navState = NavigationState::STATE_POST_REVERSE_TURN;
                stateEnteredMs = nowMs;
            }
            break;
        }

        case NavigationState::STATE_POST_REVERSE_TURN: {
            if (sonar.isFrontEmergencyClose()) {
                motors.stop_all();
                navState = NavigationState::STATE_EMERGENCY_STOP;
                stateEnteredMs = nowMs;
                return;
            }

            if (nowMs - stateEnteredMs >= EVADE_RECOVERY_MS) {
                motors.steer_center();
                navState = NavigationState::STATE_CRUISING;
                stateEnteredMs = nowMs;
            }
            break;
        }

        case NavigationState::STATE_EMERGENCY_STOP: {
            motors.stop_all();
            if (!sonar.isEmergencyHazard() && (nowMs - stateEnteredMs >= 300)) {
                if (frontDistCm < BLOCKED_REVERSE_CM && !sonar.isRearObstacleClose()) {
                    applySteerDirection(!steerLeftActive);
                    motors.reverse(REVERSE_SPEED_PWM);
                    navState = NavigationState::STATE_BLOCKED_REVERSE;
                } else {
                    navState = NavigationState::STATE_CRUISING;
                }
                stateEnteredMs = nowMs;
            }
            break;
        }
    }
}

void runFollowMeMode(float frontDistCm, uint8_t cruiseSpeedPwm) {
    const int8_t targetSteer = webUi.getFollowTargetSteer();
    if (targetSteer < 0) {
        motors.steer_left(MotorController::DEFAULT_STEER_PWM);
    } else if (targetSteer > 0) {
        motors.steer_right(MotorController::DEFAULT_STEER_PWM);
    } else {
        motors.steer_center();
    }

    if (frontDistCm < FOLLOW_BACKUP_CM) {
        if (!sonar.isRearObstacleClose()) {
            motors.reverse(cruiseSpeedPwm);
            followSubStateLabel = "FOLLOW: BACKING OFF";
        } else {
            motors.stop_drive();
            followSubStateLabel = "FOLLOW: REAR BLOCKED";
        }
    } else if (frontDistCm <= FOLLOW_MAX_HOLD_CM) {
        motors.stop_drive();
        followSubStateLabel = "FOLLOW: LOCKED HOLD";
    } else if (frontDistCm <= FOLLOW_MAX_RANGE_CM || webUi.hasActiveFollowTarget()) {
        motors.forward(cruiseSpeedPwm);
        followSubStateLabel = "FOLLOW: TRACKING";
    } else {
        motors.stop_all();
        followSubStateLabel = "FOLLOW: WAITING TARGET";
    }
}

void runManualControl(uint8_t throttlePwm, bool& watchdogTrippedOut) {
    if (webUi.isManualWatchdogExpired()) {
        motors.stop_all();
        webUi.clearManualCommand();
        watchdogTrippedOut = true;
        return;
    }

    watchdogTrippedOut = false;
    const ManualCommand cmd = webUi.getManualCommand();

    if (cmd.drive > 0) {
        motors.forward(throttlePwm);
    } else if (cmd.drive < 0) {
        motors.reverse(throttlePwm);
    } else {
        motors.stop_drive();
    }

    if (cmd.steer < 0) {
        motors.steer_left(MotorController::DEFAULT_STEER_PWM);
    } else if (cmd.steer > 0) {
        motors.steer_right(MotorController::DEFAULT_STEER_PWM);
    } else {
        motors.steer_center();
    }
}

} // namespace

void setup() {
    Serial.begin(115200);

    motors.begin();
    sonar.begin();
    webUi.begin();

    // Play 3-note rising ESC-style boot chime through the rear drive motor windings
    motors.playBootChime();

    bootStartMs = millis();
    stateEnteredMs = bootStartMs;

    Serial.println("=========================================");
    Serial.println(" Autonomous RC Rover S3 Initialized");
    Serial.printf(" SoftAP SSID: %s | IP: 192.168.4.1\n", WebInterface::AP_SSID);
    Serial.println(" Waiting 30s in STANDBY before Auto-Roam");
    Serial.println("=========================================");
}

void loop() {
    const uint32_t nowMs = millis();

    // 1. Non-blocking alternating Front + Rear ultrasonic sensor update (3-sample median filter)
    sonar.update();
    const float frontDistCm = sonar.getFrontDistanceCm();
    const float rearDistCm = sonar.getRearDistanceCm();

    // 2. Check 30-second Boot Standby timer if user hasn't interacted via the web UI yet
    RoverMode currentMode = webUi.getMode();
    if (currentMode == RoverMode::STANDBY) {
        motors.stop_all();
        const uint32_t elapsedMs = nowMs - bootStartMs;
        if (elapsedMs >= BOOT_STANDBY_TIMEOUT_MS) {
            // 30s elapsed with no web input -> Play launch chime and enter Autonomous Roam!
            motors.playRoamLaunchChime();
            webUi.activateAutonomousRoam();
            currentMode = RoverMode::AUTONOMOUS;
        } else {
            const uint32_t remSec = (BOOT_STANDBY_TIMEOUT_MS - elapsedMs + 999) / 1000;
            snprintf(standbyLabelBuf, sizeof(standbyLabelBuf), "STANDBY (%us)", static_cast<unsigned>(remSec));
        }
    }

    // 3. Mode transition cleanup & audible mode-change pip
    if (currentMode != lastMode) {
        motors.stop_all();
        if (lastMode == RoverMode::STANDBY && webUi.hasUserInteracted()) {
            motors.playTone(1400, 75); // Audible confirmation pip when web user takes over
        }
        navState = NavigationState::STATE_CRUISING;
        stateEnteredMs = millis();
        lastMode = currentMode;
    }

    // 4. Execute active control loop (Standby, Autonomous Steer-Around, Follow-Me, or Manual D-Pad)
    const uint8_t throttlePwm = webUi.getThrottleTrim();
    bool manualWatchdogTripped = false;

    if (currentMode == RoverMode::AUTONOMOUS) {
        runAutonomousStateMachine(nowMs, frontDistCm, throttlePwm);
    } else if (currentMode == RoverMode::FOLLOW) {
        runFollowMeMode(frontDistCm, throttlePwm);
    } else if (currentMode == RoverMode::MANUAL) {
        runManualControl(throttlePwm, manualWatchdogTripped);
    }

    // 5. Update onboard WS2812 RGB status LED (GPIO 48) & broadcast live WebSocket telemetry
    const bool anyWatchdogTripped =
        (currentMode == RoverMode::MANUAL) ? manualWatchdogTripped : (navState == NavigationState::STATE_EMERGENCY_STOP);
    updateStatusLed(currentMode, navState, anyWatchdogTripped);
    webUi.update(frontDistCm, rearDistCm, getStateLabel(currentMode, navState, manualWatchdogTripped), anyWatchdogTripped);
}
