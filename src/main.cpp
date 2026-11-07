#include <Arduino.h>
#include <ArduinoOTA.h>
#include <math.h>
#include "motor_controller.h"
#include "sonar_driver.h"
#include "web_interface.h"

namespace {

// 30-Second Boot Standby Window before Auto-Roam
constexpr uint32_t BOOT_STANDBY_TIMEOUT_MS = 30000;

// Steer-First Autonomous Thresholds & Timings (with Hysteresis to prevent wheel twitching)
constexpr float STEER_AVOID_THRESHOLD_CM = 38.0f; // Get closer (38cm / ~15 in) before steering around obstacles
constexpr float STEER_CLEAR_RESUME_CM = 46.0f;    // Resume straight cruise once >= 46cm (8cm hysteresis)
constexpr float BLOCKED_REVERSE_CM = 20.0f;       // Get within 20cm (~8 in) before triggering reverse
constexpr uint32_t REVERSE_DURATION_MS = 1500;    // Back up for a full 1.5 seconds to create plenty of turning room
constexpr uint32_t EVADE_RECOVERY_MS = 850;       // Committed 850ms forward turn after reversing
constexpr uint8_t REVERSE_SPEED_PWM = 165;

// Follow-Me Standoff Distance Window
constexpr float FOLLOW_MIN_HOLD_CM = 30.0f;
constexpr float FOLLOW_MAX_HOLD_CM = 50.0f;
constexpr float FOLLOW_BACKUP_CM = 24.0f;
constexpr float FOLLOW_MAX_RANGE_CM = 160.0f;

// Onboard WS2812 RGB Status LED (GPIO 48, soldered by default on YD-ESP32-S3)
constexpr uint8_t STATUS_RGB_PIN = 48;

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

// 3-Sector Spatial Clearance Memory (Left, Center, Right) derived from 12-Sector 360° World Map
float sectorLeftCm = 150.0f;
float sectorCenterCm = 150.0f;
float sectorRightCm = 150.0f;

// 12-Sector (30° per bin) 360° Persistent World Occupancy & Exploration Map
constexpr uint8_t WORLD_BINS = 12;
float worldClearanceCm[WORLD_BINS] = {
    180.0f, 180.0f, 180.0f, 180.0f, 180.0f, 180.0f,
    180.0f, 180.0f, 180.0f, 180.0f, 180.0f, 180.0f
};
float worldVisitWeight[WORLD_BINS] = {0};
float roverHeadingDeg = 0.0f;
uint32_t lastPoseStepMs = 0;
int8_t currentGoalSteer = 0;

// Sonar Odometry Stall Detector (detects if car is stuck against a low obstacle below the sonar beam)
uint32_t stallWindowStartMs = 0;
float stallRefFrontCm = 400.0f;
float stallRefRearCm = 400.0f;

const char* followSubStateLabel = "FOLLOW: STANDBY";
char standbyLabelBuf[32] = "STANDBY (30s)";

uint8_t angleToBin(float deg) {
    while (deg < 0.0f) deg += 360.0f;
    while (deg >= 360.0f) deg -= 360.0f;
    return static_cast<uint8_t>(static_cast<int>((deg + 15.0f) / 30.0f) % WORLD_BINS);
}

void stepPoseOdometry(uint32_t nowMs, int8_t driveDir, int8_t steerDir) {
    if (lastPoseStepMs == 0) {
        lastPoseStepMs = nowMs;
        return;
    }
    const float dtSec = static_cast<float>(nowMs - lastPoseStepMs) * 0.001f;
    lastPoseStepMs = nowMs;
    if (dtSec <= 0.0f || dtSec > 0.5f) {
        return;
    }

    // Integrate steering rotation into world compass heading (0-360 deg)
    if (driveDir != 0 && steerDir != 0) {
        // Driving forward + right (+1 * +1) rotates clockwise (+deg);
        // Reversing + left (-1 * -1) also swings the nose clockwise (+deg)
        const float turnRateDegPerSec = 42.0f;
        roverHeadingDeg += static_cast<float>(driveDir * steerDir) * turnRateDegPerSec * dtSec;
        while (roverHeadingDeg < 0.0f) roverHeadingDeg += 360.0f;
        while (roverHeadingDeg >= 360.0f) roverHeadingDeg -= 360.0f;
    }

    // Slowly decay visit penalties so previously visited areas can be re-explored later
    for (uint8_t i = 0; i < WORLD_BINS; i++) {
        worldVisitWeight[i] *= (1.0f - 0.04f * dtSec);
        // Gentle relaxation of very old un-observed sectors toward 180cm
        worldClearanceCm[i] = worldClearanceCm[i] * (1.0f - 0.015f * dtSec) + 180.0f * (0.015f * dtSec);
    }
}

void updateSectorMemory(float frontDistCm, float rearDistCm, int8_t driveDir, int8_t steerDir) {
    // 1. Project Front Sonar beam into the 360° World Map (accounting for wheel steer angle)
    const float frontBeamDeg = roverHeadingDeg + static_cast<float>(steerDir) * 28.0f;
    const uint8_t frontBin = angleToBin(frontBeamDeg);
    worldClearanceCm[frontBin] = worldClearanceCm[frontBin] * 0.3f + frontDistCm * 0.7f;

    if (driveDir > 0) {
        worldVisitWeight[frontBin] = fminf(60.0f, worldVisitWeight[frontBin] + 0.8f);
    }

    // 2. Project Rear Sonar beam (+180°) into the 360° World Map if rear sensor has valid echoes
    if (rearDistCm < 395.0f) {
        const uint8_t rearBin = angleToBin(roverHeadingDeg + 180.0f);
        worldClearanceCm[rearBin] = worldClearanceCm[rearBin] * 0.35f + rearDistCm * 0.65f;
    }

    // 3. Extract ego-centric Left (-30°), Center (0°), and Right (+30°) clearances from the World Map
    const uint8_t leftBin1 = angleToBin(roverHeadingDeg - 30.0f);
    const uint8_t leftBin2 = angleToBin(roverHeadingDeg - 60.0f);
    const uint8_t centerBin = angleToBin(roverHeadingDeg);
    const uint8_t rightBin1 = angleToBin(roverHeadingDeg + 30.0f);
    const uint8_t rightBin2 = angleToBin(roverHeadingDeg + 60.0f);

    sectorCenterCm = worldClearanceCm[centerBin];
    sectorLeftCm = worldClearanceCm[leftBin1] * 0.7f + worldClearanceCm[leftBin2] * 0.3f;
    sectorRightCm = worldClearanceCm[rightBin1] * 0.7f + worldClearanceCm[rightBin2] * 0.3f;

    // 4. Blend Camera + Sonar optical horizon clearances if active
    if (webUi.hasVisionSectorFusion()) {
        sectorLeftCm = sectorLeftCm * 0.55f + webUi.getVisionLeftCm() * 0.45f;
        sectorRightCm = sectorRightCm * 0.55f + webUi.getVisionRightCm() * 0.45f;
    }

    // 5. Compute Frontier Exploration Score (Clearance minus Visit Penalty)
    const float leftScore = sectorLeftCm - worldVisitWeight[leftBin1];
    const float centerScore = sectorCenterCm - worldVisitWeight[centerBin];
    const float rightScore = sectorRightCm - worldVisitWeight[rightBin1];

    if (centerScore >= leftScore - 10.0f && centerScore >= rightScore - 10.0f && sectorCenterCm >= STEER_AVOID_THRESHOLD_CM) {
        currentGoalSteer = 0;
    } else if (leftScore > rightScore) {
        currentGoalSteer = -1;
    } else {
        currentGoalSteer = 1;
    }
}

bool pickBestSteerLeft() {
    // Score Left (-30° / -60°) vs Right (+30° / +60°) using the 360° World Map + Frontier Visit memory
    const uint8_t leftBin = angleToBin(roverHeadingDeg - 30.0f);
    const uint8_t rightBin = angleToBin(roverHeadingDeg + 30.0f);
    const float leftScore = sectorLeftCm - worldVisitWeight[leftBin];
    const float rightScore = sectorRightCm - worldVisitWeight[rightBin];

    if (fabsf(leftScore - rightScore) > 6.0f) {
        return leftScore > rightScore;
    }
    return (esp_random() & 1U) != 0;
}

void updateStatusLed(RoverMode mode, NavigationState state, bool watchdogTripped) {
    if (mode == RoverMode::STANDBY) {
        const uint8_t pulse = ((millis() / 300) & 1U) ? 35 : 8;
        rgbLedWrite(STATUS_RGB_PIN, 0, pulse, pulse);
        return;
    }
    if (watchdogTripped || state == NavigationState::STATE_EMERGENCY_STOP) {
        rgbLedWrite(STATUS_RGB_PIN, 40, 0, 0); // Red: Emergency / Watchdog Stop
        return;
    }
    if (mode == RoverMode::MANUAL) {
        rgbLedWrite(STATUS_RGB_PIN, 0, 15, 40); // Blue: Manual D-Pad Mode
        return;
    }
    if (mode == RoverMode::FOLLOW) {
        rgbLedWrite(STATUS_RGB_PIN, 30, 0, 35); // Purple: Follow-Me Mode
        return;
    }
    switch (state) {
        case NavigationState::STATE_CRUISING:
            rgbLedWrite(STATUS_RGB_PIN, 0, 35, 0);  // Green: Clear Cruise
            break;
        case NavigationState::STATE_STEER_AROUND:
            rgbLedWrite(STATUS_RGB_PIN, 40, 22, 0); // Amber: Steering Around Obstacle
            break;
        case NavigationState::STATE_BLOCKED_REVERSE:
        case NavigationState::STATE_POST_REVERSE_TURN:
            rgbLedWrite(STATUS_RGB_PIN, 40, 5, 0);  // Orange-Red: Reversing / Escaping
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
    // Automatically feather throttle down when approaching obstacles (< 110cm) to prevent overshooting
    const uint8_t approachSpeedPwm = (cruiseSpeedPwm > 115) ? static_cast<uint8_t>(cruiseSpeedPwm - 20) : cruiseSpeedPwm;
    const uint8_t turnSpeedPwm = (cruiseSpeedPwm > 120) ? static_cast<uint8_t>(cruiseSpeedPwm - 15) : cruiseSpeedPwm;

    switch (navState) {
        case NavigationState::STATE_CRUISING: {
            if (sonar.isEmergencyHazard()) {
                motors.stop_all();
                navState = NavigationState::STATE_EMERGENCY_STOP;
                stateEnteredMs = nowMs;
                return;
            }

            // 1. Obstacle within 42cm -> Brake and back up early before momentum carries car into wall
            if (frontDistCm < BLOCKED_REVERSE_CM) {
                motors.stop_all();
                steerLeftActive = pickBestSteerLeft();
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

            // 2. Obstacle ahead in mid-range (20cm - 38cm) ->
            //    Commit to a smooth turn toward the highest-scoring open frontier sector!
            if (frontDistCm < STEER_AVOID_THRESHOLD_CM) {
                steerLeftActive = pickBestSteerLeft();
                steerEntryDistanceCm = frontDistCm;
                applySteerDirection(steerLeftActive);
                motors.forward(turnSpeedPwm);
                navState = NavigationState::STATE_STEER_AROUND;
                stateEnteredMs = nowMs;
                stallWindowStartMs = nowMs;
                return;
            }

            // 3. Sonar Odometry Stall Detector: if driving forward for 1.5s and neither Front nor Rear
            //    distance has changed by > 3.5cm, the car is snagged on a low obstacle below the beam!
            if (stallWindowStartMs == 0) {
                stallWindowStartMs = nowMs;
                stallRefFrontCm = frontDistCm;
                stallRefRearCm = sonar.getRearDistanceCm();
            } else if (nowMs - stallWindowStartMs >= 1500) {
                const float dFront = fabsf(frontDistCm - stallRefFrontCm);
                const float dRear = fabsf(sonar.getRearDistanceCm() - stallRefRearCm);
                stallWindowStartMs = nowMs;
                stallRefFrontCm = frontDistCm;
                stallRefRearCm = sonar.getRearDistanceCm();

                if (dFront < 3.5f && dRear < 3.5f && frontDistCm < 380.0f) {
                    // Mark invisible low obstacle in the 360° World Map and back out toward open frontier!
                    worldClearanceCm[angleToBin(roverHeadingDeg)] = 20.0f;
                    motors.stop_all();
                    steerLeftActive = pickBestSteerLeft();
                    if (!sonar.isRearObstacleClose()) {
                        applySteerDirection(!steerLeftActive);
                        motors.reverse(REVERSE_SPEED_PWM);
                        navState = NavigationState::STATE_BLOCKED_REVERSE;
                    } else {
                        applySteerDirection(steerLeftActive);
                        motors.forward(turnSpeedPwm);
                        navState = NavigationState::STATE_POST_REVERSE_TURN;
                    }
                    stateEnteredMs = nowMs;
                    return;
                }
            }

            // 4. Open path (>= 60cm) -> Cruise straight ahead (or gently curve toward a wide-open frontier)
            motors.steer_center();
            motors.forward((frontDistCm < 105.0f) ? approachSpeedPwm : cruiseSpeedPwm);
            break;
        }

        case NavigationState::STATE_STEER_AROUND: {
            if (sonar.isEmergencyHazard()) {
                motors.stop_all();
                navState = NavigationState::STATE_EMERGENCY_STOP;
                stateEnteredMs = nowMs;
                return;
            }

            // Hold the turn for at least 550ms and require >= 72cm (12cm hysteresis) before re-centering
            // so the front wheels make smooth, committed curves and never chatter Left/Right!
            if ((nowMs - stateEnteredMs >= 550) && (frontDistCm >= STEER_CLEAR_RESUME_CM)) {
                motors.steer_center();
                motors.forward(cruiseSpeedPwm);
                navState = NavigationState::STATE_CRUISING;
                stateEnteredMs = nowMs;
                return;
            }

            // Corner closed in (< 42cm) -> Immediately transition to committed reverse + counter-steer
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

            // Stay committed to the chosen steering direction (no mid-turn wheel flipping!)
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

    // Enable Wireless Over-The-Air (OTA) Firmware Updates (via 192.168.4.1 or Home Wi-Fi)
    ArduinoOTA.setHostname("rc-rover");
    ArduinoOTA.onStart([]() {
        motors.stop_all();
        rgbLedWrite(STATUS_RGB_PIN, 35, 0, 35);
    });
    ArduinoOTA.begin();

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
    ArduinoOTA.handle();
    const uint32_t nowMs = millis();

    // 1. Non-blocking alternating Front + Rear ultrasonic sensor update (3-sample median filter)
    const bool gotNewPing = sonar.update();
    const float frontDistCm = sonar.getFrontDistanceCm();
    const float rearDistCm = sonar.getRearDistanceCm();

    stepPoseOdometry(nowMs, motors.getDriveDirection(), motors.getSteerDirection());

    if (gotNewPing) {
        updateSectorMemory(frontDistCm, rearDistCm, motors.getDriveDirection(), motors.getSteerDirection());
    }

    // 2. Check 30-second Boot Standby timer if user hasn't interacted via the web UI yet
    RoverMode currentMode = webUi.getMode();
    if (currentMode == RoverMode::STANDBY) {
        motors.stop_all();
        const uint32_t elapsedMs = nowMs - bootStartMs;
        if (elapsedMs >= BOOT_STANDBY_TIMEOUT_MS) {
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
            motors.playTone(1400, 75);
        }
        navState = NavigationState::STATE_CRUISING;
        stateEnteredMs = millis();
        lastMode = currentMode;
    }

    // 4. Execute active control loop
    const uint8_t throttlePwm = webUi.getThrottleTrim();
    bool manualWatchdogTripped = false;

    if (currentMode == RoverMode::AUTONOMOUS) {
        runAutonomousStateMachine(nowMs, frontDistCm, throttlePwm);
    } else if (currentMode == RoverMode::FOLLOW) {
        runFollowMeMode(frontDistCm, throttlePwm);
    } else if (currentMode == RoverMode::MANUAL) {
        runManualControl(throttlePwm, manualWatchdogTripped);
    }

    // 5. Update onboard WS2812 RGB status LED (GPIO 48) & broadcast live WebSocket telemetry + sector map
    const bool anyWatchdogTripped =
        (currentMode == RoverMode::MANUAL) ? manualWatchdogTripped : (navState == NavigationState::STATE_EMERGENCY_STOP);
    updateStatusLed(currentMode, navState, anyWatchdogTripped);

    TelemetrySnapshot snap;
    snap.frontCm = frontDistCm;
    snap.rearCm = rearDistCm;
    snap.sectorLeftCm = sectorLeftCm;
    snap.sectorCenterCm = sectorCenterCm;
    snap.sectorRightCm = sectorRightCm;
    snap.headingDeg = roverHeadingDeg;
    snap.goalSteer = currentGoalSteer;
    snap.driveDir = motors.getDriveDirection();
    snap.steerDir = motors.getSteerDirection();
    snap.stateLabel = getStateLabel(currentMode, navState, manualWatchdogTripped);
    snap.watchdogTripped = anyWatchdogTripped;
    webUi.update(snap);
}
