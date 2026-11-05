# AGENT.md — Autonomous RC Rover (`85131` Chassis) Context & Architecture

This document provides complete hardware, wiring, firmware, and operational context for AI agents and developers working on the **Autonomous RC Rover** project in `c:\Dev\Robocar`.

---

## 1. Hardware Bill of Materials (BOM)

| Item | Model / Spec | Role in System |
| :--- | :--- | :--- |
| **Chassis & Motors** | `85131` 1/16–1/18 Toy RC Sports Car (Blue Shell) | Rear 130-can drive motor + Front 2-wire spring-centered DC steering motor (with factory orange ceramic EMI suppression capacitor across steering tabs) + 5× AA (`6.0V–7.5V`) chassis battery tray & ON/OFF switch |
| **Primary Controller** | **SANXIXING ESP32-S3 DevKit N16R8** (44-pin pre-soldered, 16MB Flash, 8MB Octal PSRAM, Dual Type-C) — [Amazon](https://a.co/d/06Lic7if) | Runs non-blocking motor control, dual ultrasonic ISR driver, state machine, SoftAP (`RC-Rover` at `192.168.4.1`), and `AsyncWebServer` + `WebSocket` (`/ws`) server |
| **Companion Camera** | **Aideepen ESP32-CAM-MB** (AI-Thinker ESP32-CAM OV2640 + CH340 Micro-USB programmer baseboard) — [Amazon](https://a.co/d/0dX22e7p) | Joins `RC-Rover` Wi-Fi SoftAP as station at static IP `192.168.4.2` and streams CORS-enabled QVGA MJPEG video at `http://192.168.4.2:81/stream` |
| **Motor Driver** | **BOJACK L298N Dual H-Bridge** — [Amazon](https://a.co/d/09Fo7jR4) | Drives rear traction motor (`OUT1`/`OUT2` + `ENA`/`IN1`/`IN2`) and front steering actuator (`OUT3`/`OUT4` + `ENB`/`IN3`/`IN4`) from the car's 5× AA battery pack |
| **Ultrasonic Sensors (2×)** | **HC-SR04P** (`3.0V–5.5V` wide voltage, `2cm–450cm` range) — [Amazon](https://a.co/d/0goX3Ql8) | Native `3.3V` logic (no voltage dividers needed). Sensor #1 mounted on front bumper (`0°`); Sensor #2 mounted on rear bumper (`180°`) |
| **Compute Power Bank** | **Nimble CHAMP LITE 5K (`NB-CPC-5K`)** (`5,200mAh` / `5V 3A` shared across `USB-A` & `Type-C`) | Powers both the `ESP32-S3` and `ESP32-CAM` via a single USB cable; mounted externally on the rear deck |
| **Interconnects** | **Chanzon SYB-170 Mini Breadboards** ([Amazon](https://a.co/d/09p7nb25)) + **ELEGOO 120pcs Dupont Jumpers** ([Amazon](https://a.co/d/0frG2YbZ)) | Direct `F-F` signal/power wiring + optional `3.3V`/`GND` distribution rail |

---

## 2. Complete GPIO & Wiring Map

> **CRITICAL ELECTRICAL & BOARD RULES (See [`docs/esp32_s3_pinout.jpg`](docs/esp32_s3_pinout.jpg), [`docs/l298n_pinout.jpg`](docs/l298n_pinout.jpg), [`docs/hcsr04p_pinout.jpg`](docs/hcsr04p_pinout.jpg), [`docs/esp32_cam_mb_pinout.jpg`](docs/esp32_cam_mb_pinout.jpg)):**
> 1. **Zero-Breadboard Direct Wiring Option:** The 44-pin `YD-ESP32-S3` board has **two `3V3` pins** (Pins 1 & 2 at top-left) and **four `GND` pins** (1 at bottom-left, 1 at top-right, 2 at bottom-right), and **all 10 signal pins (`GPIO 4–13`) are on the Left Header Row**. You can connect both `HC-SR04P` sensors (`Vcc, Trig, Echo, GND` from left to right facing the silver transducers) and the `L298N` directly to the ESP32-S3 header pins using Female-to-Female (`F-F`) Dupont wires without needing a breadboard!
> 2. **Which Pin of `ENA` & `ENB` to Use on the BOJACK `HW-095` L298N:** When you pull off the two black jumper caps on `Channel A enable (ENA)` and `Channel B enable (ENB)`, each exposes **two pins side-by-side**. Connect `GPIO 8` (`ENA`) and `GPIO 9` (`ENB`) to the **OUTER pin (the pin aligned in a straight vertical row with `IN1–IN4`)**. **NEVER** connect an ESP32 GPIO to the inner pin (the inner pin toward the round capacitor is raw `+5V`!). Leave the `Onboard 5v enable` jumper behind the blue power terminal block installed.
> 3. **Powering Both Boards from the Nimble 5K (`1× USB-C` + `1× USB-A`):**
>    - **Method A (Single USB Cable + 2 Dupont Wires — Cleanest & Lightest):** Run **one USB cable** from the Nimble 5K into the **ESP32-S3**. After flashing the `ESP32-CAM` on your PC using its `MB` Micro-USB baseboard, you can unplug the `MB` baseboard and power the `ESP32-CAM` directly from the `ESP32-S3` using just **two Female-to-Female (`F-F`) Dupont wires**: **`ESP32-S3 5V0` (Left Pin 21) → `ESP32-CAM 5V`** and **`ESP32-S3 GND` (Right Pin 22) → `ESP32-CAM GND`**. This saves space inside the cabin and eliminates a second bulky USB cable!
>    - **Method B (Two USB Cables):** Plug a **USB-C to USB-C** cable from the Nimble's `USB-C` port into the **ESP32-S3**, and a **USB-A to Micro-USB** cable from the Nimble's `USB-A` port into the **ESP32-CAM-MB**.
> 4. **Onboard WS2812 RGB Status LED (`GPIO 48`):** Pre-soldered by default (`Soldered (Default)` jumper). Firmware drives `GPIO 48` to indicate live state: **Green** = Cruising, **Amber** = Steer-Around (`28–65 cm`), **Orange-Red** = Reversing, **Red** = Emergency/Watchdog Stop, **Purple** = Follow-Me, **Blue** = Manual D-Pad.
> 5. **Common Ground (`GND`):** The L298N middle screw terminal (`GND`) **MUST** be connected to both the Car Switch Negative (`-`) lead **AND** one of the four `ESP32-S3 GND` pins.
> 6. **Keep Factory Steering Capacitor:** Do not remove the orange ceramic capacitor soldered across the front steering motor tabs; it suppresses brush EMI noise.

| Module | Pin | Connected To | Notes |
| :--- | :--- | :--- | :--- |
| **L298N** | `12V` | Car Battery Switch (`+`) | `6.0V–7.5V` from 5× AA compartment |
| **L298N** | `GND` | Car Battery Switch (`-`) + Breadboard `GND` | Shared ground with ESP32-S3 `GND` |
| **L298N** | `OUT1` / `OUT2` | Rear Drive Motor Wires | Swap `OUT1`/`OUT2` if Forward runs in Reverse |
| **L298N** | `OUT3` / `OUT4` | Front Steering Motor Wires | Swap `OUT3`/`OUT4` if Left steers Right |
| **L298N** | `IN1` | `ESP32-S3 GPIO 4` | Drive Forward direction |
| **L298N** | `IN2` | `ESP32-S3 GPIO 5` | Drive Reverse direction |
| **L298N** | `IN3` | `ESP32-S3 GPIO 6` | Steer Left command |
| **L298N** | `IN4` | `ESP32-S3 GPIO 7` | Steer Right command |
| **L298N** | `ENA` | `ESP32-S3 GPIO 8` | Rear drive speed PWM (`5 kHz`, 8-bit) |
| **L298N** | `ENB` | `ESP32-S3 GPIO 9` | Front steering PWM cap (`DEFAULT_STEER_PWM = 200`) |
| **Front HC-SR04P** | `VCC` / `GND` | Breadboard `3V3` / `GND` | Powered from ESP32-S3 `3V3` pin |
| **Front HC-SR04P** | `TRIG` | `ESP32-S3 GPIO 10` | `10µs` trigger output |
| **Front HC-SR04P** | `ECHO` | `ESP32-S3 GPIO 11` | Non-blocking `CHANGE` ISR input |
| **Rear HC-SR04P** | `VCC` / `GND` | Breadboard `3V3` / `GND` | Powered from ESP32-S3 `3V3` pin |
| **Rear HC-SR04P** | `TRIG` | `ESP32-S3 GPIO 12` | `10µs` trigger output |
| **Rear HC-SR04P** | `ECHO` | `ESP32-S3 GPIO 13` | Non-blocking `CHANGE` ISR input |

| **ESP32-CAM** | `5V` | `ESP32-S3 5V0` (Left Pin 21) | Direct `F-F` jumper (MB USB baseboard removed after flashing) |
| **ESP32-CAM** | `GND` | `ESP32-S3 GND` (Right Pin 22) | Direct `F-F` jumper |

---

## 3. Physical Layout on the `85131` Car

See [`docs/rover_hardware_layout.png`](docs/rover_hardware_layout.png) for the annotated visual placement map:
- **External Rear Deck (`Zone B / 5`):** Mount the **Nimble CHAMP LITE 5K** USB power bank externally on the rear deck/rear window with double-sided foam tape (or heavy-duty Velcro) and pass the single USB cable into the cabin through the rear body/window opening (the black rear window insert is held by 4 Phillips screws and can be unscrewed or notched so you don't even have to drill the blue shell). This leaves 100% of the interior cabin free for the electronics and lets you press the Nimble power button and recharge it without opening the shell!
- **Blue Shell Front Windshield (`Zone A`):** Unscrew the 4 Phillips screws holding the black front windshield insert (`Zone A`) so the bare **ESP32-CAM** (with its `MB` programmer board unplugged after flashing, powered via `5V0`/`GND` jumpers from the ESP32-S3) can look straight out through the windshield opening.
- **Black Chassis (Bottom):**
  - **Front Bumper (`Zone 1`):** Mount Front `HC-SR04P` pointing straight ahead (`0°`).
  - **Center Deck (`Zone 3 & 4`):** Remove the stock green RC receiver PCB and antenna wire. Mount the `L298N` and the `ESP32-S3` in the spacious center cabin area.
  - **Rear Bumper (`Zone 6`):** Mount Rear `HC-SR04P` pointing straight back (`180°`).

---

## 4. Repository Structure & Firmware Modules

- [`platformio.ini`](platformio.ini): Defines two build environments:
  1. `[env:esp32-s3-devkitc-1]` — Primary rover controller (`src/main.cpp`, `src/motor_controller.*`, `src/sonar_driver.*`, `src/web_interface.*`).
  2. `[env:esp32cam]` — Companion camera streamer (`src/cam_main.cpp`).
- [`src/motor_controller.h`](src/motor_controller.h) & [`src/motor_controller.cpp`](src/motor_controller.cpp):
  - Hardware PWM via `ledcAttach` (ESP32 Arduino Core v3.x) with automatic preprocessor fallback for Core v2.x.
  - **ESC-Style Motor-Winding Chimes (`playTone`, `playBootChime`, `playRoamLaunchChime`):** Vibrates the rear DC motor windings at musical audio frequencies (`880 Hz – 1760 Hz`) at a low sub-rolling duty cycle (`55/255`) so the motor itself sings audible startup and roam-launch beeps without moving the wheels or needing a physical buzzer.
- [`src/sonar_driver.h`](src/sonar_driver.h) & [`src/sonar_driver.cpp`](src/sonar_driver.cpp):
  - Non-blocking dual-sensor driver that alternates pings between Front (`GPIO 10/11`) and Rear (`GPIO 12/13`) every `30ms` (`60ms` full cycle per sensor) to prevent indoor acoustic crosstalk, with a 3-sample median filter per channel.
  - Gracefully ignores the rear sensor if unplugged (`_rear.hasValidReading == false`) so bench testing with 1 sensor never falsely blocks reversing.
- [`src/main.cpp`](src/main.cpp):
  - **Boot Standby Countdown (`RoverMode::STANDBY` — First 30 Seconds):**
    - Boots immediately when the USB cable is plugged into the Nimble 5K battery pack, plays a 3-note rising chime (`playBootChime`), pulses the onboard RGB LED Cyan, and waits stationary for `30 seconds` (`STANDBY (30s)...`).
    - If the user interacts with the web dashboard (`http://192.168.4.1`) during those 30 seconds, it plays a confirmation pip and switches immediately to the selected mode.
    - If no web input is received within `30 seconds`, it plays an alert chime (`playRoamLaunchChime`) and automatically transitions into `RoverMode::AUTONOMOUS` (`Auto Steer` roaming).
  - **Mode 0 — `AUTONOMOUS` (Steer-First Avoidance):**
    - `Front >= 65 cm`: Cruise straight forward (`STATE_CRUISING`).
    - `28 cm <= Front < 65 cm`: Keep driving forward while steering Left/Right (`STATE_STEER_AROUND`). Automatically flips steering direction if distance shrinks by `> 6 cm` after `350ms`.
    - `Front < 28 cm`: No forward path left (`STATE_BLOCKED_REVERSE`). Counter-steers while reversing for up to `350ms` (cuts reverse early if Rear Sonar reads `< 15 cm`), then turns forward (`STATE_POST_REVERSE_TURN`).
    - Emergency watchdog halts immediately (`STATE_EMERGENCY_STOP`) on consecutive `< 8 cm` readings or front sensor echo timeout fault.
  - **Mode 1 — `FOLLOW` (Camera + Sonar Follow-Me):**
    - Steers toward the browser-locked target (`TARGET:-1/0/1`) and uses Front/Rear sonar to hold a `30–50 cm` standoff distance (advancing up to `160 cm`, holding at `30–50 cm`, and backing off below `24 cm` if Rear Sonar is clear).
  - **Mode 2 — `MANUAL` (Touch D-Pad with 500ms Watchdog):**
    - 9-way directional control (`FWD`, `REV`, `LEFT`, `RIGHT`, and diagonals) with a strict `500ms` heartbeat failsafe watchdog (`WebInterface::isManualWatchdogExpired`).
- [`src/web_interface.h`](src/web_interface.h) & [`src/web_interface.cpp`](src/web_interface.cpp):
  - Hosts SoftAP `RC-Rover` (password `rover1234`, IP `192.168.4.1`) and serves the single-page mobile Safari dashboard + `/ws` WebSocket endpoint.
- [`src/cam_main.cpp`](src/cam_main.cpp):
  - Connects the `ESP32-CAM-MB` to `RC-Rover` at `192.168.4.2` and serves `http://192.168.4.2:81/stream` with `Access-Control-Allow-Origin: *` so the dashboard `<canvas>` can perform client-side color-centroid target tracking.

---

## 5. Development & Browser Support Conventions

- **Browser Target:** Mobile Safari (iPhone 7 / iOS 15+ through modern Safari/Chrome). Always include `@supports not` and JS custom-property fallbacks for newer CSS features (`progress()`, typed `attr()`).
- **Git Workflow:** Never run `git push` without explicit user permission in chat.
