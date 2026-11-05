# Autonomous RC Rover (ESP32-S3 + ESP32-CAM-MB + L298N + Dual HC-SR04P)

Autonomous and Wi-Fi-teleoperated RC rover firmware built with PlatformIO and the Arduino framework targeting:
1. **Primary Rover Controller:** SANXIXING **ESP32-S3 DevKit (N16R8)** (`192.168.4.1`)
2. **Companion FPV Camera:** Aideepen **ESP32-CAM-MB (OV2640)** (`192.168.4.2:81/stream`)

See **[agent.md](agent.md)** for full hardware context, assembly notes, and state-machine architecture.

![Hardware & Sensor Placement Map](docs/rover_hardware_layout.png)

---

## 1. Hardware Pinout Mapping

| Component | Component Pin | ESP32-S3 Pin (`YD-ESP32-S3` 44-Pin) | Description |
| :--- | :--- | :--- | :--- |
| **L298N** | `12V` (Power In) | Car Switch (`+`) | 5× AA Battery power (`6.0V–7.5V`) |
| **L298N** | `GND` | Car Switch (`-`) & ESP32 `GND` (Left Pin 22) | **Mandatory shared common ground** |
| **L298N** | `OUT1` / `OUT2` | Rear Motor Leads A / B | 130-can rear traction DC motor |
| **L298N** | `OUT3` / `OUT4` | Front Steering Leads A / B | Spring-centered front steering actuator |
| **L298N** | `IN1` | `GPIO 4` (Left Pin 4) | Drive motor direction forward |
| **L298N** | `IN2` | `GPIO 5` (Left Pin 5) | Drive motor direction reverse |
| **L298N** | `IN3` | `GPIO 6` (Left Pin 6) | Steering left command |
| **L298N** | `IN4` | `GPIO 7` (Left Pin 7) | Steering right command |
| **L298N** | `ENA` | `GPIO 8` (Left Pin 12) | Rear motor PWM speed control *(remove jumper)* |
| **L298N** | `ENB` | `GPIO 9` (Left Pin 15) | Steering PWM power cap *(remove jumper)* |
| **Front HC-SR04P** | `VCC` / `GND` | `3V3` (Left Pin 1) / `GND` (Right Pin 1) | Direct `F-F` jumper to ESP32-S3 pins |
| **Front HC-SR04P** | `TRIG` | `GPIO 10` (Left Pin 16) | Front 10µs trigger pulse output |
| **Front HC-SR04P** | `ECHO` | `GPIO 11` (Left Pin 17) | Front echo pulse input (`28–65 cm` steer around) |
| **Rear HC-SR04P** | `VCC` / `GND` | `3V3` (Left Pin 2) / `GND` (Right Pin 21) | Direct `F-F` jumper to ESP32-S3 pins |
| **Rear HC-SR04P** | `TRIG` | `GPIO 12` (Left Pin 18) | Rear 10µs trigger pulse output |
| **Rear HC-SR04P** | `ECHO` | `GPIO 13` (Left Pin 19) | Rear echo pulse input (`< 15 cm` reverse stop) |
| **Onboard RGB LED**| `WS2812` | `GPIO 48` (Onboard) | Green=Cruise, Amber=Steer, Red=Stop, Purple=Follow |
| **ESP32-CAM** | `5V` / `GND` | `5V0` (Left Pin 21) / `GND` (Right Pin 22) | Powered via 2 `F-F` jumpers from ESP32-S3 (`192.168.4.2`) |

*(See [`docs/esp32_s3_pinout.jpg`](docs/esp32_s3_pinout.jpg), [`docs/l298n_pinout.jpg`](docs/l298n_pinout.jpg), [`docs/hcsr04p_pinout.jpg`](docs/hcsr04p_pinout.jpg), and [`docs/esp32_cam_mb_pinout.jpg`](docs/esp32_cam_mb_pinout.jpg) for exact board pinout diagrams.)*

---

## 2. Firmware Architecture

- **[src/motor_controller.h](src/motor_controller.h) / [src/motor_controller.cpp](src/motor_controller.cpp)**:
  - Drives the L298N H-Bridge using ESP32 hardware LEDC PWM (`ledcAttach` on Arduino Core v3.x with automatic fallback for Core v2.x).
  - Caps front steering PWM at `200` (max `220`) to protect the 2-wire spring-centered steering actuator.
- **[src/sonar_driver.h](src/sonar_driver.h) / [src/sonar_driver.cpp](src/sonar_driver.cpp)**:
  - Non-blocking dual-channel interrupt-driven (`CHANGE` ISRs on `GPIO 11` and `GPIO 13`) ultrasonic driver alternating every `30ms` with 3-sample median filtering.
- **[src/web_interface.h](src/web_interface.h) / [src/web_interface.cpp](src/web_interface.cpp)**:
  - Hosts the `RC-Rover` SoftAP (`192.168.4.1`) and serves the mobile Safari dashboard with a live `<canvas>` HUD that overlays **Front/Rear Sonar Obstacle Zones** and **Tap-to-Lock Target Tracking** directly onto the `192.168.4.2:81/stream` video feed.
- **[src/main.cpp](src/main.cpp)**:
  - **Auto Steer Mode:** Steers smoothly around obstacles in the `28–65 cm` range while continuing forward, and only reverses (`< 28 cm`, guarded by the Rear Sonar `< 15 cm`) when no forward path exists.
  - **Follow-Me Mode:** Steers toward the camera-locked target (`LEFT`/`CENTER`/`RIGHT`) and uses the Front & Rear sensors to hold a `30–50 cm` standoff distance.
  - **Manual D-Pad Mode:** 9-way touch pad with a `500ms` heartbeat failsafe watchdog.
- **[src/cam_main.cpp](src/cam_main.cpp)**:
  - Companion firmware for the **Aideepen ESP32-CAM-MB**. Automatically connects to the `RC-Rover` Wi-Fi network at static IP `192.168.4.2` and streams CORS-enabled (`Access-Control-Allow-Origin: *`) QVGA MJPEG video on port `81` (`/stream`).

---

## 3. Flashing Both Boards with PlatformIO

### Step A: Flash the ESP32-S3 Rover Controller
Plug the **SANXIXING ESP32-S3** into your computer via USB-C and run:
```powershell
pio run -e esp32-s3-devkitc-1 --target upload
```

### Step B: Flash the Aideepen ESP32-CAM-MB
Snap the **ESP32-CAM** onto its included **ESP32-CAM-MB USB programmer baseboard**, plug it into your computer via Micro-USB, and run:
```powershell
pio run -e esp32cam --target upload
```

### Step C: Drive!
1. Plug both the **ESP32-S3** and the **ESP32-CAM-MB** into your **Nimble CHAMP LITE 5K** dual-port USB battery pack.
2. Join the **`RC-Rover`** Wi-Fi network (password: **`rover1234`**) on your phone and open **`http://192.168.4.1`** in Safari.
3. The live camera stream and obstacle/target HUD will appear automatically at the top of the dashboard.
